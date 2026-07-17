using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// PPO Agent for v2 dual-contact grasp training with UR5e + DG5F.
    /// This component is the sole xDrive writer in the training scene.
    /// </summary>
    public sealed class Dg5fGraspAgent : Agent
    {
        [Header("Scene references")]
        public Rigidbody ball;
        public Transform pedestal;
        public Collider pedestalCollider;
        public Transform robotBase;
        public Transform palm;
        public Transform graspPoint;
        public Transform[] fingerTips = new Transform[Dg5fGraspSpec.FingerCount];
        public GraspContactSensor[] contactSensors = new GraspContactSensor[Dg5fGraspSpec.FingerCount];

        [Header("Episode")]
        public bool useDeterministicSpawns;
        public int spawnSeed = 12345;

        [Header("Control")]
        public float armDeltaDegPerDecision = 2f;
        public float gripDeltaPerDecision = 0.04f;

        readonly Dictionary<ArticulationBody, float> _initialTargetDeg =
            new Dictionary<ArticulationBody, float>();

        ArticulationBody[] _allJoints;
        ArticulationBody[] _armJoints;
        ArticulationBody[] _handJoints;
        float[] _armTargetDeg;
        float _closure;
        float _initialBallHeight;
        float _supportTopHeight;
        float _previousApproachPotential;
        float _previousContactPotential;
        float _previousContactHoldPotential;
        float _bestGraspDistance;
        float _episodeSeconds;
        float _contactHoldSeconds;
        float _maxContactHoldSeconds;
        float _firstReachSeconds;
        int _ballReleaseFixedSteps;
        int _evaluationEpisodeId = -1;
        bool _episodeActive;
        bool _reachSucceeded;
        bool _thumbContactReached;
        bool _opposingContactReached;
        bool _dualContactReached;
        readonly bool[] _fingerContacts = new bool[Dg5fGraspSpec.FingerCount];
        System.Random _random;
        StatsRecorder _stats;

        public float CurrentClosure => _closure;
        public float CurrentSupportTopHeight => _supportTopHeight;
        public Vector3 CurrentBallLocalPosition => robotBase.InverseTransformPoint(ball.position);
        public float CurrentEpisodeSeconds => _episodeSeconds;
        public float CurrentGraspDistance => GraspDistance();
        public float BestGraspDistance => _bestGraspDistance;
        public bool IsEpisodeActive => _episodeActive;
        public bool ReachSucceeded => _reachSucceeded;
        public float FirstReachSeconds => _firstReachSeconds;
        public float CurrentContactHoldSeconds => _contactHoldSeconds;
        public float MaxContactHoldSeconds => _maxContactHoldSeconds;
        public bool HasThumbContact => Dg5fGraspSpec.HasThumbContact(_fingerContacts);
        public bool HasOpposingContact => Dg5fGraspSpec.HasOpposingContact(_fingerContacts);
        public bool HasDualContact => Dg5fGraspSpec.HasDualContact(_fingerContacts);

        public float CurrentArmTargetDeg(int index)
        {
            if (_armTargetDeg == null) throw new InvalidOperationException("Agent has not initialized.");
            return _armTargetDeg[index];
        }

        public override void Initialize()
        {
            ResolveReferences();
            ResolveJoints();
            ValidateConfiguration();

            _armTargetDeg = new float[Dg5fGraspSpec.ArmJointCount];
            foreach (var body in _allJoints)
                _initialTargetDeg[body] = body.xDrive.target;

            // The exact 20-second limit is measured in simulation time.
            MaxStep = 0;
            _random = new System.Random(spawnSeed);
            _stats = Academy.Instance.StatsRecorder;
            Dg5fEvaluationSession.Register(this);
        }

        void ResolveReferences()
        {
            if (robotBase == null) robotBase = transform;
            if (pedestal == null)
            {
                var found = GameObject.Find("GraspPanel") ?? GameObject.Find("GraspPedestal");
                if (found != null) pedestal = found.transform;
            }
            if (pedestalCollider == null && pedestal != null)
                pedestalCollider = pedestal.GetComponent<Collider>();

            var transforms = GetComponentsInChildren<Transform>(true);
            if (palm == null) palm = FindByName(transforms, "ll_dg_palm");
            if (graspPoint == null) graspPoint = FindByName(transforms, "GraspPoint");

            for (int finger = 0; finger < Dg5fGraspSpec.FingerCount; finger++)
            {
                string tipName = $"ll_dg_{finger + 1}_tip";
                if (fingerTips == null || fingerTips.Length != Dg5fGraspSpec.FingerCount)
                    fingerTips = new Transform[Dg5fGraspSpec.FingerCount];
                if (fingerTips[finger] == null) fingerTips[finger] = FindByName(transforms, tipName);
            }

            if (contactSensors == null || contactSensors.Length != Dg5fGraspSpec.FingerCount)
                contactSensors = new GraspContactSensor[Dg5fGraspSpec.FingerCount];
            for (int finger = 0; finger < contactSensors.Length; finger++)
            {
                if (contactSensors[finger] == null && fingerTips[finger] != null)
                    contactSensors[finger] = fingerTips[finger].GetComponent<GraspContactSensor>();
                if (contactSensors[finger] != null) contactSensors[finger].targetBall = ball;
            }
        }

        static Transform FindByName(IEnumerable<Transform> transforms, string name)
        {
            foreach (var item in transforms)
                if (item.name == name) return item;
            return null;
        }

        void ResolveJoints()
        {
            var bodies = GetComponentsInChildren<ArticulationBody>(true);
            var revolute = new List<ArticulationBody>();
            foreach (var body in bodies)
                if (body.jointType == ArticulationJointType.RevoluteJoint && body.dofCount > 0)
                    revolute.Add(body);
            _allJoints = revolute.ToArray();

            _armJoints = new ArticulationBody[Dg5fGraspSpec.ArmJointCount];
            for (int i = 0; i < _armJoints.Length; i++)
                _armJoints[i] = FindBody(bodies, Dg5fGraspSpec.ArmLinks[i]);

            _handJoints = new ArticulationBody[Dg5fGraspSpec.HandJointCount];
            for (int finger = 1; finger <= 5; finger++)
                for (int joint = 1; joint <= 4; joint++)
                {
                    int channel = (finger - 1) * 4 + joint - 1;
                    _handJoints[channel] = FindBodyBySuffix(bodies, $"_dg_{finger}_{joint}");
                }
        }

        static ArticulationBody FindBody(IEnumerable<ArticulationBody> bodies, string name)
        {
            foreach (var body in bodies)
                if (body.name == name) return body;
            return null;
        }

        static ArticulationBody FindBodyBySuffix(IEnumerable<ArticulationBody> bodies, string suffix)
        {
            foreach (var body in bodies)
                if (body.name.EndsWith(suffix, StringComparison.Ordinal)) return body;
            return null;
        }

        void ValidateConfiguration()
        {
            if (ball == null || pedestal == null || pedestalCollider == null
                || robotBase == null || palm == null || graspPoint == null)
            {
                throw new InvalidOperationException(
                    "[Dg5fGraspAgent] Missing ball/pedestal/robotBase/palm/graspPoint reference.");
            }
            for (int i = 0; i < _armJoints.Length; i++)
                if (_armJoints[i] == null)
                    throw new InvalidOperationException($"[Dg5fGraspAgent] Missing arm joint: {Dg5fGraspSpec.ArmLinks[i]}");
            for (int i = 0; i < _handJoints.Length; i++)
                if (_handJoints[i] == null)
                    throw new InvalidOperationException($"[Dg5fGraspAgent] Missing hand joint channel {i}.");
            for (int i = 0; i < Dg5fGraspSpec.FingerCount; i++)
                if (fingerTips[i] == null || contactSensors[i] == null)
                    throw new InvalidOperationException($"[Dg5fGraspAgent] Missing fingertip/contact sensor {i}.");
                else if (contactSensors[i].fingerIndex != i)
                    throw new InvalidOperationException(
                        $"[Dg5fGraspAgent] Contact sensor slot {i} has finger index "
                        + $"{contactSensors[i].fingerIndex}.");
        }

        public override void OnEpisodeBegin()
        {
            _episodeActive = false;
            if (Dg5fEvaluationSession.IsEnabled)
            {
                if (!Dg5fEvaluationSession.TryBeginEpisode(this, out _evaluationEpisodeId, out int evaluationSeed))
                {
                    DisableAfterEvaluationQuota();
                    return;
                }
                _random = new System.Random(evaluationSeed);
                useDeterministicSpawns = true;
            }
            else
            {
                _evaluationEpisodeId = -1;
            }

            _closure = 0f;
            ResetRobot();
            ResetBall();
            foreach (var sensor in contactSensors) sensor.ResetContacts();

            _episodeSeconds = 0f;
            _bestGraspDistance = GraspDistance();
            _previousApproachPotential = Dg5fGraspSpec.ApproachPotential(_bestGraspDistance);
            _previousContactPotential = 0f;
            _previousContactHoldPotential = 0f;
            _contactHoldSeconds = 0f;
            _maxContactHoldSeconds = 0f;
            _firstReachSeconds = -1f;
            _reachSucceeded = false;
            _thumbContactReached = false;
            _opposingContactReached = false;
            _dualContactReached = false;
            Array.Clear(_fingerContacts, 0, _fingerContacts.Length);
            _episodeActive = true;
        }

        void DisableAfterEvaluationQuota()
        {
            _episodeActive = false;
            enabled = false;
            var requester = GetComponent<Unity.MLAgents.DecisionRequester>();
            if (requester != null) requester.enabled = false;
        }

        void ResetRobot()
        {
            foreach (var body in _allJoints)
            {
                float targetDeg = _initialTargetDeg[body];
                var drive = body.xDrive;
                drive.target = targetDeg;
                body.xDrive = drive;
                body.jointPosition = new ArticulationReducedSpace(targetDeg * Mathf.Deg2Rad);
                body.jointVelocity = new ArticulationReducedSpace(0f);
            }

            for (int i = 0; i < _armJoints.Length; i++)
            {
                float initial = _initialTargetDeg[_armJoints[i]];
                _armTargetDeg[i] = Mathf.Clamp(
                    initial,
                    Dg5fGraspSpec.ArmSafeMinDeg[i],
                    Dg5fGraspSpec.ArmSafeMaxDeg[i]);
            }
            ApplyArmTargets();
            ApplyGripTargets();
        }

        void ResetBall()
        {
            Collider ballCollider = ball.GetComponent<Collider>();
            if (ballCollider == null)
                throw new InvalidOperationException("[Dg5fGraspAgent] Ball requires a collider.");
            float ballRadius = ballCollider.bounds.extents.y;
            Vector3 ballLocalPosition = Dg5fGraspSpec.SpawnBallLocalPosition(
                Next01(),
                Next01(),
                ballRadius);
            if (!Dg5fGraspSpec.IsValidSpawn(ballLocalPosition, ballRadius))
                throw new InvalidOperationException("[Dg5fGraspAgent] Generated an invalid v1 spawn pose.");

            _supportTopHeight = Dg5fGraspSpec.SupportTopHeight;

            if (!ball.isKinematic)
            {
                ball.linearVelocity = Vector3.zero;
                ball.angularVelocity = Vector3.zero;
            }
            ball.isKinematic = true;
            ball.useGravity = false;
            ball.position = robotBase.TransformPoint(ballLocalPosition);
            ball.rotation = robotBase.rotation * Quaternion.Euler(0f, Next01() * 360f, 0f);
            Physics.SyncTransforms();

            _initialBallHeight = ball.position.y;
            // Articulation collider transforms lag direct jointPosition writes by one
            // physics step. Keep the ball kinematic for that step, then release it.
            _ballReleaseFixedSteps = 2;
        }

        float Next01()
        {
            if (!useDeterministicSpawns) return UnityEngine.Random.value;
            return (float)_random.NextDouble();
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // ML-Agents can request one final observation while a scene is unloading.
            // Preserve the fixed contract instead of indexing disposed articulation state.
            if (ball == null || robotBase == null || palm == null || graspPoint == null
                || _armJoints == null || fingerTips == null || contactSensors == null)
            {
                for (int i = 0; i < Dg5fGraspSpec.ObservationSize; i++) sensor.AddObservation(0f);
                return;
            }
            if (!HasFinitePhysicsState())
            {
                for (int i = 0; i < Dg5fGraspSpec.ObservationSize; i++) sensor.AddObservation(0f);
                return;
            }

            // 0..11: normalized arm position and velocity.
            for (int i = 0; i < _armJoints.Length; i++)
            {
                float positionDeg = FirstOrZero(_armJoints[i].jointPosition) * Mathf.Rad2Deg;
                sensor.AddObservation(Dg5fGraspSpec.NormalizeJoint(
                    positionDeg,
                    Dg5fGraspSpec.ArmSafeMinDeg[i],
                    Dg5fGraspSpec.ArmSafeMaxDeg[i]));
            }
            for (int i = 0; i < _armJoints.Length; i++)
                sensor.AddObservation(Mathf.Clamp(FirstOrZero(_armJoints[i].jointVelocity) / Mathf.PI, -1f, 1f));

            // 12: normalized grip closure.
            sensor.AddObservation(_closure * 2f - 1f);

            // 13..21: ball state in robot-base coordinates.
            AddClampedVector(sensor, robotBase.InverseTransformDirection(ball.position - graspPoint.position), 1f);
            AddClampedVector(sensor, robotBase.InverseTransformDirection(ball.linearVelocity), 2f);
            AddClampedVector(sensor, robotBase.InverseTransformDirection(ball.angularVelocity), 10f);

            // 22: vertical displacement from the episode spawn pose.
            sensor.AddObservation(Mathf.Clamp((ball.position.y - _initialBallHeight) / 0.2f, -1f, 1f));

            // 23..37: each fingertip relative to the ball in palm coordinates.
            for (int i = 0; i < fingerTips.Length; i++)
                AddClampedVector(sensor, palm.InverseTransformDirection(fingerTips[i].position - ball.position), 0.2f);

            // 38..42: thumb/index/middle/ring/pinky contacts.
            for (int i = 0; i < contactSensors.Length; i++)
                sensor.AddObservation(contactSensors[i].IsTouching ? 1f : 0f);

            // 43..48: normalized commanded arm xDrive targets.
            for (int i = 0; i < _armTargetDeg.Length; i++)
                sensor.AddObservation(Dg5fGraspSpec.NormalizeJoint(
                    _armTargetDeg[i],
                    Dg5fGraspSpec.ArmSafeMinDeg[i],
                    Dg5fGraspSpec.ArmSafeMaxDeg[i]));

            // 49..52: v1/v2/v3/v4 objective one-hot. v2 is Grasp.
            sensor.AddObservation(0f);
            sensor.AddObservation(1f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);

            // 53..56: forward-compatible task progress slots.
            sensor.AddObservation(Dg5fGraspSpec.ApproachPotential(GraspDistance()));
            sensor.AddObservation(Dg5fGraspSpec.ApproachPotential(_bestGraspDistance));
            sensor.AddObservation(Dg5fGraspSpec.ApproachSuccessDistance / Dg5fGraspSpec.MaximumBallDistance);
            sensor.AddObservation(Mathf.Clamp01(
                _episodeSeconds / Dg5fGraspSpec.EpisodeTimeoutSeconds));
        }

        static void AddClampedVector(VectorSensor sensor, Vector3 value, float scale)
        {
            sensor.AddObservation(Mathf.Clamp(value.x / scale, -1f, 1f));
            sensor.AddObservation(Mathf.Clamp(value.y / scale, -1f, 1f));
            sensor.AddObservation(Mathf.Clamp(value.z / scale, -1f, 1f));
        }

        static float FirstOrZero(ArticulationReducedSpace values)
        {
            try
            {
                return values[0];
            }
            catch (IndexOutOfRangeException)
            {
                return 0f;
            }
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            var continuous = actions.ContinuousActions;
            if (continuous.Length != Dg5fGraspSpec.ActionSize)
                throw new InvalidOperationException($"Expected {Dg5fGraspSpec.ActionSize} continuous actions, got {continuous.Length}.");
            if (!_episodeActive || _ballReleaseFixedSteps > 0) return;

            AddReward(Dg5fGraspSpec.DecisionTimePenalty);
            ScoreRewardPotentials();

            for (int i = 0; i < _armTargetDeg.Length; i++)
            {
                float delta = Mathf.Clamp(continuous[i], -1f, 1f) * armDeltaDegPerDecision;
                _armTargetDeg[i] = Mathf.Clamp(
                    _armTargetDeg[i] + delta,
                    Dg5fGraspSpec.ArmSafeMinDeg[i],
                    Dg5fGraspSpec.ArmSafeMaxDeg[i]);
            }
            _closure = Mathf.Clamp01(
                _closure + Mathf.Clamp(continuous[6], -1f, 1f) * gripDeltaPerDecision);

            ApplyArmTargets();
            ApplyGripTargets();
        }

        void ApplyArmTargets()
        {
            for (int i = 0; i < _armJoints.Length; i++)
            {
                var drive = _armJoints[i].xDrive;
                drive.target = Mathf.Clamp(_armTargetDeg[i], drive.lowerLimit, drive.upperLimit);
                _armJoints[i].xDrive = drive;
            }
        }

        void ApplyGripTargets()
        {
            for (int i = 0; i < _handJoints.Length; i++)
            {
                var drive = _handJoints[i].xDrive;
                float target = Dg5fGraspSpec.GripTargetDeg(i, _closure);
                drive.target = Mathf.Clamp(target, drive.lowerLimit, drive.upperLimit);
                _handJoints[i].xDrive = drive;
            }
        }

        void FixedUpdate()
        {
            if (!_episodeActive || ball == null || robotBase == null) return;

            if (_ballReleaseFixedSteps > 0)
            {
                _ballReleaseFixedSteps--;
                if (_ballReleaseFixedSteps == 0) ReleaseBall();
                return;
            }

            Vector3 ballLocalPosition = robotBase.InverseTransformPoint(ball.position);
            if (!HasFinitePhysicsState())
            {
                FinishEpisode(false, "NonFinitePhysics");
                return;
            }

            if (Dg5fGraspSpec.ShouldResetForBall(ballLocalPosition, _supportTopHeight))
            {
                FinishEpisode(false, "BallOutOfBounds");
                return;
            }

            _episodeSeconds += Time.fixedDeltaTime;
            float distance = GraspDistance();
            _bestGraspDistance = Mathf.Min(_bestGraspDistance, distance);
            if (!_reachSucceeded && Dg5fGraspSpec.HasReachedApproachTarget(distance))
            {
                _reachSucceeded = true;
                _firstReachSeconds = _episodeSeconds;
            }

            UpdateContactState(Time.fixedDeltaTime);
            if (Dg5fGraspSpec.HasHeldDualContact(_contactHoldSeconds))
            {
                FinishEpisode(true, "None");
                return;
            }

            if (Dg5fGraspSpec.ReachedEpisodeTimeout(_episodeSeconds))
                FinishEpisode(false, "Timeout");
        }

        void ReleaseBall()
        {
            ball.isKinematic = false;
            ball.useGravity = true;
            ball.linearVelocity = Vector3.zero;
            ball.angularVelocity = Vector3.zero;
            _initialBallHeight = ball.position.y;
            _bestGraspDistance = GraspDistance();
            _previousApproachPotential = Dg5fGraspSpec.ApproachPotential(_bestGraspDistance);
            _previousContactPotential = 0f;
            _previousContactHoldPotential = 0f;
        }

        void UpdateContactState(float deltaSeconds)
        {
            for (int index = 0; index < _fingerContacts.Length; index++)
                _fingerContacts[index] = contactSensors[index].IsTouching;

            bool thumbContact = Dg5fGraspSpec.HasThumbContact(_fingerContacts);
            bool opposingContact = Dg5fGraspSpec.HasOpposingContact(_fingerContacts);
            bool dualContact = thumbContact && opposingContact;
            _thumbContactReached |= thumbContact;
            _opposingContactReached |= opposingContact;
            _dualContactReached |= dualContact;
            _contactHoldSeconds = Dg5fGraspSpec.NextContactHoldSeconds(
                _contactHoldSeconds,
                dualContact,
                deltaSeconds);
            _maxContactHoldSeconds = Mathf.Max(_maxContactHoldSeconds, _contactHoldSeconds);
        }

        void ScoreRewardPotentials()
        {
            float distance = GraspDistance();
            float currentPotential = Dg5fGraspSpec.ApproachPotential(distance);
            AddReward(Dg5fGraspSpec.ApproachRewardScale
                * Dg5fGraspSpec.PotentialDelta(_previousApproachPotential, currentPotential));
            _previousApproachPotential = currentPotential;
            if (Dg5fGraspSpec.IsFinite(distance))
                _bestGraspDistance = Mathf.Min(_bestGraspDistance, distance);

            float currentContactPotential = Dg5fGraspSpec.ContactPotential(
                Dg5fGraspSpec.HasThumbContact(_fingerContacts),
                Dg5fGraspSpec.HasOpposingContact(_fingerContacts));
            AddReward(Dg5fGraspSpec.PotentialDelta(
                _previousContactPotential,
                currentContactPotential));
            _previousContactPotential = currentContactPotential;

            float currentHoldPotential = Dg5fGraspSpec.ContactHoldPotential(_contactHoldSeconds);
            AddReward(Dg5fGraspSpec.PotentialDelta(
                _previousContactHoldPotential,
                currentHoldPotential));
            _previousContactHoldPotential = currentHoldPotential;
        }

        bool HasFinitePhysicsState()
        {
            if (!Dg5fGraspSpec.IsFinite(ball.position)
                || !Dg5fGraspSpec.IsFinite(ball.linearVelocity)
                || !Dg5fGraspSpec.IsFinite(ball.angularVelocity))
            {
                return false;
            }

            Quaternion rotation = ball.rotation;
            if (!Dg5fGraspSpec.IsFinite(rotation.x)
                || !Dg5fGraspSpec.IsFinite(rotation.y)
                || !Dg5fGraspSpec.IsFinite(rotation.z)
                || !Dg5fGraspSpec.IsFinite(rotation.w))
            {
                return false;
            }

            foreach (var joint in _allJoints)
            {
                if (!Dg5fGraspSpec.IsFinite(FirstOrZero(joint.jointPosition))
                    || !Dg5fGraspSpec.IsFinite(FirstOrZero(joint.jointVelocity))
                    || !Dg5fGraspSpec.IsFinite(joint.xDrive.target))
                {
                    return false;
                }
            }
            return true;
        }

        void FinishEpisode(bool success, string failureReason)
        {
            if (!_episodeActive) return;
            _episodeActive = false;
            ScoreRewardPotentials();
            if (success) AddReward(Dg5fGraspSpec.GraspSuccessReward);

            float finalDistance = GraspDistance();
            if (!Dg5fGraspSpec.IsFinite(finalDistance))
                finalDistance = Dg5fGraspSpec.MaximumBallDistance;
            if (!Dg5fGraspSpec.IsFinite(_bestGraspDistance))
                _bestGraspDistance = finalDistance;

            _stats.Add("Grasp/Success", success ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Grasp/MaxContactHoldSeconds", _maxContactHoldSeconds, StatAggregationMethod.Average);
            _stats.Add("Grasp/ThumbContactReached", _thumbContactReached ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Grasp/OpposingContactReached", _opposingContactReached ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Grasp/DualContactReached", _dualContactReached ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Reach/Success", _reachSucceeded ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Reach/FirstSuccessSeconds", _firstReachSeconds, StatAggregationMethod.Average);
            _stats.Add("Reach/FinalDistanceMeters", finalDistance, StatAggregationMethod.Average);
            _stats.Add("Reach/BestDistanceMeters", _bestGraspDistance, StatAggregationMethod.Average);
            RecordFailureStatistics(failureReason);

            if (_evaluationEpisodeId >= 0)
            {
                Dg5fEvaluationSession.RecordEpisode(
                    this,
                    _evaluationEpisodeId,
                    success,
                    failureReason,
                    _episodeSeconds,
                    _reachSucceeded,
                    _firstReachSeconds,
                    finalDistance,
                    _bestGraspDistance,
                    _maxContactHoldSeconds);
                _evaluationEpisodeId = -1;
            }
            EndEpisode();
        }

        void RecordFailureStatistics(string failureReason)
        {
            string[] reasons = { "Timeout", "BallOutOfBounds", "NonFinitePhysics" };
            foreach (string reason in reasons)
                _stats.Add(
                    $"Failure/{reason}",
                    failureReason == reason ? 1f : 0f,
                    StatAggregationMethod.Average);
        }

        float GraspDistance()
        {
            return Vector3.Distance(graspPoint.position, ball.position);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var actions = actionsOut.ContinuousActions;
            for (int i = 0; i < actions.Length; i++) actions[i] = 0f;

#if ENABLE_LEGACY_INPUT_MANAGER
            actions[0] = Axis(KeyCode.Q, KeyCode.A);
            actions[1] = Axis(KeyCode.W, KeyCode.S);
            actions[2] = Axis(KeyCode.E, KeyCode.D);
            actions[3] = Axis(KeyCode.R, KeyCode.F);
            actions[4] = Axis(KeyCode.T, KeyCode.G);
            actions[5] = Axis(KeyCode.Y, KeyCode.H);
            actions[6] = Axis(KeyCode.Space, KeyCode.LeftShift);
#endif
        }

#if ENABLE_LEGACY_INPUT_MANAGER
        static float Axis(KeyCode positive, KeyCode negative)
        {
            return (Input.GetKey(positive) ? 1f : 0f) - (Input.GetKey(negative) ? 1f : 0f);
        }
#endif
    }
}
