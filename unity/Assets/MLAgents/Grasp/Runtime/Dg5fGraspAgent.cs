using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// PPO Agent for stable whole-hand grasp training with independently controlled
    /// UR5e 6-axis arm and DG5F 20-joint hand.
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
        [Range(0f, Dg5fGraspSpec.MaximumHandDeltaDegPerDecision)]
        public float handDeltaDegPerDecision = Dg5fGraspSpec.MaximumHandDeltaDegPerDecision;
        public float stageOneArmDeltaScale = 0.35f;
        public float stageOneHandDeltaDegPerDecision = 1f;
        public float stageOneArmRangeDeg = 25f;
        public float stageOneSpawnRadius = 0.04f;

        readonly Dictionary<ArticulationBody, float> _initialTargetDeg =
            new Dictionary<ArticulationBody, float>();

        ArticulationBody[] _allJoints;
        ArticulationBody[] _armJoints;
        ArticulationBody[] _handJoints;
        float[] _armTargetDeg;
        float[] _handTargetDeg;
        int _curriculumStage;
        float _initialBallHeight;
        float _supportTopHeight;
        float _previousApproachPotential;
        float _previousContactPotential;
        float _previousLiftPotential;
        float _previousStableHoldPotential;
        float _bestGraspDistance;
        float _episodeSeconds;
        float _contactHoldSeconds;
        float _maxContactHoldSeconds;
        float _stableHoldSeconds;
        float _maxStableHoldSeconds;
        float _maxLiftHeight;
        float _requiredLiftHeight;
        float _minimumHoldHeight;
        float _requiredStableHoldSeconds;
        float _firstReachSeconds;
        int _ballReleaseFixedSteps;
        int _evaluationEpisodeId = -1;
        int _maxContactFingerCount;
        int _maxNonThumbContactCount;
        int _minimumNonThumbContactsAfterStable;
        bool _episodeActive;
        bool _reachSucceeded;
        bool _thumbContactReached;
        bool _stableContactReached;
        bool _stableGraspAcquired;
        bool _liftTargetReached;
        bool _thumbMaintainedAfterStable;
        string _gripLostCause = "None";
        readonly bool[] _fingerContacts = new bool[Dg5fGraspSpec.FingerCount];
        System.Random _random;
        StatsRecorder _stats;

        public int CurrentCurriculumStage => _curriculumStage;
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
        public float CurrentStableHoldSeconds => _stableHoldSeconds;
        public float MaxStableHoldSeconds => _maxStableHoldSeconds;
        public float CurrentLiftHeight => Dg5fGraspSpec.LiftHeight(ball.position.y, _initialBallHeight);
        public float MaxLiftHeight => _maxLiftHeight;
        public bool StableGraspAcquired => _stableGraspAcquired;
        public bool LiftTargetReached => _liftTargetReached;
        public bool LastEpisodeSucceeded { get; private set; }
        public string LastEpisodeFailureReason { get; private set; } = "None";
        public bool HasThumbContact => Dg5fGraspSpec.HasThumbContact(_fingerContacts);
        public int NonThumbContactCount => Dg5fGraspSpec.NonThumbContactCount(_fingerContacts);
        public bool HasStableContact => Dg5fGraspSpec.HasStableContact(_fingerContacts);

        public float CurrentArmTargetDeg(int index)
        {
            if (_armTargetDeg == null) throw new InvalidOperationException("Agent has not initialized.");
            return _armTargetDeg[index];
        }

        public float CurrentHandTargetDeg(int index)
        {
            if (_handTargetDeg == null) throw new InvalidOperationException("Agent has not initialized.");
            return _handTargetDeg[index];
        }

        public float CurrentHandDriveTargetDeg(int index)
        {
            if (_handJoints == null) throw new InvalidOperationException("Agent has not initialized.");
            return _handJoints[index].xDrive.target;
        }

        public float CurrentHandPositionDeg(int index)
        {
            if (_handJoints == null) throw new InvalidOperationException("Agent has not initialized.");
            return FirstOrZero(_handJoints[index].jointPosition) * Mathf.Rad2Deg;
        }

        public float CurrentHandVelocityRadPerSecond(int index)
        {
            if (_handJoints == null) throw new InvalidOperationException("Agent has not initialized.");
            return FirstOrZero(_handJoints[index].jointVelocity);
        }

        public override void Initialize()
        {
            ResolveReferences();
            ResolveJoints();
            ValidateConfiguration();

            _armTargetDeg = new float[Dg5fGraspSpec.ArmJointCount];
            _handTargetDeg = new float[Dg5fGraspSpec.HandJointCount];
            foreach (var body in _allJoints)
                _initialTargetDeg[body] = body.xDrive.target;

            // The exact 20-second limit is measured in simulation time.
            MaxStep = 0;
            int randomSeed = useDeterministicSpawns
                ? spawnSeed
                : unchecked(spawnSeed * 397 ^ UnityEngine.Random.Range(0, int.MaxValue));
            _random = new System.Random(randomSeed);
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

            _curriculumStage = ResolveCurriculumStage();
            ResetRobot();
            ResetBall();
            foreach (var sensor in contactSensors) sensor.ResetContacts();

            _episodeSeconds = 0f;
            _bestGraspDistance = GraspDistance();
            _previousApproachPotential = Dg5fGraspSpec.ApproachPotential(_bestGraspDistance);
            _previousContactPotential = 0f;
            _previousLiftPotential = 0f;
            _previousStableHoldPotential = 0f;
            _contactHoldSeconds = 0f;
            _maxContactHoldSeconds = 0f;
            _stableHoldSeconds = 0f;
            _maxStableHoldSeconds = 0f;
            _maxLiftHeight = 0f;
            _requiredLiftHeight = Dg5fGraspSpec.RequiredLiftHeight(_curriculumStage);
            _minimumHoldHeight = Dg5fGraspSpec.MinimumHoldHeight(_curriculumStage);
            _requiredStableHoldSeconds = Dg5fGraspSpec.RequiredStableHoldSeconds(_curriculumStage);
            _firstReachSeconds = -1f;
            _reachSucceeded = false;
            _thumbContactReached = false;
            _stableContactReached = false;
            _stableGraspAcquired = false;
            _liftTargetReached = false;
            _maxContactFingerCount = 0;
            _maxNonThumbContactCount = 0;
            _minimumNonThumbContactsAfterStable = int.MaxValue;
            _thumbMaintainedAfterStable = true;
            _gripLostCause = "None";
            Array.Clear(_fingerContacts, 0, _fingerContacts.Length);
            _episodeActive = true;
        }

        int ResolveCurriculumStage()
        {
            if (Dg5fEvaluationSession.IsEnabled) return Dg5fGraspSpec.FinalCurriculumStage;
            float configured = Academy.Instance.EnvironmentParameters.GetWithDefault(
                Dg5fGraspSpec.CurriculumParameterName,
                Dg5fGraspSpec.FinalCurriculumStage);
            if (!Dg5fGraspSpec.IsFinite(configured))
                return Dg5fGraspSpec.FinalCurriculumStage;
            return Mathf.Clamp(
                Mathf.RoundToInt(configured),
                Dg5fGraspSpec.FirstCurriculumStage,
                Dg5fGraspSpec.FinalCurriculumStage);
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
                SynchronizeJointState(body, _initialTargetDeg[body]);
            }

            for (int i = 0; i < _armJoints.Length; i++)
            {
                float initial = _initialTargetDeg[_armJoints[i]];
                _armTargetDeg[i] = Dg5fGraspSpec.ClampJointTarget(
                    initial,
                    Dg5fGraspSpec.ArmSafeMinDeg[i],
                    Dg5fGraspSpec.ArmSafeMaxDeg[i]);
                SynchronizeJointState(_armJoints[i], _armTargetDeg[i]);
                _armTargetDeg[i] = _armJoints[i].xDrive.target;
            }
            for (int i = 0; i < _handJoints.Length; i++)
            {
                var drive = _handJoints[i].xDrive;
                float resetTarget = _curriculumStage < Dg5fGraspSpec.FinalCurriculumStage
                    ? Dg5fGraspSpec.PreGrasp35Deg[i]
                    : 0f;
                _handTargetDeg[i] = Dg5fGraspSpec.ClampJointTarget(
                    resetTarget,
                    drive.lowerLimit,
                    drive.upperLimit);
                SynchronizeJointState(_handJoints[i], _handTargetDeg[i]);
            }
        }

        static void SynchronizeJointState(ArticulationBody body, float targetDeg)
        {
            var drive = body.xDrive;
            float synchronizedTarget = Dg5fGraspSpec.ClampJointTarget(
                targetDeg,
                drive.lowerLimit,
                drive.upperLimit);
            drive.target = synchronizedTarget;
            body.xDrive = drive;
            body.jointPosition = new ArticulationReducedSpace(synchronizedTarget * Mathf.Deg2Rad);
            body.jointVelocity = new ArticulationReducedSpace(0f);
        }

        void ResetBall()
        {
            Collider ballCollider = ball.GetComponent<Collider>();
            if (ballCollider == null)
                throw new InvalidOperationException("[Dg5fGraspAgent] Ball requires a collider.");
            float ballRadius = ballCollider.bounds.extents.y;
            Vector3 ballLocalPosition;
            if (_curriculumStage == Dg5fGraspSpec.FirstCurriculumStage)
            {
                Vector3 graspLocal = robotBase.InverseTransformPoint(graspPoint.position);
                float radius = Mathf.Max(0f, stageOneSpawnRadius) * Mathf.Pow(Next01(), 1f / 3f);
                float azimuth = Next01() * 2f * Mathf.PI;
                float cosine = Next01() * 2f - 1f;
                float sine = Mathf.Sqrt(Mathf.Max(0f, 1f - cosine * cosine));
                ballLocalPosition = graspLocal + radius * new Vector3(
                    sine * Mathf.Cos(azimuth),
                    cosine,
                    sine * Mathf.Sin(azimuth));
            }
            else
            {
                ballLocalPosition = Dg5fGraspSpec.SpawnBallLocalPosition(
                    Next01(),
                    Next01(),
                    ballRadius);
                if (!Dg5fGraspSpec.IsValidSpawn(ballLocalPosition, ballRadius))
                    throw new InvalidOperationException("[Dg5fGraspAgent] Generated an invalid v1 spawn pose.");
            }

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

            // 0..5: normalized arm position.
            for (int i = 0; i < _armJoints.Length; i++)
            {
                float positionDeg = FirstOrZero(_armJoints[i].jointPosition) * Mathf.Rad2Deg;
                sensor.AddObservation(Dg5fGraspSpec.NormalizeJoint(
                    positionDeg,
                    Dg5fGraspSpec.ArmSafeMinDeg[i],
                    Dg5fGraspSpec.ArmSafeMaxDeg[i]));
            }
            // 6..11: arm velocity.
            for (int i = 0; i < _armJoints.Length; i++)
                sensor.AddObservation(Mathf.Clamp(FirstOrZero(_armJoints[i].jointVelocity) / Mathf.PI, -1f, 1f));

            // 12..31: normalized hand position in thumb-to-pinky, joint 1..4 order.
            for (int i = 0; i < _handJoints.Length; i++)
            {
                var drive = _handJoints[i].xDrive;
                float positionDeg = FirstOrZero(_handJoints[i].jointPosition) * Mathf.Rad2Deg;
                sensor.AddObservation(Dg5fGraspSpec.NormalizeJoint(
                    positionDeg,
                    drive.lowerLimit,
                    drive.upperLimit));
            }

            // 32..51: hand velocity.
            for (int i = 0; i < _handJoints.Length; i++)
                sensor.AddObservation(Mathf.Clamp(
                    FirstOrZero(_handJoints[i].jointVelocity) / Mathf.PI,
                    -1f,
                    1f));

            // 52..71: normalized commanded hand xDrive targets.
            for (int i = 0; i < _handJoints.Length; i++)
            {
                var drive = _handJoints[i].xDrive;
                sensor.AddObservation(Dg5fGraspSpec.NormalizeJoint(
                    _handTargetDeg[i],
                    drive.lowerLimit,
                    drive.upperLimit));
            }

            // 72..80: ball state in robot-base coordinates.
            AddClampedVector(sensor, robotBase.InverseTransformDirection(ball.position - graspPoint.position), 1f);
            AddClampedVector(sensor, robotBase.InverseTransformDirection(ball.linearVelocity), 2f);
            AddClampedVector(sensor, robotBase.InverseTransformDirection(ball.angularVelocity), 10f);

            // 81: vertical displacement from the episode spawn pose.
            sensor.AddObservation(Mathf.Clamp((ball.position.y - _initialBallHeight) / 0.2f, -1f, 1f));

            // 82..96: each fingertip relative to the ball in palm coordinates.
            for (int i = 0; i < fingerTips.Length; i++)
                AddClampedVector(sensor, palm.InverseTransformDirection(fingerTips[i].position - ball.position), 0.2f);

            // 97..101: thumb/index/middle/ring/pinky contacts.
            for (int i = 0; i < contactSensors.Length; i++)
                sensor.AddObservation(contactSensors[i].IsTouching ? 1f : 0f);

            // 102..107: normalized commanded arm xDrive targets.
            for (int i = 0; i < _armTargetDeg.Length; i++)
                sensor.AddObservation(Dg5fGraspSpec.NormalizeJoint(
                    _armTargetDeg[i],
                    Dg5fGraspSpec.ArmSafeMinDeg[i],
                    Dg5fGraspSpec.ArmSafeMaxDeg[i]));

            // 108..111: inherited objective one-hot; v4 is stable whole-hand grasp.
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(1f);

            // 112..115: approach, stable contact, lift, and low-speed hold progress.
            sensor.AddObservation(Dg5fGraspSpec.ApproachPotential(GraspDistance()));
            sensor.AddObservation(Dg5fGraspSpec.StableContactProgress(
                _contactHoldSeconds,
                _stableGraspAcquired));
            sensor.AddObservation(_requiredLiftHeight <= 0f
                ? (_stableGraspAcquired ? 1f : 0f)
                : Dg5fGraspSpec.LiftPotential(
                    CurrentLiftHeight,
                    _requiredLiftHeight,
                    _stableGraspAcquired));
            sensor.AddObservation(_requiredStableHoldSeconds <= 0f
                ? (_stableGraspAcquired ? 1f : 0f)
                : Dg5fGraspSpec.StableHoldPotential(
                    _stableHoldSeconds,
                    _requiredStableHoldSeconds));
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
                float maximumDelta = armDeltaDegPerDecision;
                float lower = Dg5fGraspSpec.ArmSafeMinDeg[i];
                float upper = Dg5fGraspSpec.ArmSafeMaxDeg[i];
                if (_curriculumStage == Dg5fGraspSpec.FirstCurriculumStage)
                {
                    maximumDelta *= Mathf.Clamp01(stageOneArmDeltaScale);
                    float initial = _initialTargetDeg[_armJoints[i]];
                    lower = Mathf.Max(lower, initial - Mathf.Max(0f, stageOneArmRangeDeg));
                    upper = Mathf.Min(upper, initial + Mathf.Max(0f, stageOneArmRangeDeg));
                }
                _armTargetDeg[i] = Dg5fGraspSpec.AccumulateJointTarget(
                    _armTargetDeg[i],
                    continuous[i],
                    maximumDelta,
                    lower,
                    upper);
            }

            float maximumHandDelta = Mathf.Min(
                Mathf.Max(0f, handDeltaDegPerDecision),
                Dg5fGraspSpec.MaximumHandDeltaDegPerDecision);
            if (_curriculumStage == Dg5fGraspSpec.FirstCurriculumStage)
                maximumHandDelta = Mathf.Min(
                    maximumHandDelta,
                    Mathf.Max(0f, stageOneHandDeltaDegPerDecision));
            for (int i = 0; i < _handTargetDeg.Length; i++)
            {
                var drive = _handJoints[i].xDrive;
                _handTargetDeg[i] = Dg5fGraspSpec.AccumulateJointTarget(
                    _handTargetDeg[i],
                    continuous[Dg5fGraspSpec.HandActionIndex(i)],
                    maximumHandDelta,
                    drive.lowerLimit,
                    drive.upperLimit);
            }

            ApplyArmTargets();
            ApplyHandTargets();
        }

        void ApplyArmTargets()
        {
            for (int i = 0; i < _armJoints.Length; i++)
            {
                var drive = _armJoints[i].xDrive;
                drive.target = Dg5fGraspSpec.ClampJointTarget(
                    _armTargetDeg[i],
                    drive.lowerLimit,
                    drive.upperLimit);
                _armTargetDeg[i] = drive.target;
                _armJoints[i].xDrive = drive;
            }
        }

        void ApplyHandTargets()
        {
            for (int i = 0; i < _handJoints.Length; i++)
            {
                var drive = _handJoints[i].xDrive;
                drive.target = Dg5fGraspSpec.ClampJointTarget(
                    _handTargetDeg[i],
                    drive.lowerLimit,
                    drive.upperLimit);
                _handTargetDeg[i] = drive.target;
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

            string ballFailureReason = Dg5fGraspSpec.BallFailureReason(
                ballLocalPosition,
                _supportTopHeight);
            if (ballFailureReason != null)
            {
                FinishEpisode(false, ballFailureReason);
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
            if (_stableGraspAcquired && !Dg5fGraspSpec.HasStableContact(_fingerContacts))
            {
                _gripLostCause = ResolveGripLostCause();
                FinishEpisode(false, "GripLost");
                return;
            }

            if (!_stableGraspAcquired
                && Dg5fGraspSpec.HasAcquiredStableGrasp(_contactHoldSeconds))
            {
                _stableGraspAcquired = true;
                _thumbMaintainedAfterStable = true;
                _minimumNonThumbContactsAfterStable =
                    Dg5fGraspSpec.NonThumbContactCount(_fingerContacts);
                AddReward(Dg5fGraspSpec.StableGraspAcquiredReward);
                if (_curriculumStage == Dg5fGraspSpec.FirstCurriculumStage)
                {
                    FinishEpisode(true, "None");
                    return;
                }
            }

            float liftHeight = CurrentLiftHeight;
            _maxLiftHeight = Mathf.Max(_maxLiftHeight, liftHeight);
            if (_stableGraspAcquired
                && Dg5fGraspSpec.HasReachedLiftTarget(liftHeight, _requiredLiftHeight))
            {
                _liftTargetReached = true;
            }

            _stableHoldSeconds = Dg5fGraspSpec.NextStableHoldSeconds(
                _stableHoldSeconds,
                Dg5fGraspSpec.HasStableContact(_fingerContacts),
                _liftTargetReached,
                liftHeight,
                _minimumHoldHeight,
                ball.linearVelocity.magnitude,
                Time.fixedDeltaTime);
            _maxStableHoldSeconds = Mathf.Max(_maxStableHoldSeconds, _stableHoldSeconds);
            if (Dg5fGraspSpec.HasCompletedStableHold(
                _stableHoldSeconds,
                _requiredStableHoldSeconds))
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
            _previousLiftPotential = 0f;
            _previousStableHoldPotential = 0f;
        }

        void UpdateContactState(float deltaSeconds)
        {
            for (int index = 0; index < _fingerContacts.Length; index++)
                _fingerContacts[index] = contactSensors[index].IsTouching;

            bool thumbContact = Dg5fGraspSpec.HasThumbContact(_fingerContacts);
            int contactCount = Dg5fGraspSpec.ContactFingerCount(_fingerContacts);
            int nonThumbContactCount = Dg5fGraspSpec.NonThumbContactCount(_fingerContacts);
            bool stableContact = Dg5fGraspSpec.HasStableContact(_fingerContacts);
            _thumbContactReached |= thumbContact;
            _stableContactReached |= stableContact;
            _maxContactFingerCount = Mathf.Max(_maxContactFingerCount, contactCount);
            _maxNonThumbContactCount = Mathf.Max(_maxNonThumbContactCount, nonThumbContactCount);
            _contactHoldSeconds = Dg5fGraspSpec.NextContactHoldSeconds(
                _contactHoldSeconds,
                stableContact,
                deltaSeconds);
            _maxContactHoldSeconds = Mathf.Max(_maxContactHoldSeconds, _contactHoldSeconds);
            if (_stableGraspAcquired)
            {
                _thumbMaintainedAfterStable &= thumbContact;
                _minimumNonThumbContactsAfterStable = Mathf.Min(
                    _minimumNonThumbContactsAfterStable,
                    nonThumbContactCount);
            }
        }

        string ResolveGripLostCause()
        {
            int contactCount = Dg5fGraspSpec.ContactFingerCount(_fingerContacts);
            if (contactCount == 0) return "AllContactsLost";
            if (!Dg5fGraspSpec.HasThumbContact(_fingerContacts)) return "ThumbContactLost";
            return "NonThumbContactCountBelowTwo";
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

            float currentContactPotential = Dg5fGraspSpec.ContactPotential(_fingerContacts);
            AddReward(Dg5fGraspSpec.PotentialDelta(
                _previousContactPotential,
                currentContactPotential));
            _previousContactPotential = currentContactPotential;

            float currentLiftPotential = Dg5fGraspSpec.LiftPotential(
                CurrentLiftHeight,
                _requiredLiftHeight,
                _stableGraspAcquired);
            AddReward(Dg5fGraspSpec.PotentialDelta(
                _previousLiftPotential,
                currentLiftPotential));
            _previousLiftPotential = currentLiftPotential;

            float currentHoldPotential = Dg5fGraspSpec.StableHoldPotential(
                _stableHoldSeconds,
                _requiredStableHoldSeconds);
            AddReward(Dg5fGraspSpec.PotentialDelta(
                _previousStableHoldPotential,
                currentHoldPotential));
            _previousStableHoldPotential = currentHoldPotential;
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
            LastEpisodeSucceeded = success;
            LastEpisodeFailureReason = success ? "None" : failureReason;
            ScoreRewardPotentials();
            if (success)
            {
                AddReward(Dg5fGraspSpec.StableGraspSuccessReward);
            }
            else
            {
                SettleFailurePotentials();
                AddReward(Dg5fGraspSpec.FailurePenalty(failureReason));
            }

            float finalDistance = GraspDistance();
            if (!Dg5fGraspSpec.IsFinite(finalDistance))
                finalDistance = Dg5fGraspSpec.MaximumBallDistance;
            if (!Dg5fGraspSpec.IsFinite(_bestGraspDistance))
                _bestGraspDistance = finalDistance;

            _stats.Add("Grasp/Success", success ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Grasp/MaxContactHoldSeconds", _maxContactHoldSeconds, StatAggregationMethod.Average);
            _stats.Add("Grasp/MaxContactFingerCount", _maxContactFingerCount, StatAggregationMethod.Average);
            _stats.Add("Grasp/MaxNonThumbContactCount", _maxNonThumbContactCount, StatAggregationMethod.Average);
            _stats.Add("Grasp/ThumbContactReached", _thumbContactReached ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Grasp/StableContactReached", _stableContactReached ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Grasp/StableGraspAcquired", _stableGraspAcquired ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Lift/MaxHeightMeters", _maxLiftHeight, StatAggregationMethod.Average);
            _stats.Add("Hold/MaxValidSeconds", _maxStableHoldSeconds, StatAggregationMethod.Average);
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
                    _maxContactHoldSeconds,
                    _maxContactFingerCount,
                    _maxNonThumbContactCount,
                    _stableGraspAcquired,
                    _maxLiftHeight,
                    _maxStableHoldSeconds,
                    _gripLostCause,
                    _thumbMaintainedAfterStable,
                    _minimumNonThumbContactsAfterStable == int.MaxValue
                        ? 0
                        : _minimumNonThumbContactsAfterStable);
                _evaluationEpisodeId = -1;
            }
            EndEpisode();
        }

        void SettleFailurePotentials()
        {
            AddReward(Dg5fGraspSpec.FailurePotentialSettlement(
                _previousApproachPotential,
                _previousContactPotential,
                _previousLiftPotential,
                _previousStableHoldPotential));
            _previousApproachPotential = 0f;
            _previousContactPotential = 0f;
            _previousLiftPotential = 0f;
            _previousStableHoldPotential = 0f;
        }

        void RecordFailureStatistics(string failureReason)
        {
            string[] reasons =
            {
                "Timeout", "GripLost", "Drop", "WorkspaceExit", "NonFinitePhysics"
            };
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

        void OnDrawGizmos()
        {
            if (graspPoint == null) return;
            Gizmos.color = new Color(0.1f, 0.85f, 1f, 0.8f);
            Gizmos.DrawWireSphere(graspPoint.position, Dg5fGraspSpec.GraspBallRadius);
            if (fingerTips == null) return;
            for (int index = 0; index < fingerTips.Length; index++)
            {
                if (fingerTips[index] != null)
                    Gizmos.DrawLine(graspPoint.position, fingerTips[index].position);
            }
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
