using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// PPO Agent for staged reach, grasp, lift, and hold training with UR5e + DG5F.
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
        Collider[] _robotColliders;
        Collider[] _handColliders;
        float[] _armTargetDeg;
        float[] _currentArmPositionDeg;
        float _closure;
        float _initialBallHeight;
        float _supportTopHeight;
        float[] _previousArmPositionDeg;
        float _previousReachPotential;
        float _previousContactPotential;
        float _previousLiftPotential;
        float _previousHoldPotential;
        float _normalizedArmTravel;
        float _movementSinceDecision;
        Dg5fCurriculumStage _curriculumStage;
        Dg5fEpisodeState _episodeState;
        bool _episodeActive;
        System.Random _random;
        StatsRecorder _stats;

        public float CurrentClosure => _closure;
        public float CurrentSupportTopHeight => _supportTopHeight;
        public Vector3 CurrentBallLocalPosition => robotBase.InverseTransformPoint(ball.position);
        public Dg5fGraspPhase CurrentPhase => _episodeState.Phase;
        public int CurrentCurriculumStage => _curriculumStage.Index;
        public float CurrentGraspSeconds => _episodeState.GraspSeconds;
        public float CurrentHoldSeconds => _episodeState.HoldSeconds;
        public float CurrentEpisodeSeconds => _episodeState.EpisodeSeconds;
        public float MaximumLiftMeters => _episodeState.MaximumLiftMeters;
        public float MaximumHoldSeconds => _episodeState.MaximumHoldSeconds;
        public float NormalizedArmTravel => _normalizedArmTravel;

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
            _previousArmPositionDeg = new float[Dg5fGraspSpec.ArmJointCount];
            _currentArmPositionDeg = new float[Dg5fGraspSpec.ArmJointCount];
            foreach (var body in _allJoints)
                _initialTargetDeg[body] = body.xDrive.target;

            // The exact 20-second limit is measured in physics time by the V4 state machine.
            MaxStep = 0;
            _random = new System.Random(spawnSeed);
            _stats = Academy.Instance.StatsRecorder;
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

            if (robotBase != null) _robotColliders = robotBase.GetComponentsInChildren<Collider>(true);
            if (palm != null) _handColliders = palm.GetComponentsInChildren<Collider>(true);
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
            if (_handColliders == null || _handColliders.Length == 0)
                throw new InvalidOperationException("[Dg5fGraspAgent] No palm/finger colliders were found.");
            if (_robotColliders == null || _robotColliders.Length == 0)
                throw new InvalidOperationException("[Dg5fGraspAgent] No robot colliders were found.");
            for (int i = 0; i < _armJoints.Length; i++)
                if (_armJoints[i] == null)
                    throw new InvalidOperationException($"[Dg5fGraspAgent] Missing arm joint: {Dg5fGraspSpec.ArmLinks[i]}");
            for (int i = 0; i < _handJoints.Length; i++)
                if (_handJoints[i] == null)
                    throw new InvalidOperationException($"[Dg5fGraspAgent] Missing hand joint channel {i}.");
            for (int i = 0; i < Dg5fGraspSpec.FingerCount; i++)
                if (fingerTips[i] == null || contactSensors[i] == null)
                    throw new InvalidOperationException($"[Dg5fGraspAgent] Missing fingertip/contact sensor {i}.");
        }

        public override void OnEpisodeBegin()
        {
            _episodeActive = false;
            _closure = 0f;
            int stageIndex = Mathf.RoundToInt(
                Academy.Instance.EnvironmentParameters.GetWithDefault(
                    "lesson",
                    Dg5fGraspSpec.CurriculumStages.Length - 1));
            _curriculumStage = Dg5fGraspSpec.GetCurriculumStage(stageIndex);
            ResetRobot();
            ResetBall();
            foreach (var sensor in contactSensors) sensor.ResetContacts();

            _episodeState.Reset();
            _normalizedArmTravel = 0f;
            _movementSinceDecision = 0f;
            _previousReachPotential = Dg5fGraspSpec.ReachPotential(GraspDistance());
            _previousContactPotential = 0f;
            _previousLiftPotential = 0f;
            _previousHoldPotential = 0f;
            _episodeActive = true;
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
                _previousArmPositionDeg[i] = FirstOrZero(_armJoints[i].jointPosition) * Mathf.Rad2Deg;
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
            Physics.SyncTransforms();

            Vector3 ballLocalPosition = default;
            Vector3 localGraspPoint = robotBase.InverseTransformPoint(graspPoint.position);
            float centerAzimuthDegrees = Mathf.Atan2(localGraspPoint.z, localGraspPoint.x) * Mathf.Rad2Deg;
            bool found = false;
            for (int attempt = 0; attempt < 1024; attempt++)
            {
                ballLocalPosition = Dg5fGraspSpec.SpawnBallLocalPosition(
                    Next01(),
                    Next01(),
                    ballRadius,
                    _curriculumStage,
                    centerAzimuthDegrees);
                if (Dg5fGraspSpec.IsValidSpawn(ballLocalPosition, ballRadius)
                    && HasRobotSpawnClearance(
                        robotBase.TransformPoint(ballLocalPosition),
                        ballRadius))
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                throw new InvalidOperationException(
                    "[Dg5fGraspAgent] Could not sample a valid workspace pose with robot clearance.");
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

            ball.isKinematic = false;
            ball.useGravity = true;
            ball.linearVelocity = Vector3.zero;
            ball.angularVelocity = Vector3.zero;
            _initialBallHeight = ball.position.y;
        }

        bool HasRobotSpawnClearance(Vector3 ballCenter, float ballRadius)
        {
            foreach (var robotCollider in _robotColliders)
            {
                if (robotCollider == null
                    || !robotCollider.enabled
                    || !robotCollider.gameObject.activeInHierarchy
                    || robotCollider.isTrigger)
                {
                    continue;
                }

                Vector3 closestPoint = robotCollider.ClosestPoint(ballCenter);
                float centerToSurfaceDistance = Vector3.Distance(ballCenter, closestPoint);
                if (!Dg5fGraspSpec.HasMinimumRobotSpawnClearance(
                    centerToSurfaceDistance,
                    ballRadius))
                {
                    return false;
                }
            }
            return true;
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

            // 49..52: Reach/Grasp/Lift/Hold one-hot.
            for (int phase = 0; phase < 4; phase++)
                sensor.AddObservation((int)_episodeState.Phase == phase ? 1f : 0f);

            // 53..54: continuous grasp and hold progress.
            sensor.AddObservation(Mathf.Clamp01(
                _episodeState.GraspSeconds / Dg5fGraspSpec.StableGraspSeconds));
            sensor.AddObservation(Mathf.Clamp01(
                _episodeState.HoldSeconds / Mathf.Max(1e-5f, _curriculumStage.HoldTargetSeconds)));

            // 55..56: current curriculum lift and hold targets.
            sensor.AddObservation(Mathf.Clamp01(_curriculumStage.LiftTargetMeters / 0.10f));
            sensor.AddObservation(Mathf.Clamp01(_curriculumStage.HoldTargetSeconds / 5f));
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
            if (!_episodeActive) return;

            float decisionMovement = _movementSinceDecision;
            _movementSinceDecision = 0f;

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

            float reachPotential = Dg5fGraspSpec.ReachPotential(GraspDistance());
            float contactPotential = CurrentContactPotential();
            float liftPotential = _episodeState.Phase >= Dg5fGraspPhase.Lift
                ? Dg5fGraspSpec.LiftPotential(CurrentLiftMeters(), _curriculumStage.LiftTargetMeters)
                : 0f;
            float holdPotential = Dg5fGraspSpec.HoldPotential(
                _episodeState.HoldSeconds,
                _curriculumStage.HoldTargetSeconds);
            float decisionReward = Dg5fGraspSpec.DecisionTimePenalty
                + Dg5fGraspSpec.PotentialDelta(_previousReachPotential, reachPotential)
                + Dg5fGraspSpec.PotentialDelta(_previousContactPotential, contactPotential)
                + Dg5fGraspSpec.PotentialDelta(_previousLiftPotential, liftPotential)
                + Dg5fGraspSpec.PotentialDelta(_previousHoldPotential, holdPotential)
                + Dg5fGraspSpec.MovementPenalty(
                    decisionMovement,
                    _curriculumStage.MovementPenaltyLambda);

            _previousReachPotential = reachPotential;
            _previousContactPotential = contactPotential;
            _previousLiftPotential = liftPotential;
            _previousHoldPotential = holdPotential;
            AddReward(decisionReward);
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

        void SampleArmMovement()
        {
            for (int i = 0; i < _currentArmPositionDeg.Length; i++)
                _currentArmPositionDeg[i] = FirstOrZero(_armJoints[i].jointPosition) * Mathf.Rad2Deg;
            float movement = Dg5fGraspSpec.NormalizedArmMovement(
                _previousArmPositionDeg,
                _currentArmPositionDeg);
            _normalizedArmTravel += movement;
            _movementSinceDecision += movement;
            Array.Copy(_currentArmPositionDeg, _previousArmPositionDeg, _currentArmPositionDeg.Length);
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

            Vector3 ballLocalPosition = robotBase.InverseTransformPoint(ball.position);
            bool finitePhysics = HasFinitePhysicsState();
            if (!finitePhysics)
            {
                FinishEpisode(false, Dg5fFailureReason.NonFinitePhysics);
                return;
            }
            SampleArmMovement();

            if (ballLocalPosition.y < _supportTopHeight)
            {
                FinishEpisode(false, Dg5fFailureReason.Dropped);
                return;
            }

            bool workspaceValid = ballLocalPosition.magnitude <= Dg5fGraspSpec.MaximumBallDistance;
            bool penetration = MaximumHandPedestalPenetration()
                >= Dg5fGraspSpec.PenetrationDepthMeters;
            Dg5fEpisodeStepResult result = _episodeState.Advance(
                CurrentEpisodeInput(workspaceValid, penetration, finitePhysics),
                _curriculumStage,
                Time.fixedDeltaTime);

            if (result.StableGraspReached)
                AddReward(Dg5fGraspSpec.StableGraspReward);
            if (result.LiftTargetReached)
                AddReward(Dg5fGraspSpec.LiftTargetReward);
            if (result.Succeeded)
                FinishEpisode(true, Dg5fFailureReason.None);
            else if (result.Failed)
                FinishEpisode(false, result.FailureReason);
        }

        float CurrentContactPotential()
        {
            bool thumbContact = contactSensors[0].IsTouching;
            bool opposingContact = false;
            for (int i = 1; i < contactSensors.Length; i++)
                opposingContact |= contactSensors[i].IsTouching;
            return Dg5fGraspSpec.ContactPotential(thumbContact, opposingContact);
        }

        Dg5fEpisodeInput CurrentEpisodeInput(
            bool workspaceValid,
            bool penetration = false,
            bool physicsFinite = true)
        {
            bool thumbContact = contactSensors[0].IsTouching;
            bool opposingContact = false;
            for (int i = 1; i < contactSensors.Length; i++)
                opposingContact |= contactSensors[i].IsTouching;
            return new Dg5fEpisodeInput(
                Dg5fGraspSpec.HasGraspContact(thumbContact, opposingContact),
                CurrentLiftMeters(),
                ball.linearVelocity.magnitude,
                workspaceValid,
                penetration,
                physicsFinite);
        }

        float CurrentLiftMeters()
        {
            return ball.position.y - _initialBallHeight;
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

        float MaximumHandPedestalPenetration()
        {
            float maximumDepth = 0f;
            foreach (var handCollider in _handColliders)
            {
                if (handCollider == null || !handCollider.enabled || handCollider.isTrigger) continue;
                if (Physics.ComputePenetration(
                    handCollider,
                    handCollider.transform.position,
                    handCollider.transform.rotation,
                    pedestalCollider,
                    pedestalCollider.transform.position,
                    pedestalCollider.transform.rotation,
                    out _,
                    out float depth))
                {
                    maximumDepth = Mathf.Max(maximumDepth, depth);
                }
            }
            return maximumDepth;
        }

        void FinishEpisode(bool success, Dg5fFailureReason failureReason)
        {
            if (!_episodeActive) return;
            _episodeActive = false;
            if (!success && failureReason == Dg5fFailureReason.None)
                throw new ArgumentException("Every failed episode requires a failure reason.", nameof(failureReason));

            float finalHoldPotential = Dg5fGraspSpec.HoldPotential(
                _episodeState.HoldSeconds,
                _curriculumStage.HoldTargetSeconds);
            float finalReward = Dg5fGraspSpec.TerminalReward(success, failureReason)
                + Dg5fGraspSpec.PotentialDelta(_previousHoldPotential, finalHoldPotential)
                + Dg5fGraspSpec.MovementPenalty(
                    _movementSinceDecision,
                    _curriculumStage.MovementPenaltyLambda);
            _previousHoldPotential = finalHoldPotential;
            _movementSinceDecision = 0f;
            AddReward(finalReward);
            _stats.Add("Grasp/Success", success ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Curriculum/Stage", _curriculumStage.Index, StatAggregationMethod.Average);
            _stats.Add("Grasp/CompletionSeconds", _episodeState.EpisodeSeconds, StatAggregationMethod.Average);
            _stats.Add("Grasp/MaxLiftHeightMeters", _episodeState.MaximumLiftMeters, StatAggregationMethod.Average);
            _stats.Add("Grasp/HoldSeconds", _episodeState.MaximumHoldSeconds, StatAggregationMethod.Average);
            _stats.Add("Motion/NormalizedArmTravel", _normalizedArmTravel, StatAggregationMethod.Average);
            RecordFailureStats(failureReason);
            EndEpisode();
        }

        void RecordFailureStats(Dg5fFailureReason reason)
        {
            Dg5fFailureReason[] reasons =
            {
                Dg5fFailureReason.GripLost,
                Dg5fFailureReason.Dropped,
                Dg5fFailureReason.WorkspaceExit,
                Dg5fFailureReason.Penetration,
                Dg5fFailureReason.NonFinitePhysics,
                Dg5fFailureReason.Timeout
            };
            foreach (Dg5fFailureReason candidate in reasons)
                _stats.Add($"Failure/{candidate}", reason == candidate ? 1f : 0f, StatAggregationMethod.Average);
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
