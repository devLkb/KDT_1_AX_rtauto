using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace KDT.ReachTraining
{
    /// <summary>
    /// Moves an open DG5F hand through a clearance waypoint, descends toward the
    /// grasp target, and latches the six UR5e targets once the grasp-ready pose is
    /// stable. Finger drives are held at their prefab open pose and are never policy
    /// actions.
    /// </summary>
    public sealed class Dg5fGraspPointReachAgent : Agent
    {
        [Header("Scene references")]
        public Transform target;
        public Rigidbody targetBody;
        public Transform robotBase;
        public Transform palm;
        public Transform graspPoint;
        public ArticulationBody graspPointBody;
        public Collider panelCollider;
        public Collider[] robotColliders = Array.Empty<Collider>();
        public ReachSurfaceContactSensor[] safetySensors =
            Array.Empty<ReachSurfaceContactSensor>();

        [Header("Episode")]
        public bool useDeterministicSpawns;
        public int spawnSeed = 12345;
        public float targetCenterLocalY = Dg5fReachSpec.TargetRadius;

        [Header("Control")]
        [Range(0f, Dg5fReachSpec.MaximumArmDeltaDegPerDecision)]
        public float armDeltaDegPerDecision =
            Dg5fReachSpec.TrainingArmDeltaDegPerDecision;
        [Tooltip("Training ends the episode on lock. Deployment keeps the latch until ReleaseArmLock().")]
        public bool endEpisodeOnLock = true;

        readonly Dictionary<ArticulationBody, float> _initialArmTargetDeg =
            new Dictionary<ArticulationBody, float>();
        readonly Dictionary<ArticulationBody, float> _openHandTargetDeg =
            new Dictionary<ArticulationBody, float>();

        ArticulationBody[] _armJoints;
        ArticulationBody[] _handJoints;
        float[] _armTargetDeg;
        float[] _lockedArmTargetDeg;
        System.Random _random;
        StatsRecorder _stats;
        float _previousActiveDistance;
        float _previousPalmAlignment;
        float _bestDistance;
        float _minimumTransitClearance;
        float _episodeSeconds;
        float _lockHoldSeconds;
        int _evaluationEpisode = -1;
        bool _episodeActive;
        bool _unsafeSurfaceContact;
        bool _outcomeRecorded;

        public ReachPhase CurrentPhase { get; private set; } = ReachPhase.Transit;
        public bool IsArmLocked => CurrentPhase == ReachPhase.Locked;
        public bool IsEpisodeActive => _episodeActive;
        public float CurrentEpisodeSeconds => _episodeSeconds;
        public float CurrentDistance => CenterDistance();
        public float CurrentPlanarDistance => PlanarDistance();
        public float CurrentFloorClearance => FloorClearance();
        public float CurrentPalmAlignment => PalmAlignment();
        public float CurrentUpperConeAlignment => UpperConeAlignment();
        public float CurrentLockHoldSeconds => _lockHoldSeconds;
        public float MinimumTransitClearance => _minimumTransitClearance;
        public float BestDistance => _bestDistance;
        public bool LastEpisodeSucceeded { get; private set; }
        public string LastTerminationReason { get; private set; } = "None";
        public Vector3 CurrentTargetLocalPosition =>
            robotBase == null || target == null
                ? Vector3.zero
                : robotBase.InverseTransformPoint(target.position);
        public Vector3 CurrentGraspPointLocalPosition =>
            robotBase == null || graspPoint == null
                ? Vector3.zero
                : robotBase.InverseTransformPoint(graspPoint.position);
        public Vector3 CurrentPreGraspPoint =>
            target == null ? Vector3.zero : Dg5fReachSpec.PreGraspPoint(target.position);
        public Vector3 CurrentGraspPointVelocity => GraspPointVelocity();

        public float CurrentArmTargetDeg(int index)
        {
            if (_armTargetDeg == null)
                throw new InvalidOperationException("Agent has not initialized.");
            return _armTargetDeg[index];
        }

        public float OpenHandTargetDeg(int index)
        {
            if (_handJoints == null || index < 0 || index >= _handJoints.Length)
                throw new ArgumentOutOfRangeException(nameof(index));
            return _openHandTargetDeg[_handJoints[index]];
        }

        public override void Initialize()
        {
            ResolveReferences();
            ResolveJoints();
            ResolveRobotColliders();
            ResolveSafetySensors();
            ValidateConfiguration();

            _armTargetDeg = new float[Dg5fReachSpec.ArmJointCount];
            _lockedArmTargetDeg = new float[Dg5fReachSpec.ArmJointCount];
            for (int index = 0; index < _armJoints.Length; index++)
            {
                float initial = Dg5fReachSpec.ClampJointTarget(
                    _armJoints[index].xDrive.target,
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Dg5fReachSpec.ArmSafeMaxDeg[index]);
                _initialArmTargetDeg.Add(_armJoints[index], initial);
                _armTargetDeg[index] = initial;
            }
            foreach (ArticulationBody handJoint in _handJoints)
                _openHandTargetDeg.Add(handJoint, handJoint.xDrive.target);

            MaxStep = 0;
            int randomSeed = useDeterministicSpawns
                ? spawnSeed
                : unchecked(spawnSeed * 397
                    ^ UnityEngine.Random.Range(0, int.MaxValue));
            _random = new System.Random(randomSeed);
            _stats = Academy.Instance.StatsRecorder;
            ArmReachEvaluationSession.Register(this);
        }

        void ResolveReferences()
        {
            if (robotBase == null) robotBase = transform;
            Transform[] transforms = GetComponentsInChildren<Transform>(true);
            if (palm == null) palm = FindTransform(transforms, "ll_dg_palm");
            if (graspPoint == null)
                graspPoint = FindTransform(transforms, "GraspPoint");
            if (graspPointBody == null && graspPoint != null)
                graspPointBody = graspPoint.GetComponentInParent<ArticulationBody>();
            if (target == null)
            {
                GameObject found = GameObject.Find("ReachTarget")
                    ?? GameObject.Find("Target")
                    ?? GameObject.Find("RedBall");
                if (found != null) target = found.transform;
            }
            if (targetBody == null && target != null)
                targetBody = target.GetComponent<Rigidbody>();
        }

        void ResolveJoints()
        {
            ArticulationBody[] bodies = GetComponentsInChildren<ArticulationBody>(true);
            _armJoints = new ArticulationBody[Dg5fReachSpec.ArmJointCount];
            for (int index = 0; index < _armJoints.Length; index++)
                _armJoints[index] = FindBody(bodies, Dg5fReachSpec.ArmLinks[index]);

            _handJoints = new ArticulationBody[Dg5fReachSpec.HandJointCount];
            for (int finger = 1; finger <= 5; finger++)
            {
                for (int joint = 1; joint <= 4; joint++)
                {
                    int channel = (finger - 1) * 4 + joint - 1;
                    _handJoints[channel] = FindBodyBySuffix(
                        bodies,
                        $"_dg_{finger}_{joint}");
                }
            }
        }

        void ResolveRobotColliders()
        {
            if (robotColliders != null && robotColliders.Length > 0) return;
            var enabledSolids = new List<Collider>();
            foreach (Collider item in GetComponentsInChildren<Collider>(true))
                if (item != null && item.enabled && !item.isTrigger)
                    enabledSolids.Add(item);
            robotColliders = enabledSolids.ToArray();
        }

        void ResolveSafetySensors()
        {
            if (safetySensors != null && safetySensors.Length > 0) return;
            safetySensors = GetComponentsInChildren<ReachSurfaceContactSensor>(true);
        }

        void ValidateConfiguration()
        {
            if (target == null || targetBody == null || robotBase == null
                || palm == null || graspPoint == null || graspPointBody == null
                || panelCollider == null)
            {
                throw new InvalidOperationException(
                    "[Dg5fGraspPointReachAgent] target/body, robotBase, palm, "
                    + "graspPoint/body, and panelCollider are required.");
            }
            if (!targetBody.isKinematic)
                throw new InvalidOperationException(
                    "[Dg5fGraspPointReachAgent] Target body must be kinematic.");
            for (int index = 0; index < _armJoints.Length; index++)
                if (_armJoints[index] == null)
                    throw new InvalidOperationException(
                        $"[Dg5fGraspPointReachAgent] Missing arm joint "
                        + $"'{Dg5fReachSpec.ArmLinks[index]}'.");
            for (int index = 0; index < _handJoints.Length; index++)
                if (_handJoints[index] == null)
                    throw new InvalidOperationException(
                        $"[Dg5fGraspPointReachAgent] Missing hand joint {index}.");
            if (safetySensors == null || safetySensors.Length == 0)
                throw new InvalidOperationException(
                    "[Dg5fGraspPointReachAgent] Moving-link safety sensors are required.");
            if (!Dg5fReachSpec.IsFinite(targetCenterLocalY))
                throw new InvalidOperationException(
                    "[Dg5fGraspPointReachAgent] Target height must be finite.");
        }

        public override void OnEpisodeBegin()
        {
            _episodeActive = false;
            if (ArmReachEvaluationSession.IsEnabled)
            {
                if (!ArmReachEvaluationSession.TryBeginEpisode(
                        this,
                        out _evaluationEpisode,
                        out int evaluationSeed))
                {
                    DisableAfterEvaluationQuota();
                    return;
                }
                _random = new System.Random(evaluationSeed);
                useDeterministicSpawns = true;
            }
            else
            {
                _evaluationEpisode = -1;
            }

            CurrentPhase = ReachPhase.Transit;
            _unsafeSurfaceContact = false;
            _outcomeRecorded = false;
            foreach (ReachSurfaceContactSensor sensor in safetySensors)
                if (sensor != null) sensor.ResetContacts();

            ResetRobot();
            Physics.SyncTransforms();
            ResetTarget();
            Physics.SyncTransforms();

            _episodeSeconds = 0f;
            _lockHoldSeconds = 0f;
            _previousActiveDistance = ActiveWaypointDistance();
            _previousPalmAlignment = PalmAlignment();
            _bestDistance = CenterDistance();
            _minimumTransitClearance = FloorClearance();
            LastEpisodeSucceeded = false;
            LastTerminationReason = "None";
            _episodeActive = true;
        }

        void ResetRobot()
        {
            for (int index = 0; index < _armJoints.Length; index++)
            {
                float targetDeg = Dg5fReachSpec.ClampJointTarget(
                    _initialArmTargetDeg[_armJoints[index]],
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Dg5fReachSpec.ArmSafeMaxDeg[index]);
                SynchronizeJointState(_armJoints[index], targetDeg);
                _armTargetDeg[index] = _armJoints[index].xDrive.target;
                _lockedArmTargetDeg[index] = _armTargetDeg[index];
            }
            foreach (ArticulationBody handJoint in _handJoints)
                SynchronizeJointState(handJoint, _openHandTargetDeg[handJoint]);
        }

        static void SynchronizeJointState(ArticulationBody body, float targetDeg)
        {
            ArticulationDrive drive = body.xDrive;
            float synchronized = Dg5fReachSpec.ClampJointTarget(
                targetDeg,
                drive.lowerLimit,
                drive.upperLimit);
            drive.target = synchronized;
            body.xDrive = drive;
            body.jointPosition =
                new ArticulationReducedSpace(synchronized * Mathf.Deg2Rad);
            body.jointVelocity = new ArticulationReducedSpace(0f);
        }

        void ResetTarget()
        {
            Vector3 initialGraspLocal =
                robotBase.InverseTransformPoint(graspPoint.position);
            Vector3 targetLocal = Dg5fReachSpec.SpawnTargetLocalPosition(
                _random,
                initialGraspLocal,
                targetCenterLocalY,
                IsSceneValidSpawn);
            targetBody.position = robotBase.TransformPoint(targetLocal);
            targetBody.rotation = Quaternion.identity;
        }

        bool IsSceneValidSpawn(Vector3 targetLocal)
        {
            Vector3 worldCenter = robotBase.TransformPoint(targetLocal);
            if (!Dg5fReachSpec.IsTargetSphereWithinPanel(
                    worldCenter,
                    Dg5fReachSpec.TargetRadius,
                    panelCollider)) return false;

            if (robotColliders == null) return true;
            float radiusSquared =
                Dg5fReachSpec.TargetRadius * Dg5fReachSpec.TargetRadius;
            foreach (Collider robotCollider in robotColliders)
            {
                if (robotCollider == null || !robotCollider.enabled
                    || robotCollider.isTrigger
                    || !robotCollider.gameObject.activeInHierarchy) continue;
                Vector3 closestPoint = robotCollider.ClosestPoint(worldCenter);
                if ((closestPoint - worldCenter).sqrMagnitude < radiusSquared)
                    return false;
            }
            return true;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            if (!CanCollectFiniteObservations())
            {
                AddZeroObservations(sensor);
                return;
            }

            for (int index = 0; index < _armJoints.Length; index++)
            {
                float positionDeg =
                    FirstOrZero(_armJoints[index].jointPosition) * Mathf.Rad2Deg;
                sensor.AddObservation(Dg5fReachSpec.NormalizeJoint(
                    positionDeg,
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Dg5fReachSpec.ArmSafeMaxDeg[index]));
            }
            for (int index = 0; index < _armJoints.Length; index++)
                sensor.AddObservation(Mathf.Clamp(
                    FirstOrZero(_armJoints[index].jointVelocity) / Mathf.PI,
                    -1f,
                    1f));
            for (int index = 0; index < _armTargetDeg.Length; index++)
                sensor.AddObservation(Dg5fReachSpec.NormalizeJoint(
                    _armTargetDeg[index],
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Dg5fReachSpec.ArmSafeMaxDeg[index]));

            AddClampedVector(
                sensor,
                robotBase.InverseTransformDirection(
                    target.position - graspPoint.position),
                Dg5fReachSpec.WorkspaceRadius);
            AddClampedVector(
                sensor,
                robotBase.InverseTransformDirection(
                    ActiveWaypoint() - graspPoint.position),
                Dg5fReachSpec.WorkspaceRadius);
            sensor.AddObservation(Mathf.Clamp01(
                CenterDistance() / Dg5fReachSpec.WorkspaceRadius));
            sensor.AddObservation(Mathf.Clamp01(
                PlanarDistance() / Dg5fReachSpec.WorkspaceRadius));
            sensor.AddObservation(Mathf.Clamp(
                FloorClearance() / 0.20f,
                -1f,
                1f));
            AddClampedVector(
                sensor,
                robotBase.InverseTransformDirection(GraspPointVelocity()),
                1f);
            AddClampedVector(
                sensor,
                palm.InverseTransformDirection(target.position - palm.position),
                0.20f);
            sensor.AddObservation(PalmAlignment());
            sensor.AddObservation(UpperConeAlignment());
            sensor.AddObservation((int)CurrentPhase - 1f);
            sensor.AddObservation(Dg5fReachSpec.LockHoldProgress(_lockHoldSeconds));
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            ActionSegment<float> continuous = actions.ContinuousActions;
            if (continuous.Length != Dg5fReachSpec.ActionSize)
                throw new InvalidOperationException(
                    $"Expected {Dg5fReachSpec.ActionSize} continuous actions, "
                    + $"got {continuous.Length}.");
            if (!_episodeActive) return;

            ApplyOpenHandTargets();
            if (IsArmLocked)
            {
                ApplyLockedArmTargets();
                return;
            }

            float activeDistance = ActiveWaypointDistance();
            float alignment = PalmAlignment();
            AddReward(Dg5fReachSpec.DecisionReward(
                _previousActiveDistance,
                activeDistance,
                CurrentPhase,
                _previousPalmAlignment,
                alignment));
            _previousActiveDistance = activeDistance;
            _previousPalmAlignment = alignment;

            float maximumDelta = Mathf.Min(
                Dg5fReachSpec.ArmDeltaDeg(activeDistance),
                Mathf.Max(0f, armDeltaDegPerDecision));
            for (int index = 0; index < _armJoints.Length; index++)
            {
                ArticulationDrive drive = _armJoints[index].xDrive;
                float lower = Mathf.Max(
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Mathf.Min(drive.lowerLimit, drive.upperLimit));
                float upper = Mathf.Min(
                    Dg5fReachSpec.ArmSafeMaxDeg[index],
                    Mathf.Max(drive.lowerLimit, drive.upperLimit));
                _armTargetDeg[index] = Dg5fReachSpec.AccumulateJointTarget(
                    _armTargetDeg[index],
                    continuous[index],
                    maximumDelta,
                    lower,
                    upper);
                drive.target = _armTargetDeg[index];
                _armJoints[index].xDrive = drive;
            }
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            ActionSegment<float> actions = actionsOut.ContinuousActions;
            for (int index = 0; index < actions.Length; index++)
                actions[index] = 0f;
        }

        void FixedUpdate()
        {
            if (!_episodeActive) return;
            ApplyOpenHandTargets();
            if (IsArmLocked)
            {
                ApplyLockedArmTargets();
                return;
            }

            if (!HasFinitePhysicsState())
            {
                FinishEpisode(false, "NonFinitePhysics", false, false, false);
                return;
            }

            Vector3 graspPointLocal =
                robotBase.InverseTransformPoint(graspPoint.position);
            if (!Dg5fReachSpec.IsWithinWorkspace(graspPointLocal))
            {
                FinishEpisode(false, "WorkspaceExit", true, false, false);
                return;
            }
            if (_unsafeSurfaceContact || HasUnsafeSurfaceContact())
            {
                FinishEpisode(false, "UnsafeSurfaceContact", true, true, true);
                return;
            }

            _episodeSeconds += Time.fixedDeltaTime;
            float clearance = FloorClearance();
            float planarDistance = PlanarDistance();
            if (CurrentPhase == ReachPhase.Transit)
                _minimumTransitClearance = Mathf.Min(
                    _minimumTransitClearance,
                    clearance);
            if (Dg5fReachSpec.IsPrematureDescent(
                    CurrentPhase,
                    planarDistance,
                    clearance))
            {
                FinishEpisode(false, "PrematureDescent", true, true, false);
                return;
            }

            float distance = CenterDistance();
            _bestDistance = Mathf.Min(_bestDistance, distance);
            if (CurrentPhase == ReachPhase.Transit
                && Dg5fReachSpec.CanEnterDescend(
                    ActiveWaypointDistance(),
                    clearance))
            {
                CurrentPhase = ReachPhase.Descend;
                _previousActiveDistance = distance;
                _previousPalmAlignment = PalmAlignment();
                AddReward(Dg5fReachSpec.DescendPhaseReward);
            }

            if (CurrentPhase == ReachPhase.Descend)
            {
                _lockHoldSeconds = Dg5fReachSpec.NextLockHoldSeconds(
                    _lockHoldSeconds,
                    distance,
                    GraspPointVelocity().magnitude,
                    PalmAlignment(),
                    UpperConeAlignment(),
                    Time.fixedDeltaTime);
                if (Dg5fReachSpec.HasCompletedLockHold(_lockHoldSeconds))
                {
                    LockArm();
                    return;
                }
            }

            if (Dg5fReachSpec.ReachedEpisodeTimeout(_episodeSeconds))
                FinishEpisode(false, "Timeout", true, true, false);
        }

        public void NotifyUnsafeSurfaceContact(Collider surface)
        {
            if (surface == panelCollider) _unsafeSurfaceContact = true;
        }

        public void ReleaseArmLock()
        {
            if (!IsArmLocked) return;
            CurrentPhase = ReachPhase.Transit;
            _lockHoldSeconds = 0f;
            _previousActiveDistance = ActiveWaypointDistance();
            _previousPalmAlignment = PalmAlignment();
            _outcomeRecorded = false;
        }

        void LockArm()
        {
            for (int index = 0; index < _armTargetDeg.Length; index++)
                _lockedArmTargetDeg[index] = _armJoints[index].xDrive.target;
            CurrentPhase = ReachPhase.Locked;
            ApplyLockedArmTargets();
            AddReward(Dg5fReachSpec.LockSuccessReward);
            RecordOutcome(true, "Success", true, true, false);
            if (endEpisodeOnLock || ArmReachEvaluationSession.IsEnabled)
            {
                _episodeActive = false;
                EndEpisode();
            }
        }

        void FinishEpisode(
            bool success,
            string reason,
            bool finitePhysics,
            bool workspaceSafe,
            bool unsafeSurfaceContact)
        {
            if (!_episodeActive) return;
            _episodeActive = false;
            AddReward(success
                ? Dg5fReachSpec.LockSuccessReward
                : Dg5fReachSpec.FailurePenalty(reason));
            RecordOutcome(
                success,
                reason,
                finitePhysics,
                workspaceSafe,
                unsafeSurfaceContact);
            EndEpisode();
        }

        void RecordOutcome(
            bool success,
            string reason,
            bool finitePhysics,
            bool workspaceSafe,
            bool unsafeSurfaceContact)
        {
            if (_outcomeRecorded) return;
            _outcomeRecorded = true;
            float finalDistance = SafeNonNegative(CenterDistance());
            float pointSpeed = SafeNonNegative(GraspPointVelocity().magnitude);
            float palmAlignment = SafeAlignment(PalmAlignment());
            float upperConeAlignment = SafeAlignment(UpperConeAlignment());
            if (!Dg5fReachSpec.IsFinite(_bestDistance))
                _bestDistance = finalDistance;
            if (!Dg5fReachSpec.IsFinite(_minimumTransitClearance))
                _minimumTransitClearance = 0f;

            LastEpisodeSucceeded = success;
            LastTerminationReason = reason;
            _stats.Add("Reach/LockSuccess", success ? 1f : 0f,
                StatAggregationMethod.Average);
            _stats.Add("Reach/FinalDistanceMeters", finalDistance,
                StatAggregationMethod.Average);
            _stats.Add("Reach/BestDistanceMeters", _bestDistance,
                StatAggregationMethod.Average);
            _stats.Add("Reach/CompletionSeconds", _episodeSeconds,
                StatAggregationMethod.Average);
            _stats.Add("Reach/PalmAlignment", palmAlignment,
                StatAggregationMethod.Average);
            _stats.Add("Reach/UpperConeAlignment", upperConeAlignment,
                StatAggregationMethod.Average);
            _stats.Add("Reach/MinimumTransitClearanceMeters",
                _minimumTransitClearance,
                StatAggregationMethod.Average);
            if (unsafeSurfaceContact)
                _stats.Add("Failure/UnsafeSurfaceContact", 1f,
                    StatAggregationMethod.Sum);
            if (string.Equals(reason, "PrematureDescent", StringComparison.Ordinal))
                _stats.Add("Failure/PrematureDescent", 1f,
                    StatAggregationMethod.Sum);

            if (_evaluationEpisode >= 0)
            {
                ArmReachEvaluationSession.RecordEpisode(
                    this,
                    _evaluationEpisode,
                    success,
                    finalDistance,
                    pointSpeed,
                    palmAlignment,
                    upperConeAlignment,
                    _lockHoldSeconds,
                    _episodeSeconds,
                    _minimumTransitClearance,
                    unsafeSurfaceContact,
                    string.Equals(
                        reason,
                        "PrematureDescent",
                        StringComparison.Ordinal),
                    workspaceSafe,
                    finitePhysics,
                    reason);
                _evaluationEpisode = -1;
            }
        }

        void ApplyOpenHandTargets()
        {
            if (_handJoints == null) return;
            foreach (ArticulationBody handJoint in _handJoints)
            {
                if (handJoint == null) continue;
                ArticulationDrive drive = handJoint.xDrive;
                drive.target = Dg5fReachSpec.ClampJointTarget(
                    _openHandTargetDeg[handJoint],
                    drive.lowerLimit,
                    drive.upperLimit);
                handJoint.xDrive = drive;
            }
        }

        void ApplyLockedArmTargets()
        {
            for (int index = 0; index < _armJoints.Length; index++)
            {
                ArticulationDrive drive = _armJoints[index].xDrive;
                drive.target = Dg5fReachSpec.ClampJointTarget(
                    _lockedArmTargetDeg[index],
                    drive.lowerLimit,
                    drive.upperLimit);
                _armJoints[index].xDrive = drive;
                _armTargetDeg[index] = drive.target;
            }
        }

        bool HasUnsafeSurfaceContact()
        {
            foreach (ReachSurfaceContactSensor sensor in safetySensors)
                if (sensor != null && sensor.HasUnsafeContact) return true;
            return false;
        }

        bool CanCollectFiniteObservations()
        {
            return robotBase != null && target != null && palm != null
                && graspPoint != null && graspPointBody != null
                && panelCollider != null && _armJoints != null
                && _armTargetDeg != null && HasFinitePhysicsState();
        }

        bool HasFinitePhysicsState()
        {
            if (target == null || palm == null || graspPoint == null
                || graspPointBody == null
                || !Dg5fReachSpec.IsFinite(target.position)
                || !Dg5fReachSpec.IsFinite(palm.position)
                || !Dg5fReachSpec.IsFinite(graspPoint.position)
                || !Dg5fReachSpec.IsFinite(graspPointBody.linearVelocity)
                || !Dg5fReachSpec.IsFinite(graspPointBody.angularVelocity)
                || !Dg5fReachSpec.IsFinite(graspPointBody.centerOfMass)
                || !Dg5fReachSpec.IsFinite(GraspPointVelocity())) return false;

            foreach (ArticulationBody joint in _armJoints)
            {
                if (joint == null
                    || !Dg5fReachSpec.IsFinite(FirstOrZero(joint.jointPosition))
                    || !Dg5fReachSpec.IsFinite(FirstOrZero(joint.jointVelocity))
                    || !Dg5fReachSpec.IsFinite(joint.xDrive.target)) return false;
            }
            foreach (ArticulationBody joint in _handJoints)
                if (joint == null || !Dg5fReachSpec.IsFinite(joint.xDrive.target))
                    return false;
            return true;
        }

        Vector3 ActiveWaypoint()
        {
            return CurrentPhase == ReachPhase.Transit
                ? Dg5fReachSpec.PreGraspPoint(target.position)
                : target.position;
        }

        float ActiveWaypointDistance()
        {
            if (target == null || graspPoint == null) return float.PositiveInfinity;
            return Vector3.Distance(ActiveWaypoint(), graspPoint.position);
        }

        float CenterDistance()
        {
            if (target == null || graspPoint == null) return float.PositiveInfinity;
            return Vector3.Distance(target.position, graspPoint.position);
        }

        float PlanarDistance()
        {
            if (target == null || graspPoint == null) return float.PositiveInfinity;
            return Dg5fReachSpec.PlanarDistance(target.position, graspPoint.position);
        }

        float FloorClearance()
        {
            if (graspPoint == null || panelCollider == null)
                return float.NegativeInfinity;
            return graspPoint.position.y - panelCollider.bounds.max.y;
        }

        float PalmAlignment()
        {
            if (target == null || palm == null || graspPoint == null) return -1f;
            return Dg5fReachSpec.PalmFacingAlignment(
                graspPoint.forward,
                target.position - palm.position);
        }

        float UpperConeAlignment()
        {
            if (target == null || palm == null) return -1f;
            return Dg5fReachSpec.UpperConeAlignment(
                palm.position,
                target.position);
        }

        Vector3 GraspPointVelocity()
        {
            if (graspPointBody == null || graspPoint == null) return Vector3.zero;
            Vector3 worldCenterOfMass =
                graspPointBody.transform.TransformPoint(graspPointBody.centerOfMass);
            return Dg5fReachSpec.PointVelocity(
                graspPointBody.linearVelocity,
                graspPointBody.angularVelocity,
                worldCenterOfMass,
                graspPoint.position);
        }

        void DisableAfterEvaluationQuota()
        {
            _episodeActive = false;
            enabled = false;
            DecisionRequester requester = GetComponent<DecisionRequester>();
            if (requester != null) requester.enabled = false;
        }

        void OnDestroy()
        {
            ArmReachEvaluationSession.Unregister(this);
        }

        static Transform FindTransform(IEnumerable<Transform> transforms, string name)
        {
            foreach (Transform item in transforms)
                if (item.name == name) return item;
            return null;
        }

        static ArticulationBody FindBody(
            IEnumerable<ArticulationBody> bodies,
            string name)
        {
            foreach (ArticulationBody body in bodies)
                if (body.name == name) return body;
            return null;
        }

        static ArticulationBody FindBodyBySuffix(
            IEnumerable<ArticulationBody> bodies,
            string suffix)
        {
            foreach (ArticulationBody body in bodies)
                if (body.name.EndsWith(suffix, StringComparison.Ordinal)) return body;
            return null;
        }

        static float FirstOrZero(ArticulationReducedSpace values)
        {
            try { return values[0]; }
            catch (IndexOutOfRangeException) { return 0f; }
        }

        static float SafeNonNegative(float value)
        {
            return Dg5fReachSpec.IsFinite(value) ? Mathf.Max(0f, value) : 0f;
        }

        static float SafeAlignment(float value)
        {
            return Dg5fReachSpec.IsFinite(value) ? Mathf.Clamp(value, -1f, 1f) : -1f;
        }

        static void AddZeroObservations(VectorSensor sensor)
        {
            for (int index = 0; index < Dg5fReachSpec.ObservationSize; index++)
                sensor.AddObservation(0f);
        }

        static void AddClampedVector(
            VectorSensor sensor,
            Vector3 value,
            float scale)
        {
            float safeScale = Mathf.Max(1e-6f, scale);
            sensor.AddObservation(Mathf.Clamp(value.x / safeScale, -1f, 1f));
            sensor.AddObservation(Mathf.Clamp(value.y / safeScale, -1f, 1f));
            sensor.AddObservation(Mathf.Clamp(value.z / safeScale, -1f, 1f));
        }

        void OnDrawGizmos()
        {
            if (target != null)
            {
                Gizmos.color = new Color(1f, 0.15f, 0.1f, 0.8f);
                Gizmos.DrawWireSphere(target.position, Dg5fReachSpec.TargetRadius);
                Gizmos.color = new Color(1f, 0.8f, 0.1f, 0.8f);
                Gizmos.DrawWireSphere(
                    Dg5fReachSpec.PreGraspPoint(target.position),
                    Dg5fReachSpec.TransitWaypointTolerance);
            }
            if (graspPoint != null)
            {
                Gizmos.color = new Color(0.1f, 0.85f, 1f, 0.8f);
                Gizmos.DrawWireSphere(graspPoint.position, Dg5fReachSpec.SuccessDistance);
            }
        }
    }
}
