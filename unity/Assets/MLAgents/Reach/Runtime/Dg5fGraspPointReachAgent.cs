using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace KDT.ReachTraining
{
    /// <summary>
    /// Positions the DG5F GraspPoint by commanding only the six UR5e arm xDrives.
    /// Hand articulation bodies are neither discovered nor written by this Agent.
    /// </summary>
    public sealed class Dg5fGraspPointReachAgent : Agent
    {
        [Header("Scene references")]
        public Transform target;
        public Transform robotBase;
        public Transform graspPoint;
        public ArticulationBody graspPointBody;
        public Collider panelCollider;
        public Collider[] robotColliders = Array.Empty<Collider>();

        [Header("Episode")]
        public bool useDeterministicSpawns;
        public int spawnSeed = 12345;
        public float targetCenterLocalY = Dg5fReachSpec.TargetRadius;

        [Header("Control")]
        [Range(0f, Dg5fReachSpec.MaximumArmDeltaDegPerDecision)]
        public float armDeltaDegPerDecision =
            Dg5fReachSpec.MaximumArmDeltaDegPerDecision;

        readonly Dictionary<ArticulationBody, float> _initialArmTargetDeg =
            new Dictionary<ArticulationBody, float>();

        ArticulationBody[] _armJoints;
        float[] _armTargetDeg;
        System.Random _random;
        StatsRecorder _stats;
        float _previousDistance;
        float _bestDistance;
        float _episodeSeconds;
        float _successHoldSeconds;
        int _evaluationEpisode = -1;
        bool _episodeActive;

        public bool IsEpisodeActive => _episodeActive;
        public float CurrentEpisodeSeconds => _episodeSeconds;
        public float CurrentDistance => CenterDistance();
        public float BestDistance => _bestDistance;
        public float CurrentSuccessHoldSeconds => _successHoldSeconds;
        public Vector3 CurrentTargetLocalPosition =>
            robotBase == null || target == null
                ? Vector3.zero
                : robotBase.InverseTransformPoint(target.position);
        public Vector3 CurrentGraspPointLocalPosition =>
            robotBase == null || graspPoint == null
                ? Vector3.zero
                : robotBase.InverseTransformPoint(graspPoint.position);
        public Vector3 CurrentGraspPointVelocity => GraspPointVelocity();
        public bool LastEpisodeSucceeded { get; private set; }
        public string LastTerminationReason { get; private set; } = "None";

        public float CurrentArmTargetDeg(int index)
        {
            if (_armTargetDeg == null)
                throw new InvalidOperationException("Agent has not initialized.");
            return _armTargetDeg[index];
        }

        public override void Initialize()
        {
            ResolveReferences();
            ResolveArmJoints();
            ResolveRobotColliders();
            ValidateConfiguration();

            _armTargetDeg = new float[Dg5fReachSpec.ArmJointCount];
            for (int index = 0; index < _armJoints.Length; index++)
            {
                float initial = Dg5fReachSpec.ClampJointTarget(
                    _armJoints[index].xDrive.target,
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Dg5fReachSpec.ArmSafeMaxDeg[index]);
                _initialArmTargetDeg.Add(_armJoints[index], initial);
                _armTargetDeg[index] = initial;
            }

            // The exact limit is simulation time rather than an ML-Agents step count.
            MaxStep = 0;
            int randomSeed = useDeterministicSpawns
                ? spawnSeed
                : unchecked(spawnSeed * 397 ^ UnityEngine.Random.Range(0, int.MaxValue));
            _random = new System.Random(randomSeed);
            _stats = Academy.Instance.StatsRecorder;
            ArmReachEvaluationSession.Register(this);
        }

        void ResolveReferences()
        {
            if (robotBase == null) robotBase = transform;
            Transform[] transforms = GetComponentsInChildren<Transform>(true);
            if (graspPoint == null) graspPoint = FindTransform(transforms, "GraspPoint");
            if (graspPointBody == null && graspPoint != null)
                graspPointBody = graspPoint.GetComponentInParent<ArticulationBody>();
            if (target == null)
            {
                GameObject found = GameObject.Find("ReachTarget")
                    ?? GameObject.Find("Target")
                    ?? GameObject.Find("RedBall");
                if (found != null) target = found.transform;
            }
        }

        void ResolveArmJoints()
        {
            ArticulationBody[] bodies = GetComponentsInChildren<ArticulationBody>(true);
            _armJoints = new ArticulationBody[Dg5fReachSpec.ArmJointCount];
            for (int index = 0; index < _armJoints.Length; index++)
                _armJoints[index] = FindBody(bodies, Dg5fReachSpec.ArmLinks[index]);
        }

        void ResolveRobotColliders()
        {
            if (robotColliders != null && robotColliders.Length > 0) return;

            Collider[] discovered = GetComponentsInChildren<Collider>(true);
            var enabledSolids = new List<Collider>();
            foreach (Collider item in discovered)
            {
                if (item != null && item.enabled && !item.isTrigger)
                    enabledSolids.Add(item);
            }
            robotColliders = enabledSolids.ToArray();
        }

        void ValidateConfiguration()
        {
            if (target == null || robotBase == null || graspPoint == null
                || graspPointBody == null || panelCollider == null)
            {
                throw new InvalidOperationException(
                    "[Dg5fGraspPointReachAgent] target, robotBase, graspPoint, and "
                    + "graspPointBody, and panelCollider are required.");
            }
            for (int index = 0; index < _armJoints.Length; index++)
            {
                if (_armJoints[index] == null)
                    throw new InvalidOperationException(
                        $"[Dg5fGraspPointReachAgent] Missing arm joint "
                        + $"'{Dg5fReachSpec.ArmLinks[index]}'.");
            }
            if (!Dg5fReachSpec.IsFinite(targetCenterLocalY))
                throw new InvalidOperationException(
                    "[Dg5fGraspPointReachAgent] Target center height must be finite.");
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

            ResetArm();
            Physics.SyncTransforms();
            ResetTarget();
            Physics.SyncTransforms();

            _episodeSeconds = 0f;
            _successHoldSeconds = 0f;
            _previousDistance = CenterDistance();
            _bestDistance = _previousDistance;
            LastEpisodeSucceeded = false;
            LastTerminationReason = "None";
            _episodeActive = true;
        }

        void ResetArm()
        {
            for (int index = 0; index < _armJoints.Length; index++)
            {
                float targetDeg = Dg5fReachSpec.ClampJointTarget(
                    _initialArmTargetDeg[_armJoints[index]],
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Dg5fReachSpec.ArmSafeMaxDeg[index]);
                SynchronizeJointState(_armJoints[index], targetDeg);
                _armTargetDeg[index] = _armJoints[index].xDrive.target;
            }
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
            target.position = robotBase.TransformPoint(targetLocal);
        }

        bool IsSceneValidSpawn(Vector3 targetLocal)
        {
            Vector3 worldCenter = robotBase.TransformPoint(targetLocal);
            if (!Dg5fReachSpec.IsTargetSphereWithinPanel(
                    worldCenter,
                    Dg5fReachSpec.TargetRadius,
                    panelCollider))
            {
                return false;
            }

            if (robotColliders == null) return true;
            float radiusSquared =
                Dg5fReachSpec.TargetRadius * Dg5fReachSpec.TargetRadius;
            foreach (Collider robotCollider in robotColliders)
            {
                if (robotCollider == null
                    || !robotCollider.enabled
                    || robotCollider.isTrigger
                    || !robotCollider.gameObject.activeInHierarchy)
                {
                    continue;
                }

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

            // 0..5: normalized arm joint position.
            for (int index = 0; index < _armJoints.Length; index++)
            {
                float positionDeg =
                    FirstOrZero(_armJoints[index].jointPosition) * Mathf.Rad2Deg;
                sensor.AddObservation(Dg5fReachSpec.NormalizeJoint(
                    positionDeg,
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Dg5fReachSpec.ArmSafeMaxDeg[index]));
            }

            // 6..11: arm joint velocity, normalized around pi rad/s.
            for (int index = 0; index < _armJoints.Length; index++)
            {
                sensor.AddObservation(Mathf.Clamp(
                    FirstOrZero(_armJoints[index].jointVelocity) / Mathf.PI,
                    -1f,
                    1f));
            }

            // 12..17: normalized commanded arm xDrive target.
            for (int index = 0; index < _armTargetDeg.Length; index++)
            {
                sensor.AddObservation(Dg5fReachSpec.NormalizeJoint(
                    _armTargetDeg[index],
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Dg5fReachSpec.ArmSafeMaxDeg[index]));
            }

            // 18..20: target - GraspPoint in robot-base coordinates.
            Vector3 targetOffsetLocal = robotBase.InverseTransformDirection(
                target.position - graspPoint.position);
            AddClampedVector(
                sensor,
                targetOffsetLocal,
                Dg5fReachSpec.WorkspaceRadius);

            // 21: target center distance.
            sensor.AddObservation(Mathf.Clamp01(
                CenterDistance() / Dg5fReachSpec.WorkspaceRadius));

            // 22..24: velocity of the actual offset GraspPoint, not its parent origin.
            Vector3 pointVelocityLocal = robotBase.InverseTransformDirection(
                GraspPointVelocity());
            AddClampedVector(sensor, pointVelocityLocal, 1f);

            // 25: low-speed in-tolerance hold progress.
            sensor.AddObservation(
                Dg5fReachSpec.SuccessHoldProgress(_successHoldSeconds));
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            ActionSegment<float> continuous = actions.ContinuousActions;
            if (continuous.Length != Dg5fReachSpec.ActionSize)
                throw new InvalidOperationException(
                    $"Expected {Dg5fReachSpec.ActionSize} continuous actions, "
                    + $"got {continuous.Length}.");
            if (!_episodeActive) return;

            float distance = CenterDistance();
            AddReward(Dg5fReachSpec.DecisionReward(_previousDistance, distance));
            _previousDistance = distance;
            if (Dg5fReachSpec.IsFinite(distance))
                _bestDistance = Mathf.Min(_bestDistance, distance);

            float maximumDelta = Mathf.Min(
                Dg5fReachSpec.MaximumArmDeltaDegPerDecision,
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

        void FixedUpdate()
        {
            if (!_episodeActive) return;

            bool finitePhysics = HasFinitePhysicsState();
            if (!finitePhysics)
            {
                FinishEpisode(
                    false,
                    "NonFinitePhysics",
                    false,
                    false);
                return;
            }

            Vector3 graspPointLocal =
                robotBase.InverseTransformPoint(graspPoint.position);
            bool workspaceSafe = Dg5fReachSpec.IsWithinWorkspace(graspPointLocal);
            if (!workspaceSafe)
            {
                FinishEpisode(
                    false,
                    "WorkspaceExit",
                    true,
                    false);
                return;
            }

            _episodeSeconds += Time.fixedDeltaTime;
            float distance = CenterDistance();
            float pointSpeed = GraspPointVelocity().magnitude;
            _bestDistance = Mathf.Min(_bestDistance, distance);
            _successHoldSeconds = Dg5fReachSpec.NextSuccessHoldSeconds(
                _successHoldSeconds,
                distance,
                pointSpeed,
                Time.fixedDeltaTime);

            if (Dg5fReachSpec.HasCompletedSuccessHold(_successHoldSeconds))
            {
                FinishEpisode(true, "Success", true, true);
                return;
            }
            if (Dg5fReachSpec.ReachedEpisodeTimeout(_episodeSeconds))
                FinishEpisode(false, "Timeout", true, true);
        }

        void FinishEpisode(
            bool success,
            string terminationReason,
            bool finitePhysics,
            bool workspaceSafe)
        {
            if (!_episodeActive) return;
            _episodeActive = false;

            float finalDistance = CenterDistance();
            float pointSpeed = GraspPointVelocity().magnitude;
            if (!Dg5fReachSpec.IsFinite(finalDistance))
                finalDistance = Dg5fReachSpec.WorkspaceRadius;
            if (!Dg5fReachSpec.IsFinite(pointSpeed)) pointSpeed = 0f;
            if (!Dg5fReachSpec.IsFinite(_bestDistance))
                _bestDistance = finalDistance;

            LastEpisodeSucceeded = success;
            LastTerminationReason = terminationReason;
            AddReward(success
                ? Dg5fReachSpec.SuccessReward(_episodeSeconds, finalDistance)
                : Dg5fReachSpec.FailurePenalty(terminationReason));

            _stats.Add(
                "Reach/Success",
                success ? 1f : 0f,
                StatAggregationMethod.Average);
            _stats.Add(
                "Reach/FinalDistanceMeters",
                finalDistance,
                StatAggregationMethod.Average);
            _stats.Add(
                "Reach/BestDistanceMeters",
                _bestDistance,
                StatAggregationMethod.Average);
            _stats.Add(
                "Reach/CompletionSeconds",
                _episodeSeconds,
                StatAggregationMethod.Average);

            if (_evaluationEpisode >= 0)
            {
                ArmReachEvaluationSession.RecordEpisode(
                    this,
                    _evaluationEpisode,
                    success,
                    finalDistance,
                    pointSpeed,
                    _successHoldSeconds,
                    _episodeSeconds,
                    workspaceSafe,
                    finitePhysics,
                    terminationReason);
                _evaluationEpisode = -1;
            }
            EndEpisode();
        }

        bool CanCollectFiniteObservations()
        {
            return robotBase != null
                && target != null
                && graspPoint != null
                && graspPointBody != null
                && _armJoints != null
                && _armTargetDeg != null
                && HasFinitePhysicsState();
        }

        bool HasFinitePhysicsState()
        {
            if (target == null || graspPoint == null || graspPointBody == null
                || !Dg5fReachSpec.IsFinite(target.position)
                || !Dg5fReachSpec.IsFinite(graspPoint.position)
                || !Dg5fReachSpec.IsFinite(graspPointBody.linearVelocity)
                || !Dg5fReachSpec.IsFinite(graspPointBody.angularVelocity)
                || !Dg5fReachSpec.IsFinite(graspPointBody.centerOfMass)
                || !Dg5fReachSpec.IsFinite(GraspPointVelocity()))
            {
                return false;
            }
            for (int index = 0; index < _armJoints.Length; index++)
            {
                if (_armJoints[index] == null
                    || !Dg5fReachSpec.IsFinite(
                        FirstOrZero(_armJoints[index].jointPosition))
                    || !Dg5fReachSpec.IsFinite(
                        FirstOrZero(_armJoints[index].jointVelocity))
                    || !Dg5fReachSpec.IsFinite(_armJoints[index].xDrive.target)
                    || !Dg5fReachSpec.IsFinite(_armTargetDeg[index]))
                {
                    return false;
                }
            }
            return true;
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

        float CenterDistance()
        {
            if (target == null || graspPoint == null) return float.PositiveInfinity;
            return Vector3.Distance(target.position, graspPoint.position);
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
            if (graspPoint != null)
            {
                Gizmos.color = new Color(0.1f, 0.85f, 1f, 0.8f);
                Gizmos.DrawWireSphere(
                    graspPoint.position,
                    Dg5fReachSpec.SuccessDistance);
            }
            if (target != null)
            {
                Gizmos.color = new Color(1f, 0.15f, 0.1f, 0.8f);
                Gizmos.DrawWireSphere(target.position, Dg5fReachSpec.TargetRadius);
            }
        }
    }
}
