using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace KDT.GraspLiftTraining
{
    /// <summary>
    /// PPO agent for the DG5F grasp + lift stage on a UR5e.
    ///
    /// The policy commands the 6 arm joints and a single hand-closure scalar that
    /// interpolates all 20 DG5F finger joints between the prefab's open pose and the
    /// validated <see cref="Dg5fGraspLiftSpec.LeftFistDeg"/> power-grasp pose. This
    /// component is the sole xDrive writer in the training scene.
    ///
    /// The episode is a single continuous task (no scripted lift phase): the policy
    /// must approach, close on the block, satisfy a geometric grasp contract, and then
    /// raise the block itself.
    /// </summary>
    public sealed class Dg5fGraspLiftAgent : Agent
    {
        [Header("Scene references")]
        public Rigidbody graspObject;
        public Transform pedestal;
        public Collider pedestalCollider;
        public Transform robotBase;
        public Transform palm;
        public Transform graspPoint;
        public Transform[] fingerTips = new Transform[Dg5fGraspLiftSpec.FingerCount];
        // Variable length. The URDF importer parents every collider under a
        // "Collisions" child of its link, so a single sensor on the link GameObject
        // would never receive OnCollision*. The scene builder therefore instruments
        // the link *and* each of its colliders, all tagged with the same
        // contactIndex (0..4 fingertips, 5 palm); a contact point counts as touching
        // when any of its sensors reports contact.
        public GraspLiftObjectContactSensor[] contactSensors =
            Array.Empty<GraspLiftObjectContactSensor>();
        public GraspLiftSurfaceContactSensor[] safetySensors =
            Array.Empty<GraspLiftSurfaceContactSensor>();
        public GraspLiftHandSurfaceSensor[] handSurfaceSensors =
            Array.Empty<GraspLiftHandSurfaceSensor>();

        [Header("Episode")]
        public bool useDeterministicSpawns;
        public int spawnSeed = 12345;

        [Header("Control")]
        public float armDeltaDegPerDecision = 2f;
        public float gripDeltaPerDecision = 0.08f;

        readonly Dictionary<ArticulationBody, float> _initialTargetDeg =
            new Dictionary<ArticulationBody, float>();

        ArticulationBody[] _allJoints;
        ArticulationBody[] _armJoints;
        ArticulationBody[] _handJoints;
        Collider _objectCollider;
        float[] _armTargetDeg;
        float[] _openHandDeg;
        readonly Vector3[] _contactDirections =
            new Vector3[Dg5fGraspLiftSpec.ContactPointCount];

        float _closure;
        float _episodeSeconds;
        float _spawnObjectHeight;
        Vector3 _spawnObjectLocalPosition;

        float _previousApproachPotential;
        float _bestTopDownPotential;
        float _bestClosurePotential;
        float _bestContactPotential;
        float _bestGraspPotential;
        float _previousLiftPotential;

        float _graspSeconds;
        float _liftHoldSeconds;
        float _slipSeconds;
        float _bestLiftHeight;
        float _maxObjectTiltDeg;
        float _handSurfaceContactSeconds;
        float _handSurfaceContactSecondsSinceDecision;
        float _sumSquaredArmActionDeltas;
        float _sumGraspPostureAngleDegrees;
        int _armActionDecisionCount;
        int _graspPostureAngleSampleCount;
        int _contactCount;
        Vector3 _contactCentroid;
        readonly float[] _previousArmActions =
            new float[Dg5fGraspLiftSpec.ArmJointCount];

        int _objectReleaseFixedSteps;
        bool _hasPreviousArmAction;
        bool _episodeActive;
        bool _graspConfirmed;
        bool _unsafeSurfaceContact;
        bool _resolved;

        System.Random _random;
        StatsRecorder _stats;

        public float CurrentClosure => _closure;
        public float CurrentEpisodeSeconds => _episodeSeconds;
        public bool IsGraspConfirmed => _graspConfirmed;
        public float CurrentGraspSeconds => _graspSeconds;
        public int CurrentContactCount => _contactCount;
        public float CurrentLiftHeight => LiftHeight();
        public float BestLiftHeight => _bestLiftHeight;
        public bool IsEpisodeActive => _episodeActive;
        public Vector3 CurrentObjectLocalPosition =>
            robotBase != null && graspObject != null
                ? robotBase.InverseTransformPoint(graspObject.position)
                : Vector3.zero;
        public string LastTerminationReason { get; private set; } = "None";

        public float CurrentArmTargetDeg(int index)
        {
            if (_armTargetDeg == null)
                throw new InvalidOperationException("Agent has not initialized.");
            return _armTargetDeg[index];
        }

        public override void Initialize()
        {
            EnsureResolved();
        }

        // ML-Agents can fire OnEpisodeBegin (via Academy.AgentForceReset) before
        // Initialize(). Both entry points funnel through the same idempotent setup,
        // and _resolved is only latched once every step succeeded so a partially
        // built robot hierarchy is retried instead of silently skipped.
        void EnsureResolved()
        {
            if (_resolved) return;

            ResolveReferences();
            _objectCollider = graspObject != null ? graspObject.GetComponent<Collider>() : null;
            ResolveJoints();
            ResolveSafetySensors();
            ValidateConfiguration();

            _armTargetDeg = new float[Dg5fGraspLiftSpec.ArmJointCount];
            _openHandDeg = new float[Dg5fGraspLiftSpec.HandJointCount];
            foreach (var body in _allJoints)
            {
                // Drive state can still read stale for a few joints this early in the
                // lifecycle; InitialTargetDeg() fills in whatever this loop misses.
                try { _initialTargetDeg[body] = body.xDrive.target; }
                catch (Exception) { /* filled in lazily by InitialTargetDeg */ }
            }
            for (int i = 0; i < _handJoints.Length; i++)
                _openHandDeg[i] = InitialTargetDeg(_handJoints[i]);

            // Episode length is measured in simulation seconds, not ML-Agents steps.
            MaxStep = 0;
            _random = new System.Random(spawnSeed);
            _stats = Academy.Instance.StatsRecorder;

            _resolved = true;
        }

        void ResolveReferences()
        {
            if (robotBase == null) robotBase = transform;
            if (pedestal == null)
            {
                var found = GameObject.Find("GraspPanel") ?? GameObject.Find("GraspLiftPanel");
                if (found != null) pedestal = found.transform;
            }
            if (pedestalCollider == null && pedestal != null)
                pedestalCollider = pedestal.GetComponent<Collider>();

            var transforms = GetComponentsInChildren<Transform>(true);
            if (palm == null) palm = FindByName(transforms, "ll_dg_palm");
            if (graspPoint == null) graspPoint = FindByName(transforms, "GraspPoint");

            if (fingerTips == null || fingerTips.Length != Dg5fGraspLiftSpec.FingerCount)
                fingerTips = new Transform[Dg5fGraspLiftSpec.FingerCount];
            for (int finger = 0; finger < Dg5fGraspLiftSpec.FingerCount; finger++)
            {
                if (fingerTips[finger] == null)
                    fingerTips[finger] = FindByName(transforms, $"ll_dg_{finger + 1}_tip");
            }

            if (contactSensors == null || contactSensors.Length == 0)
                contactSensors = GetComponentsInChildren<GraspLiftObjectContactSensor>(true);
            foreach (var sensor in contactSensors)
                if (sensor != null) sensor.targetObject = graspObject;
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

            _armJoints = new ArticulationBody[Dg5fGraspLiftSpec.ArmJointCount];
            for (int i = 0; i < _armJoints.Length; i++)
                _armJoints[i] = FindBody(bodies, Dg5fGraspLiftSpec.ArmLinks[i]);

            _handJoints = new ArticulationBody[Dg5fGraspLiftSpec.HandJointCount];
            for (int finger = 1; finger <= Dg5fGraspLiftSpec.FingerCount; finger++)
                for (int joint = 1; joint <= 4; joint++)
                {
                    int channel = (finger - 1) * 4 + joint - 1;
                    _handJoints[channel] = FindBodyBySuffix(bodies, $"_dg_{finger}_{joint}");
                }

            var all = new List<ArticulationBody>(_armJoints.Length + _handJoints.Length);
            foreach (var body in _armJoints)
                if (body != null) all.Add(body);
            foreach (var body in _handJoints)
                if (body != null) all.Add(body);
            _allJoints = all.ToArray();
        }

        void ResolveSafetySensors()
        {
            if (safetySensors == null || safetySensors.Length == 0)
                safetySensors = GetComponentsInChildren<GraspLiftSurfaceContactSensor>(true);
            if (handSurfaceSensors == null || handSurfaceSensors.Length == 0)
                handSurfaceSensors =
                    GetComponentsInChildren<GraspLiftHandSurfaceSensor>(true);
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
            if (graspObject == null || pedestal == null || pedestalCollider == null
                || robotBase == null || palm == null || graspPoint == null
                || _objectCollider == null)
            {
                throw new InvalidOperationException(
                    "[Dg5fGraspLiftAgent] Missing graspObject/pedestal/robotBase/palm/graspPoint reference.");
            }
            for (int i = 0; i < _armJoints.Length; i++)
                if (_armJoints[i] == null)
                    throw new InvalidOperationException(
                        $"[Dg5fGraspLiftAgent] Missing arm joint: {Dg5fGraspLiftSpec.ArmLinks[i]}");
            for (int i = 0; i < _handJoints.Length; i++)
                if (_handJoints[i] == null)
                    throw new InvalidOperationException(
                        $"[Dg5fGraspLiftAgent] Missing hand joint channel {i}.");
            for (int i = 0; i < Dg5fGraspLiftSpec.FingerCount; i++)
                if (fingerTips[i] == null)
                    throw new InvalidOperationException(
                        $"[Dg5fGraspLiftAgent] Missing fingertip {i}.");
            if (contactSensors == null || contactSensors.Length == 0)
                throw new InvalidOperationException(
                    "[Dg5fGraspLiftAgent] No object contact sensors were resolved.");
            for (int index = 0; index < Dg5fGraspLiftSpec.ContactPointCount; index++)
            {
                bool covered = false;
                foreach (var sensor in contactSensors)
                    if (sensor != null && sensor.contactIndex == index) { covered = true; break; }
                if (!covered)
                    throw new InvalidOperationException(
                        $"[Dg5fGraspLiftAgent] No object contact sensor covers contact point {index}.");
            }
            if (safetySensors == null || safetySensors.Length == 0)
                throw new InvalidOperationException(
                    "[Dg5fGraspLiftAgent] Moving arm-link panel safety sensors are required.");
            // Old generated scenes do not contain these non-terminal sensors yet.
            // Warn instead of throwing so the currently built scene remains usable
            // until the orchestrator regenerates it.
            if (handSurfaceSensors == null || handSurfaceSensors.Length == 0)
                Debug.LogWarning(
                    "[Dg5fGraspLiftAgent] No hand-panel contact sensors were resolved; "
                    + "the scrape penalty and contact-time stat will remain zero.",
                    this);
        }

        // ------------------------------------------------------------------ episode

        public override void OnEpisodeBegin()
        {
            EnsureResolved();
            _episodeActive = false;
            Dg5fGraspLiftSpec.RefreshGraspStage();
            Dg5fGraspLiftSpec.RefreshBlockWidth();
            Dg5fGraspLiftSpec.RefreshBlockHeight();
            Dg5fGraspLiftSpec.RefreshToppleLimit();
            Dg5fGraspLiftSpec.RefreshBlockCenterOfMass();
            Dg5fGraspLiftSpec.RefreshTopDownAlignmentPotentialMax();
            Dg5fGraspLiftSpec.RefreshActionRatePenaltyScale();
            Dg5fGraspLiftSpec.RefreshHandSurfacePenaltyPerSecond();
            Dg5fGraspLiftSpec.RefreshGraspPosturePenaltyScale();

            _closure = 0f;
            _episodeSeconds = 0f;
            _graspSeconds = 0f;
            _liftHoldSeconds = 0f;
            _slipSeconds = 0f;
            _bestLiftHeight = 0f;
            _maxObjectTiltDeg = 0f;
            _handSurfaceContactSeconds = 0f;
            _handSurfaceContactSecondsSinceDecision = 0f;
            _sumSquaredArmActionDeltas = 0f;
            _sumGraspPostureAngleDegrees = 0f;
            _armActionDecisionCount = 0;
            _graspPostureAngleSampleCount = 0;
            Array.Clear(_previousArmActions, 0, _previousArmActions.Length);
            _hasPreviousArmAction = false;
            _graspConfirmed = false;
            _unsafeSurfaceContact = false;
            _contactCount = 0;
            _contactCentroid = Vector3.zero;
            _bestClosurePotential = 0f;
            _bestContactPotential = 0f;
            _bestGraspPotential = 0f;
            _previousLiftPotential = 0f;
            LastTerminationReason = "None";

            ResetRobot();
            ResetObject();
            foreach (var sensor in contactSensors)
                if (sensor != null) sensor.ResetContacts();
            foreach (var sensor in safetySensors)
                if (sensor != null) sensor.ResetContacts();
            foreach (var sensor in handSurfaceSensors)
                if (sensor != null) sensor.ResetContacts();

            _previousApproachPotential = Dg5fGraspLiftSpec.DirectionalApproachPotential(
                GraspDistance(),
                PalmFacingAlignment());
            _bestTopDownPotential = TopDownAlignmentPotential();
            _episodeActive = true;
        }

        float InitialTargetDeg(ArticulationBody body)
        {
            if (_initialTargetDeg.TryGetValue(body, out float cached)) return cached;
            float value = body.xDrive.target;
            _initialTargetDeg[body] = value;
            return value;
        }

        void ResetRobot()
        {
            foreach (var body in _allJoints)
            {
                float targetDeg = InitialTargetDeg(body);
                var drive = body.xDrive;
                drive.target = targetDeg;
                body.xDrive = drive;
                body.jointPosition = new ArticulationReducedSpace(targetDeg * Mathf.Deg2Rad);
                body.jointVelocity = new ArticulationReducedSpace(0f);
            }

            for (int i = 0; i < _armJoints.Length; i++)
            {
                float initial = InitialTargetDeg(_armJoints[i]);
                _armTargetDeg[i] = Mathf.Clamp(
                    initial,
                    Dg5fGraspLiftSpec.ArmSafeMinDeg[i],
                    Dg5fGraspLiftSpec.ArmSafeMaxDeg[i]);
            }
            ApplyArmTargets();
            _closure = 0f;
            ApplyGripTargets();
        }

        void ResetObject()
        {
            if (_objectCollider == null)
                throw new InvalidOperationException(
                    "[Dg5fGraspLiftAgent] The grasp object requires a collider.");

            Vector3 localPosition = Vector3.zero;
            bool sampled = false;
            for (int attempt = 0; attempt < 32 && !sampled; attempt++)
            {
                localPosition = Dg5fGraspLiftSpec.SpawnBlockLocalPosition(
                    Next01(), Next01(), Next01(), Dg5fGraspLiftSpec.CurrentBlockHeight);
                sampled = Dg5fGraspLiftSpec.IsValidSpawn(
                    localPosition,
                    Dg5fGraspLiftSpec.CurrentBlockWidth,
                    Dg5fGraspLiftSpec.CurrentBlockHeight);
            }
            if (!sampled)
                throw new InvalidOperationException(
                    "[Dg5fGraspLiftAgent] Could not sample a valid block spawn pose.");

            if (!graspObject.isKinematic)
            {
                graspObject.linearVelocity = Vector3.zero;
                graspObject.angularVelocity = Vector3.zero;
            }
            graspObject.isKinematic = true;
            graspObject.useGravity = false;
            // Resize while kinematic and still parked away from the hand: rescaling a
            // live Rigidbody that is touching the fingers makes the solver explode.
            ApplyBlockSize();
            graspObject.position = robotBase.TransformPoint(localPosition);
            // Yaw only: the block must always start upright so "lift" is measured
            // against a repeatable pose.
            graspObject.rotation = robotBase.rotation
                * Quaternion.AngleAxis(Next01() * 360f, Vector3.up);
            Physics.SyncTransforms();

            _spawnObjectLocalPosition = localPosition;
            _spawnObjectHeight = graspObject.position.y;
            // Articulation collider transforms lag direct jointPosition writes by one
            // physics step. Keep the block kinematic for that step, then release it.
            _objectReleaseFixedSteps = 2;
        }

        /// Applies the current block-size lesson to the block's scale and mass.
        /// Mass tracks volume so wider or taller blocks are proportionally heavier.
        void ApplyBlockSize()
        {
            float width = Dg5fGraspLiftSpec.CurrentBlockWidth;
            graspObject.transform.localScale =
                new Vector3(width, Dg5fGraspLiftSpec.CurrentBlockHeight, width);
            graspObject.mass = Dg5fGraspLiftSpec.CurrentBlockMass;
            // Rigidbody COM uses the unit cube's unscaled local space.
            graspObject.centerOfMass = Dg5fGraspLiftSpec.CurrentBlockCenterOfMassLocal;
        }

        void ReleaseObject()
        {
            graspObject.isKinematic = false;
            graspObject.useGravity = true;
            graspObject.linearVelocity = Vector3.zero;
            graspObject.angularVelocity = Vector3.zero;
            // Re-latch every baseline after the settle step so the two throwaway
            // physics frames cannot leak shaping reward into the episode.
            _spawnObjectHeight = graspObject.position.y;
            _spawnObjectLocalPosition = robotBase.InverseTransformPoint(graspObject.position);
            _previousApproachPotential = Dg5fGraspLiftSpec.DirectionalApproachPotential(
                GraspDistance(),
                PalmFacingAlignment());
            _bestTopDownPotential = TopDownAlignmentPotential();
            _previousLiftPotential = 0f;
        }

        float Next01()
        {
            if (!useDeterministicSpawns) return UnityEngine.Random.value;
            return (float)_random.NextDouble();
        }

        // ------------------------------------------------------------- observations

        public override void CollectObservations(VectorSensor sensor)
        {
            // ML-Agents can request one final observation while a scene unloads.
            // Preserve the fixed contract instead of indexing disposed state.
            if (graspObject == null || robotBase == null || palm == null || graspPoint == null
                || _armJoints == null || fingerTips == null || contactSensors == null
                || !HasFinitePhysicsState())
            {
                for (int i = 0; i < Dg5fGraspLiftSpec.ObservationSize; i++)
                    sensor.AddObservation(0f);
                return;
            }

            // 0..5: normalized arm joint positions.
            for (int i = 0; i < _armJoints.Length; i++)
            {
                float positionDeg = FirstOrZero(_armJoints[i].jointPosition) * Mathf.Rad2Deg;
                sensor.AddObservation(Dg5fGraspLiftSpec.NormalizeJoint(
                    positionDeg,
                    Dg5fGraspLiftSpec.ArmSafeMinDeg[i],
                    Dg5fGraspLiftSpec.ArmSafeMaxDeg[i]));
            }

            // 6..11: normalized arm joint velocities.
            for (int i = 0; i < _armJoints.Length; i++)
                sensor.AddObservation(
                    Mathf.Clamp(FirstOrZero(_armJoints[i].jointVelocity) / Mathf.PI, -1f, 1f));

            // 12: hand closure, centred on zero.
            sensor.AddObservation(_closure * 2f - 1f);

            // 13..21: block state in robot-base coordinates. The offset is measured to
            // the grasp target, i.e. the point the palm should actually reach.
            AddClampedVector(
                sensor,
                robotBase.InverseTransformDirection(GraspTargetPosition() - graspPoint.position),
                1f);
            AddClampedVector(
                sensor, robotBase.InverseTransformDirection(graspObject.linearVelocity), 2f);
            AddClampedVector(
                sensor, robotBase.InverseTransformDirection(graspObject.angularVelocity), 10f);

            // 22: vertical displacement from the spawn pose (the raw lift signal).
            sensor.AddObservation(
                Mathf.Clamp((graspObject.position.y - _spawnObjectHeight) / 0.2f, -1f, 1f));

            // 23..37: each fingertip relative to the block, in palm coordinates.
            for (int i = 0; i < fingerTips.Length; i++)
                AddClampedVector(
                    sensor,
                    palm.InverseTransformDirection(fingerTips[i].position - graspObject.position),
                    0.2f);

            // 38..42: fingertip contact flags.
            for (int i = 0; i < Dg5fGraspLiftSpec.FingerCount; i++)
                sensor.AddObservation(IsContactActive(i) ? 1f : 0f);

            // 43..48: commanded arm xDrive targets.
            for (int i = 0; i < _armTargetDeg.Length; i++)
                sensor.AddObservation(Dg5fGraspLiftSpec.NormalizeJoint(
                    _armTargetDeg[i],
                    Dg5fGraspLiftSpec.ArmSafeMinDeg[i],
                    Dg5fGraspLiftSpec.ArmSafeMaxDeg[i]));

            // 49..56: grasp/lift task state (these slots carried reach bookkeeping in
            // the transferred checkpoint and are repurposed here).
            sensor.AddObservation(
                IsContactActive(Dg5fGraspLiftSpec.PalmContactIndex) ? 1f : 0f);
            sensor.AddObservation(_contactCount / (float)Dg5fGraspLiftSpec.ContactPointCount);
            sensor.AddObservation(Dg5fGraspLiftSpec.GraspProgress(_graspSeconds));
            sensor.AddObservation(_graspConfirmed ? 1f : 0f);
            sensor.AddObservation(Dg5fGraspLiftSpec.LiftProgress(LiftHeight()));
            sensor.AddObservation(Mathf.Clamp01(
                _liftHoldSeconds / Dg5fGraspLiftSpec.CurrentLiftHoldSeconds));
            sensor.AddObservation(Mathf.Clamp01(
                GraspDistance() / Dg5fGraspLiftSpec.MaximumObjectDistance));
            sensor.AddObservation(Mathf.Clamp01(
                _episodeSeconds / Dg5fGraspLiftSpec.EpisodeTimeoutSeconds));
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

        // ------------------------------------------------------------------ actions

        public override void OnActionReceived(ActionBuffers actions)
        {
            var continuous = actions.ContinuousActions;
            if (continuous.Length != Dg5fGraspLiftSpec.ActionSize)
                throw new InvalidOperationException(
                    $"Expected {Dg5fGraspLiftSpec.ActionSize} continuous actions, got {continuous.Length}.");
            if (!_episodeActive || _objectReleaseFixedSteps > 0) return;

            // Time cost: every decision spent not progressing is slightly negative.
            AddReward(Dg5fGraspLiftSpec.DecisionTimePenalty);
            ScoreApproachProgress();
            float handSurfaceContactSeconds = _handSurfaceContactSecondsSinceDecision;
            _handSurfaceContactSecondsSinceDecision = 0f;
            AddReward(Dg5fGraspLiftSpec.HandSurfaceContactPenalty(
                handSurfaceContactSeconds,
                _graspConfirmed));
            float graspDistance = GraspDistance();
            float graspPostureAngleDegrees =
                Dg5fGraspLiftSpec.TopDownAngleDegrees(TopDownAlignment());
            AddReward(Dg5fGraspLiftSpec.GraspPosturePenalty(
                graspPostureAngleDegrees,
                graspDistance,
                _graspConfirmed));
            // Sample the same pre-confirmation pose constrained by the direct cost.
            // Excluding the lift prevents post-grasp wrist motion from contaminating
            // the diagnostic that the posture sweep is intended to read.
            if (!_graspConfirmed
                && Dg5fGraspLiftSpec.IsFinite(graspDistance)
                && Dg5fGraspLiftSpec.IsFinite(graspPostureAngleDegrees)
                && graspDistance <= Dg5fGraspLiftSpec.GraspReadyDistance)
            {
                _sumGraspPostureAngleDegrees += graspPostureAngleDegrees;
                _graspPostureAngleSampleCount++;
            }

            // Arm: integrate bounded joint deltas, damped near the block so the hand
            // does not punt it across the panel.
            bool nearObject = Dg5fGraspLiftSpec.UsesNearObjectControl(graspDistance);
            float actionScale = nearObject ? Dg5fGraspLiftSpec.NearObjectArmDeltaScale : 1f;
            float sumSquaredArmActions = 0f;
            float sumSquaredArmActionDeltas = 0f;
            for (int i = 0; i < _armTargetDeg.Length; i++)
            {
                float action = Mathf.Clamp(continuous[i], -1f, 1f);
                sumSquaredArmActions += action * action;
                if (_hasPreviousArmAction)
                {
                    float actionDelta = action - _previousArmActions[i];
                    sumSquaredArmActionDeltas += actionDelta * actionDelta;
                }
                _previousArmActions[i] = action;
                _armTargetDeg[i] = Mathf.Clamp(
                    _armTargetDeg[i] + action * armDeltaDegPerDecision * actionScale,
                    Dg5fGraspLiftSpec.ArmSafeMinDeg[i],
                    Dg5fGraspLiftSpec.ArmSafeMaxDeg[i]);
            }
            if (_hasPreviousArmAction)
            {
                AddReward(Dg5fGraspLiftSpec.ArmActionRatePenalty(
                    sumSquaredArmActionDeltas));
                _sumSquaredArmActionDeltas += sumSquaredArmActionDeltas;
            }
            _hasPreviousArmAction = true;
            _armActionDecisionCount++;
            if (nearObject)
                AddReward(Dg5fGraspLiftSpec.NearObjectActionPenalty(sumSquaredArmActions));
            ApplyArmTargets();

            // Grip: closing pays only within GraspReadyDistance of the block, and only
            // as a new-best potential so the fingers cannot be pumped for reward.
            float delta = Mathf.Clamp(continuous[6], -1f, 1f) * gripDeltaPerDecision;
            float newClosure = Mathf.Clamp01(_closure + delta);
            bool readyToGrasp = IsReadyToGrasp();
            AddReward(Dg5fGraspLiftSpec.ClosureFarPenalty(newClosure - _closure, readyToGrasp));
            float closurePotential =
                Dg5fGraspLiftSpec.ClosurePotential(newClosure, readyToGrasp);
            AddReward(Dg5fGraspLiftSpec.NewBestPotentialDelta(
                _bestClosurePotential, closurePotential));
            _bestClosurePotential = Mathf.Max(_bestClosurePotential, closurePotential);
            _closure = newClosure;
            ApplyGripTargets();

            // A closed but empty hand climbing away from the table is the classic
            // "lift without grasping" degenerate policy.
            if (Dg5fGraspLiftSpec.IsClosedHandAscent(
                    GraspPointHeightAbovePanel(), _closure, _graspConfirmed))
            {
                AddReward(Dg5fGraspLiftSpec.ClosedHandAscentPenalty);
            }
        }

        bool IsReadyToGrasp()
        {
            return GraspDistance() <= Dg5fGraspLiftSpec.GraspReadyDistance;
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
                float target = Mathf.Lerp(
                    _openHandDeg[i],
                    Dg5fGraspLiftSpec.LeftFistDeg[i],
                    Mathf.Clamp01(_closure));
                drive.target = Mathf.Clamp(target, drive.lowerLimit, drive.upperLimit);
                _handJoints[i].xDrive = drive;
            }
        }

        // ------------------------------------------------------------------ physics

        void FixedUpdate()
        {
            if (!_resolved)
            {
                // A ForcedFullReset can invoke OnEpisodeBegin before the robot
                // hierarchy is fully built. Retry every tick; once resolved, start the
                // episode OnEpisodeBegin could not complete earlier.
                EnsureResolved();
                if (!_resolved) return;
                OnEpisodeBegin();
                return;
            }
            if (!_episodeActive || graspObject == null || robotBase == null) return;

            if (_objectReleaseFixedSteps > 0)
            {
                _objectReleaseFixedSteps--;
                if (_objectReleaseFixedSteps == 0) ReleaseObject();
                return;
            }

            if (!HasFinitePhysicsState())
            {
                FinishEpisode(false, "NonFinitePhysics");
                return;
            }

            if (_unsafeSurfaceContact || HasUnsafeSurfaceContact())
            {
                FinishEpisode(false, "UnsafeSurfaceContact");
                return;
            }

            Vector3 objectLocalPosition = robotBase.InverseTransformPoint(graspObject.position);
            if (Dg5fGraspLiftSpec.IsOutOfBounds(
                    objectLocalPosition,
                    Dg5fGraspLiftSpec.SupportTopHeight,
                    Dg5fGraspLiftSpec.CurrentBlockHalfHeight))
            {
                FinishEpisode(false, "ObjectOutOfBounds");
                return;
            }

            if (Dg5fGraspLiftSpec.IsPushedAway(
                    objectLocalPosition, _spawnObjectLocalPosition, _graspConfirmed))
            {
                FinishEpisode(false, "ObjectPushedAway");
                return;
            }

            float objectTiltDeg = ObjectTiltDegrees();
            _maxObjectTiltDeg = Mathf.Max(_maxObjectTiltDeg, objectTiltDeg);
            // A toppled block is unliftable and used to be a profitable dead end.
            if (Dg5fGraspLiftSpec.IsToppled(objectTiltDeg, _graspConfirmed))
            {
                FinishEpisode(false, "ObjectToppled");
                return;
            }

            _episodeSeconds += Time.fixedDeltaTime;
            if (HasHandSurfaceContact())
            {
                _handSurfaceContactSeconds += Time.fixedDeltaTime;
                _handSurfaceContactSecondsSinceDecision += Time.fixedDeltaTime;
            }
            UpdateContacts();

            if (!_graspConfirmed)
                UpdateGraspProgress();
            else if (UpdateLiftProgress())
                return;

            if (Dg5fGraspLiftSpec.ReachedEpisodeTimeout(_episodeSeconds))
                FinishEpisode(false, "Timeout");
        }

        /// Recomputes contact count, the contact centroid, and the unit directions
        /// from the block centre toward every active contact (used by the
        /// force-closure proxy).
        /// True when any sensor tagged with this contact index is touching the block.
        bool IsContactActive(int contactIndex)
        {
            if (contactSensors == null) return false;
            foreach (var sensor in contactSensors)
                if (sensor != null && sensor.contactIndex == contactIndex && sensor.IsTouching)
                    return true;
            return false;
        }

        void UpdateContacts()
        {
            _contactCount = 0;
            Vector3 centroidSum = Vector3.zero;
            Vector3 objectCenter = graspObject.position;

            for (int i = 0; i < Dg5fGraspLiftSpec.ContactPointCount; i++)
            {
                if (!IsContactActive(i)) continue;
                Transform contactTransform =
                    i == Dg5fGraspLiftSpec.PalmContactIndex ? palm : fingerTips[i];
                if (contactTransform == null) continue;

                Vector3 position = contactTransform.position;
                centroidSum += position;
                _contactDirections[_contactCount] = position - objectCenter;
                _contactCount++;
            }

            _contactCentroid = _contactCount > 0
                ? centroidSum / _contactCount
                : objectCenter;
        }

        void UpdateGraspProgress()
        {
            bool candidate = Dg5fGraspLiftSpec.IsGraspCandidate(
                _contactCount,
                _contactDirections,
                graspObject.position,
                _contactCentroid,
                _closure);

            if (!candidate)
            {
                // A grasp has to be continuous. Any frame that breaks the contract
                // restarts the dwell so contact flicker cannot accumulate into a
                // confirmed grasp.
                _graspSeconds = 0f;
                return;
            }

            // Dense credit for the number of fingers on the block. New-best only, so
            // repeatedly tapping the block cannot farm reward.
            float contactPotential = Dg5fGraspLiftSpec.ContactPotential(_contactCount);
            AddReward(Dg5fGraspLiftSpec.NewBestPotentialDelta(
                _bestContactPotential, contactPotential));
            _bestContactPotential = Mathf.Max(_bestContactPotential, contactPotential);

            _graspSeconds += Time.fixedDeltaTime;

            // Partial credit for holding a valid grasp, again new-best so the total
            // paid for reaching a confirmed grasp is exactly GraspConfirmReward.
            float graspPotential = Dg5fGraspLiftSpec.GraspConfirmReward
                * Dg5fGraspLiftSpec.GraspProgress(_graspSeconds);
            AddReward(Dg5fGraspLiftSpec.NewBestPotentialDelta(
                _bestGraspPotential, graspPotential));
            _bestGraspPotential = Mathf.Max(_bestGraspPotential, graspPotential);

            if (Dg5fGraspLiftSpec.IsGraspConfirmed(_graspSeconds))
            {
                _graspConfirmed = true;
                _slipSeconds = 0f;
                _liftHoldSeconds = 0f;
                _previousLiftPotential = Dg5fGraspLiftSpec.LiftPotential(LiftHeight());
            }
        }

        /// Returns true when the episode ended inside this call.
        bool UpdateLiftProgress()
        {
            float liftHeight = LiftHeight();
            _bestLiftHeight = Mathf.Max(_bestLiftHeight, liftHeight);

            // Plain (not new-best) potential: the block has to stay up. Letting it
            // sink hands the shaping back, which is the gradient a drop deserves.
            float liftPotential = Dg5fGraspLiftSpec.LiftPotential(liftHeight);
            AddReward(Dg5fGraspLiftSpec.PotentialDelta(_previousLiftPotential, liftPotential));
            _previousLiftPotential = liftPotential;

            bool stillGrasped = Dg5fGraspLiftSpec.IsGraspCandidate(
                _contactCount,
                _contactDirections,
                graspObject.position,
                _contactCentroid,
                _closure);
            _slipSeconds = stillGrasped ? 0f : _slipSeconds + Time.fixedDeltaTime;

            // Dropped: contact lost past the grace window AND the block has fallen
            // back toward the table. Losing contact while the block is still up is a
            // re-grip, not a drop.
            if (_slipSeconds > Dg5fGraspLiftSpec.SlipGraceSeconds
                && liftHeight < _bestLiftHeight * 0.3f)
            {
                FinishEpisode(false, "Dropped");
                return true;
            }

            float objectSpeed = graspObject.linearVelocity.magnitude;
            _liftHoldSeconds = Dg5fGraspLiftSpec.IsStableLift(liftHeight, objectSpeed)
                ? _liftHoldSeconds + Time.fixedDeltaTime
                : 0f;

            if (Dg5fGraspLiftSpec.IsLiftComplete(_liftHoldSeconds))
            {
                FinishEpisode(true, "Success");
                return true;
            }
            return false;
        }

        void ScoreApproachProgress()
        {
            // Once the block is grasped the hand is supposed to carry it away from
            // where it started, so the approach terms are frozen at that point.
            if (_graspConfirmed) return;

            float currentApproach = Dg5fGraspLiftSpec.DirectionalApproachPotential(
                GraspDistance(),
                PalmFacingAlignment());
            AddReward(Dg5fGraspLiftSpec.PotentialDelta(
                _previousApproachPotential, currentApproach));
            _previousApproachPotential = currentApproach;

            float currentTopDown = TopDownAlignmentPotential();
            AddReward(Dg5fGraspLiftSpec.NewBestPotentialDelta(
                _bestTopDownPotential, currentTopDown));
            _bestTopDownPotential = Mathf.Max(_bestTopDownPotential, currentTopDown);
        }

        bool HasFinitePhysicsState()
        {
            if (!Dg5fGraspLiftSpec.IsFinite(graspObject.position)
                || !Dg5fGraspLiftSpec.IsFinite(graspObject.linearVelocity)
                || !Dg5fGraspLiftSpec.IsFinite(graspObject.angularVelocity))
            {
                return false;
            }

            Quaternion rotation = graspObject.rotation;
            if (!Dg5fGraspLiftSpec.IsFinite(rotation.x)
                || !Dg5fGraspLiftSpec.IsFinite(rotation.y)
                || !Dg5fGraspLiftSpec.IsFinite(rotation.z)
                || !Dg5fGraspLiftSpec.IsFinite(rotation.w))
            {
                return false;
            }

            foreach (var joint in _allJoints)
            {
                if (!Dg5fGraspLiftSpec.IsFinite(FirstOrZero(joint.jointPosition))
                    || !Dg5fGraspLiftSpec.IsFinite(FirstOrZero(joint.jointVelocity))
                    || !Dg5fGraspLiftSpec.IsFinite(joint.xDrive.target))
                {
                    return false;
                }
            }
            return true;
        }

        bool HasUnsafeSurfaceContact()
        {
            foreach (var sensor in safetySensors)
                if (sensor != null && sensor.HasUnsafeContact) return true;
            return false;
        }

        bool HasHandSurfaceContact()
        {
            if (handSurfaceSensors == null) return false;
            foreach (var sensor in handSurfaceSensors)
                if (sensor != null && sensor.IsTouching) return true;
            return false;
        }

        public void NotifyUnsafeSurfaceContact(Collider surface)
        {
            if (surface == pedestalCollider) _unsafeSurfaceContact = true;
        }

        void FinishEpisode(bool success, string reason)
        {
            if (!_episodeActive) return;
            _episodeActive = false;
            ScoreApproachProgress();

            if (success)
                AddReward(Dg5fGraspLiftSpec.LiftSuccessReward);
            else
                AddReward(Dg5fGraspLiftSpec.FailurePenalty(reason));

            RecordOutcome(success, reason);
            EndEpisode();
        }

        void RecordOutcome(bool success, string reason)
        {
            LastTerminationReason = reason;
            if (_stats == null) return;

            _stats.Add("GraspLift/Success", success ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("GraspLift/GraspConfirmed", _graspConfirmed ? 1f : 0f,
                StatAggregationMethod.Average);
            _stats.Add("GraspLift/GraspSeconds", _graspSeconds, StatAggregationMethod.Average);
            _stats.Add("GraspLift/ContactCount", _contactCount, StatAggregationMethod.Average);
            _stats.Add("GraspLift/FinalDistanceMeters", GraspDistance(),
                StatAggregationMethod.Average);
            _stats.Add("GraspLift/BestLiftHeight", _bestLiftHeight, StatAggregationMethod.Average);
            _stats.Add("GraspLift/FinalLiftHeight", LiftHeight(), StatAggregationMethod.Average);
            _stats.Add("GraspLift/ObjectTiltDegrees", ObjectTiltDegrees(),
                StatAggregationMethod.Average);
            _stats.Add("GraspLift/MaxObjectTiltDegrees", _maxObjectTiltDeg,
                StatAggregationMethod.Average);
            _stats.Add("GraspLift/LiftHoldSeconds", _liftHoldSeconds,
                StatAggregationMethod.Average);
            _stats.Add("GraspLift/CompletionSeconds", _episodeSeconds,
                StatAggregationMethod.Average);
            _stats.Add("GraspLift/FinalClosure", _closure, StatAggregationMethod.Average);
            _stats.Add("GraspLift/HandSurfaceContactSeconds", _handSurfaceContactSeconds,
                StatAggregationMethod.Average);
            _stats.Add(
                "GraspLift/TopDownAngleDegrees",
                Dg5fGraspLiftSpec.TopDownAngleDegrees(TopDownAlignment()),
                StatAggregationMethod.Average);
            _stats.Add(
                "GraspLift/GraspPostureAngleDegrees",
                _graspPostureAngleSampleCount > 0
                    ? _sumGraspPostureAngleDegrees / _graspPostureAngleSampleCount
                    : 0f,
                StatAggregationMethod.Average);
            _stats.Add(
                "GraspLift/MeanArmActionRate",
                _armActionDecisionCount > 0
                    ? _sumSquaredArmActionDeltas / _armActionDecisionCount
                    : 0f,
                StatAggregationMethod.Average);
            _stats.Add("Curriculum/GraspStage", Dg5fGraspLiftSpec.CurrentGraspStage,
                StatAggregationMethod.Average);
            _stats.Add("Curriculum/BlockWidth", Dg5fGraspLiftSpec.CurrentBlockWidth,
                StatAggregationMethod.Average);
            _stats.Add("Curriculum/BlockHeight", Dg5fGraspLiftSpec.CurrentBlockHeight,
                StatAggregationMethod.Average);
            if (!success)
                _stats.Add($"Failure/{reason}", 1f, StatAggregationMethod.Sum);
        }

        // ------------------------------------------------------------------ geometry

        /// Where the palm grasp volume should end up: on the block axis, 2.0 cm below
        /// its top face rather than at its geometric centre (see the spec constant).
        /// Offset along the robot vertical, not the block's own axis, so a tipped
        /// block cannot swing the target around.
        Vector3 GraspTargetPosition()
        {
            if (graspObject == null) return Vector3.zero;
            Vector3 up = robotBase != null ? robotBase.up : Vector3.up;
            return graspObject.position
                + up * Dg5fGraspLiftSpec.CurrentGraspTargetHeightOffset;
        }

        float GraspDistance()
        {
            if (graspPoint == null || graspObject == null) return float.PositiveInfinity;
            return Vector3.Distance(graspPoint.position, GraspTargetPosition());
        }

        float LiftHeight()
        {
            if (graspObject == null) return 0f;
            return Dg5fGraspLiftSpec.LiftHeight(graspObject.position.y, _spawnObjectHeight);
        }

        float ObjectTiltDegrees()
        {
            if (graspObject == null || robotBase == null) return 0f;
            return Dg5fGraspLiftSpec.ObjectTiltDegrees(
                graspObject.transform.up,
                robotBase.up);
        }

        float GraspPointHeightAbovePanel()
        {
            if (graspPoint == null || pedestalCollider == null) return 0f;
            return graspPoint.position.y - pedestalCollider.bounds.max.y;
        }

        float PalmFacingAlignment()
        {
            if (graspPoint == null || palm == null || graspObject == null) return -1f;
            return Dg5fGraspLiftSpec.PalmFacingAlignment(
                graspPoint.forward,
                GraspTargetPosition() - palm.position);
        }

        float TopDownAlignment()
        {
            if (graspPoint == null || robotBase == null) return -1f;
            return Dg5fGraspLiftSpec.TopDownAlignment(graspPoint.forward, robotBase.up);
        }

        float TopDownAlignmentPotential()
        {
            if (graspPoint == null || graspObject == null || robotBase == null) return 0f;
            float heightAboveObject = Vector3.Dot(
                graspPoint.position - GraspTargetPosition(),
                robotBase.up);
            return Dg5fGraspLiftSpec.TopDownAlignmentPotential(
                GraspDistance(),
                heightAboveObject,
                TopDownAlignment());
        }

        // ---------------------------------------------------------------- heuristic

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
