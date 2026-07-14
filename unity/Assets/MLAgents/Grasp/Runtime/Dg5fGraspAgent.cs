using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// PPO Agent for reaching, grasping, and lifting a ball with UR5e + DG5F.
    /// This component is the sole xDrive writer in the training scene.
    /// </summary>
    public sealed class Dg5fGraspAgent : Agent
    {
        [Header("Scene references")]
        public Rigidbody ball;
        public Transform spawnCenter;
        public Transform robotBase;
        public Transform palm;
        public Transform graspPoint;
        public Transform[] fingerTips = new Transform[Dg5fGraspSpec.FingerCount];
        public GraspContactSensor[] contactSensors = new GraspContactSensor[Dg5fGraspSpec.FingerCount];

        [Header("Episode")]
        public float tableTopY = 0.43f;
        public float requiredLiftMeters = 0.05f;
        public float requiredLiftHoldSeconds = 1f;
        public float maximumGraspDistance = 0.10f;
        public float workspaceRadius = 0.35f;
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
        float _previousGraspDistance;
        float _previousMeanTipDistance;
        float _previousLift;
        float _successHoldSeconds;
        float _contactHoldSeconds;
        float _decisionDeltaTime = 0.1f;
        bool _contactBonusGranted;
        int _lesson;
        System.Random _random;
        StatsRecorder _stats;

        public float CurrentClosure => _closure;
        public int CurrentLesson => _lesson;

        public override void Initialize()
        {
            ResolveReferences();
            ResolveJoints();
            ValidateConfiguration();

            _armTargetDeg = new float[Dg5fGraspSpec.ArmJointCount];
            foreach (var body in _allJoints)
                _initialTargetDeg[body] = body.xDrive.target;

            var requester = GetComponent<DecisionRequester>();
            if (requester != null)
                _decisionDeltaTime = requester.DecisionPeriod * Time.fixedDeltaTime;
            // MaxStep counts 50 Hz Academy steps, not 10 Hz decisions.
            if (MaxStep <= 0) MaxStep = 750;

            _random = new System.Random(spawnSeed);
            _stats = Academy.Instance.StatsRecorder;
        }

        void ResolveReferences()
        {
            if (robotBase == null) robotBase = transform;
            if (spawnCenter == null)
            {
                var found = GameObject.Find("BallSpawnCenter");
                if (found != null) spawnCenter = found.transform;
            }

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
            if (ball == null || spawnCenter == null || palm == null || graspPoint == null)
                throw new InvalidOperationException("[Dg5fGraspAgent] Missing ball/spawn/palm/graspPoint reference.");
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
            _lesson = Mathf.Clamp(
                Mathf.RoundToInt(Academy.Instance.EnvironmentParameters.GetWithDefault("lesson", 2f)),
                0,
                2);

            // Reset closure before writing hand drives; otherwise a new episode can
            // briefly inherit the previous episode's closed-hand target.
            _closure = 0f;
            ResetRobot();
            ResetBall();
            foreach (var sensor in contactSensors) sensor.ResetContacts();

            _successHoldSeconds = 0f;
            _contactHoldSeconds = 0f;
            _contactBonusGranted = false;
            _previousGraspDistance = GraspDistance();
            _previousMeanTipDistance = MeanTipDistance();
            _previousLift = 0f;
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
            if (!ball.isKinematic)
            {
                ball.linearVelocity = Vector3.zero;
                ball.angularVelocity = Vector3.zero;
            }
            ball.isKinematic = true;
            float radius = _lesson == 0 ? 0.02f : _lesson == 1 ? 0.04f : 0.06f;
            float x = NextRange(-radius, radius);
            float z = NextRange(-radius, radius);
            ball.position = spawnCenter.position + new Vector3(x, 0f, z);
            ball.rotation = Quaternion.Euler(0f, NextRange(0f, 360f), 0f);
            ball.isKinematic = _lesson == 0;
            ball.useGravity = _lesson != 0;
            if (!ball.isKinematic)
            {
                ball.linearVelocity = Vector3.zero;
                ball.angularVelocity = Vector3.zero;
            }
            _initialBallHeight = ball.position.y;
            Physics.SyncTransforms();
        }

        float NextRange(float min, float max)
        {
            if (!useDeterministicSpawns) return UnityEngine.Random.Range(min, max);
            return min + (float)_random.NextDouble() * (max - min);
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

            // 0..11: normalized arm position and velocity.
            for (int i = 0; i < _armJoints.Length; i++)
            {
                var drive = _armJoints[i].xDrive;
                float positionDeg = FirstOrZero(_armJoints[i].jointPosition) * Mathf.Rad2Deg;
                sensor.AddObservation(Dg5fGraspSpec.NormalizeJoint(positionDeg, drive.lowerLimit, drive.upperLimit));
            }
            for (int i = 0; i < _armJoints.Length; i++)
                sensor.AddObservation(Mathf.Clamp(FirstOrZero(_armJoints[i].jointVelocity) / Mathf.PI, -1f, 1f));

            // 12: normalized grip closure.
            sensor.AddObservation(_closure * 2f - 1f);

            // 13..21: ball state in robot-base coordinates.
            AddClampedVector(sensor, robotBase.InverseTransformDirection(ball.position - graspPoint.position), 1f);
            AddClampedVector(sensor, robotBase.InverseTransformDirection(ball.linearVelocity), 2f);
            AddClampedVector(sensor, robotBase.InverseTransformDirection(ball.angularVelocity), 10f);

            // 22: lift from episode start.
            sensor.AddObservation(Mathf.Clamp((ball.position.y - _initialBallHeight) / 0.2f, -1f, 1f));

            // 23..37: each fingertip relative to the ball in palm coordinates.
            for (int i = 0; i < fingerTips.Length; i++)
                AddClampedVector(sensor, palm.InverseTransformDirection(fingerTips[i].position - ball.position), 0.2f);

            // 38..42: thumb/index/middle/ring/pinky contacts.
            for (int i = 0; i < contactSensors.Length; i++)
                sensor.AddObservation(contactSensors[i].IsTouching ? 1f : 0f);
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
            ScoreAndCheckEpisode();
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

        void ScoreAndCheckEpisode()
        {
            AddReward(-0.001f);

            float graspDistance = GraspDistance();
            float reachProgress = Mathf.Clamp(
                (_previousGraspDistance - graspDistance) / 0.02f,
                -1f,
                1f);
            AddReward(0.15f * reachProgress);

            float meanTipDistance = MeanTipDistance();
            if (graspDistance <= 0.12f)
            {
                float enclosureProgress = Mathf.Clamp(
                    (_previousMeanTipDistance - meanTipDistance) / 0.02f,
                    -1f,
                    1f);
                AddReward(0.10f * enclosureProgress);
            }

            bool thumbContact = contactSensors[0].IsTouching;
            bool opposingContact = false;
            for (int i = 1; i < contactSensors.Length; i++)
                opposingContact |= contactSensors[i].IsTouching;
            bool graspContact = thumbContact && opposingContact;

            if (graspContact && !_contactBonusGranted)
            {
                AddReward(0.15f);
                _contactBonusGranted = true;
            }

            float lift = ball.position.y - _initialBallHeight;
            float liftProgress = Mathf.Clamp((lift - _previousLift) / requiredLiftMeters, -1f, 1f);
            AddReward(0.25f * liftProgress);

            _previousGraspDistance = graspDistance;
            _previousMeanTipDistance = meanTipDistance;
            _previousLift = lift;

            bool success;
            if (_lesson == 0)
            {
                _successHoldSeconds = graspDistance <= 0.05f
                    ? _successHoldSeconds + _decisionDeltaTime
                    : 0f;
                success = _successHoldSeconds >= 0.25f;
            }
            else if (_lesson == 1)
            {
                _contactHoldSeconds = graspContact
                    ? _contactHoldSeconds + _decisionDeltaTime
                    : 0f;
                success = _contactHoldSeconds >= 0.5f;
            }
            else
            {
                bool finalCondition = Dg5fGraspSpec.FinalSuccess(
                    lift,
                    requiredLiftMeters,
                    thumbContact,
                    opposingContact,
                    graspDistance,
                    maximumGraspDistance);
                _successHoldSeconds = finalCondition
                    ? _successHoldSeconds + _decisionDeltaTime
                    : 0f;
                success = _successHoldSeconds >= requiredLiftHoldSeconds;
            }

            if (success)
            {
                AddReward(1f);
                _stats.Add("Grasp/Success", 1f, StatAggregationMethod.Average);
                // StepCount is Academy physics steps, not policy decisions.
                _stats.Add("Grasp/CompletionSeconds", StepCount * Time.fixedDeltaTime, StatAggregationMethod.Average);
                EndEpisode();
                return;
            }

            if (HasFailed())
            {
                AddReward(-0.5f);
                _stats.Add("Grasp/Success", 0f, StatAggregationMethod.Average);
                EndEpisode();
            }
        }

        float GraspDistance()
        {
            return Vector3.Distance(graspPoint.position, ball.position);
        }

        float MeanTipDistance()
        {
            float sum = 0f;
            foreach (var tip in fingerTips) sum += Vector3.Distance(tip.position, ball.position);
            return sum / fingerTips.Length;
        }

        bool HasFailed()
        {
            Vector3 p = ball.position;
            if (!IsFinite(p)) return true;
            if (_lesson > 0 && p.y < tableTopY - 0.03f) return true;
            Vector2 horizontal = new Vector2(p.x - spawnCenter.position.x, p.z - spawnCenter.position.z);
            return horizontal.magnitude > workspaceRadius;
        }

        static bool IsFinite(Vector3 value)
        {
            return !(float.IsNaN(value.x) || float.IsInfinity(value.x)
                || float.IsNaN(value.y) || float.IsInfinity(value.y)
                || float.IsNaN(value.z) || float.IsInfinity(value.z));
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
