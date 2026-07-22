using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// PPO Agent for v1 reach training with UR5e + DG5F.
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
        [Tooltip("false면 손가락 20관절 xDrive를 쓰지 않음 — 손은 별도 텔레옵(Dg5fHandDriver)이 전담. " +
                 "팔 6관절 제어는 영향받지 않음. 학습 인스턴스는 항상 true(기본값)로 둘 것.")]
        public bool driveHandJoints = true;
        [Tooltip("설정하면 랜덤 스폰 대신 이 리시버의 최신 카메라 좌표로 공을 스폰 (없으면 기존 랜덤 로직 유지). " +
                 "학습 인스턴스는 비워둘 것.")]
        public CameraTargetReceiver cameraReceiver;

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
        float _bestGraspDistance;
        float _episodeSeconds;
        int _ballReleaseFixedSteps;
        bool _episodeActive;
        System.Random _random;
        StatsRecorder _stats;

        public float CurrentClosure => _closure;
        public float CurrentSupportTopHeight => _supportTopHeight;
        public Vector3 CurrentBallLocalPosition => robotBase.InverseTransformPoint(ball.position);
        public float CurrentEpisodeSeconds => _episodeSeconds;
        public float CurrentGraspDistance => GraspDistance();
        public float BestGraspDistance => _bestGraspDistance;
        public float CurrentPalmFacingAlignment => PalmFacingAlignment();

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
        }

        public override void OnEpisodeBegin()
        {
            _episodeActive = false;
            _closure = 0f;
            ResetRobot();
            ResetBall();
            foreach (var sensor in contactSensors) sensor.ResetContacts();

            _episodeSeconds = 0f;
            _bestGraspDistance = GraspDistance();
            _previousApproachPotential = Dg5fGraspSpec.DirectionalApproachPotential(
                _bestGraspDistance,
                PalmFacingAlignment());
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
            Vector3 ballLocalPosition = TryGetCameraSpawnPosition(ballRadius, out Vector3 cameraPosition)
                ? cameraPosition
                : Dg5fGraspSpec.SpawnBallLocalPosition(Next01(), Next01(), ballRadius);
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

        /// cameraReceiver의 최신 좌표를 v1 스폰 규칙(패널 상판 고정 높이, 수평 반경
        /// [V1MinimumSpawnRadius, V1MaximumSpawnRadius] 클램프)에 맞춰 변환한다.
        /// 카메라의 원시 Y값은 깊이 노이즈로 패널 표면과 어긋날 수 있어 신뢰하지 않고,
        /// 항상 SupportTopHeight + ballRadius로 고정한다(랜덤 스폰과 동일한 방식).
        bool TryGetCameraSpawnPosition(float ballRadius, out Vector3 ballLocalPosition)
        {
            ballLocalPosition = Vector3.zero;
            if (cameraReceiver == null
                || !cameraReceiver.TryGetRobotLocalPosition(out Vector3 cameraLocal))
            {
                return false;
            }

            float horizontalRadius = new Vector2(cameraLocal.x, cameraLocal.z).magnitude;
            if (horizontalRadius < 1e-6f) return false;
            horizontalRadius = Mathf.Clamp(
                horizontalRadius,
                Dg5fGraspSpec.V1MinimumSpawnRadius,
                Dg5fGraspSpec.V1MaximumSpawnRadius);
            float azimuth = Mathf.Atan2(cameraLocal.z, cameraLocal.x);
            ballLocalPosition = new Vector3(
                Mathf.Cos(azimuth) * horizontalRadius,
                Dg5fGraspSpec.SupportTopHeight + Mathf.Max(0f, ballRadius),
                Mathf.Sin(azimuth) * horizontalRadius);
            return Dg5fGraspSpec.IsValidSpawn(ballLocalPosition, ballRadius);
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

            // 49..52: v1/v2/v3/v4 objective one-hot. v1 is Reach.
            sensor.AddObservation(1f);
            sensor.AddObservation(0f);
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
            ScoreApproachProgress();

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
            if (!driveHandJoints) return;
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
            float palmFacingAlignment = PalmFacingAlignment();
            _bestGraspDistance = Mathf.Min(_bestGraspDistance, distance);
            if (Dg5fGraspSpec.HasReachedApproachTarget(distance, palmFacingAlignment))
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
            _previousApproachPotential = Dg5fGraspSpec.DirectionalApproachPotential(
                _bestGraspDistance,
                PalmFacingAlignment());
        }

        void ScoreApproachProgress()
        {
            float distance = GraspDistance();
            float currentPotential = Dg5fGraspSpec.DirectionalApproachPotential(
                distance,
                PalmFacingAlignment());
            AddReward(Dg5fGraspSpec.PotentialDelta(_previousApproachPotential, currentPotential));
            _previousApproachPotential = currentPotential;
            if (Dg5fGraspSpec.IsFinite(distance))
                _bestGraspDistance = Mathf.Min(_bestGraspDistance, distance);
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
            ScoreApproachProgress();
            if (success) AddReward(Dg5fGraspSpec.ApproachSuccessReward);

            _stats.Add("Reach/Success", success ? 1f : 0f, StatAggregationMethod.Average);
            _stats.Add("Reach/CompletionSeconds", _episodeSeconds, StatAggregationMethod.Average);
            _stats.Add("Reach/FinalDistanceMeters", GraspDistance(), StatAggregationMethod.Average);
            _stats.Add("Reach/BestDistanceMeters", _bestGraspDistance, StatAggregationMethod.Average);
            _stats.Add("Reach/FinalPalmFacingAlignment", PalmFacingAlignment(), StatAggregationMethod.Average);
            if (!success)
                _stats.Add($"Failure/{failureReason}", 1f, StatAggregationMethod.Average);
            EndEpisode();
        }

        float GraspDistance()
        {
            return Vector3.Distance(graspPoint.position, ball.position);
        }

        float PalmFacingAlignment()
        {
            return Dg5fGraspSpec.PalmFacingAlignment(
                graspPoint.forward,
                ball.position - palm.position);
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
