using System;
using UnityEngine;

namespace KDT.ReachTraining
{
    public enum ReachPhase
    {
        Transit = 0,
        Descend = 1,
        Locked = 2
    }

    /// <summary>
    /// Policy and safety contract for moving an open DG5F hand to a grasp-ready
    /// pose. The policy controls only the UR5e arm; the hand remains at the
    /// prefab's calibrated open pose throughout training.
    /// </summary>
    public static class Dg5fReachSpec
    {
        public const string SpecVersion = "2.0.0";
        public const string BehaviorName = "DG5FGraspReadyReach";

        public const int ArmJointCount = 6;
        public const int HandJointCount = 20;
        public const int ObservationSize = 37;
        public const int ActionSize = 6;
        public const int DecisionPeriod = 5;

        public const int ArmPositionObservationOffset = 0;
        public const int ArmVelocityObservationOffset = 6;
        public const int ArmTargetObservationOffset = 12;
        public const int TargetOffsetObservationOffset = 18;
        public const int ActiveWaypointOffsetObservationOffset = 21;
        public const int DistanceObservationIndex = 24;
        public const int PlanarDistanceObservationIndex = 25;
        public const int FloorClearanceObservationIndex = 26;
        public const int GraspPointVelocityObservationOffset = 27;
        public const int PalmTargetObservationOffset = 30;
        public const int PalmAlignmentObservationIndex = 33;
        public const int UpperConeObservationIndex = 34;
        public const int PhaseObservationIndex = 35;
        public const int HoldProgressObservationIndex = 36;

        public const float MaximumArmDeltaDegPerDecision = 4f;
        public const float TrainingArmDeltaDegPerDecision = 2f;
        public const float PrecisionArmDeltaDegPerDecision = 1f;
        public const float PrecisionDeltaDistance = 0.10f;

        public const float MinimumTargetRadius = 0.20f;
        public const float MaximumTargetRadius = 0.85f;
        public const float TargetRadius = 0.02f;
        public const float MinimumInitialCenterDistance = 0.10f;
        public const int MaximumSpawnAttempts = 256;
        public const float PanelWidth = 1.80f;
        public const float PanelDepth = 1.80f;
        public const float PanelThickness = 0.25f;

        public const float PreGraspHeight = 0.10f;
        public const float TransitWaypointTolerance = 0.03f;
        public const float MinimumTransitClearance = 0.10f;
        public const float MaximumDescendPlanarDistance = 0.05f;
        public const float SuccessDistance = 0.01f;
        public const float MaximumSuccessPointSpeed = 0.05f;
        public const float RequiredSuccessHoldSeconds = 0.25f;
        public const float MinimumPalmAlignment = 0.965925826f; // cos(15 deg)
        public const float MinimumUpperConeAlignment = 0.707106781f; // cos(45 deg)
        public const float EpisodeTimeoutSeconds = 20f;

        public const float WorkspaceRadius = 1.05f;
        public const float WorkspaceMinimumY = -0.05f;
        public const float WorkspaceMaximumY = 1.05f;

        public const float ProgressRewardScale = 2f;
        public const float AlignmentRewardScale = 0.25f;
        public const float DescendPhaseReward = 0.25f;
        public const float LockSuccessReward = 4f;
        public const float DecisionTimePenalty = -0.001f;
        public const float TimeoutPenalty = -1f;
        public const float SafetyPenalty = -2f;

        public static readonly Vector3 CalibratedGraspPointLocalPosition =
            new Vector3(0.0170203224f, 0.152462155f, 0.0135399457f);

        public static readonly string[] ArmLinks =
        {
            "shoulder_link", "upper_arm_link", "forearm_link",
            "wrist_1_link", "wrist_2_link", "wrist_3_link"
        };

        public static readonly float[] ArmSafeMinDeg =
        {
            -180f, -120f, 20f, -180f, -150f, -180f
        };

        public static readonly float[] ArmSafeMaxDeg =
        {
            180f, -20f, 140f, 0f, -30f, 180f
        };

        public static Vector3 SpawnTargetLocalPosition(
            System.Random random,
            Vector3 initialGraspPointLocal,
            float targetCenterY = TargetRadius,
            Predicate<Vector3> acceptsCandidate = null)
        {
            if (random == null) throw new ArgumentNullException(nameof(random));
            if (!IsFinite(initialGraspPointLocal) || !IsFinite(targetCenterY))
                throw new ArgumentException("Spawn inputs must be finite.");

            for (int attempt = 0; attempt < MaximumSpawnAttempts; attempt++)
            {
                float radius = Mathf.Lerp(
                    MinimumTargetRadius,
                    MaximumTargetRadius,
                    (float)random.NextDouble());
                float azimuth = 2f * Mathf.PI * (float)random.NextDouble();
                var candidate = new Vector3(
                    radius * Mathf.Cos(azimuth),
                    targetCenterY,
                    radius * Mathf.Sin(azimuth));
                if (Vector3.Distance(candidate, initialGraspPointLocal)
                        >= MinimumInitialCenterDistance
                    && (acceptsCandidate == null || acceptsCandidate(candidate)))
                {
                    return candidate;
                }
            }

            throw new InvalidOperationException(
                $"Could not sample a valid target in {MaximumSpawnAttempts} attempts.");
        }

        public static bool IsValidSpawn(
            Vector3 targetLocalPosition,
            Vector3 initialGraspPointLocal,
            float targetCenterY = TargetRadius)
        {
            if (!IsFinite(targetLocalPosition) || !IsFinite(initialGraspPointLocal)
                || !IsFinite(targetCenterY)) return false;

            float planarRadius = new Vector2(
                targetLocalPosition.x,
                targetLocalPosition.z).magnitude;
            return planarRadius >= MinimumTargetRadius - 1e-5f
                && planarRadius <= MaximumTargetRadius + 1e-5f
                && Mathf.Abs(targetLocalPosition.y - targetCenterY) <= 1e-5f
                && Vector3.Distance(targetLocalPosition, initialGraspPointLocal)
                    >= MinimumInitialCenterDistance - 1e-5f;
        }

        public static bool IsTargetSphereWithinPanel(
            Vector3 worldCenter,
            float worldRadius,
            Collider panelCollider)
        {
            if (panelCollider == null || !panelCollider.enabled
                || !panelCollider.gameObject.activeInHierarchy
                || !IsFinite(worldCenter) || !IsFinite(worldRadius)
                || worldRadius < 0f) return false;

            if (panelCollider is BoxCollider box)
            {
                Vector3 local = box.transform.InverseTransformPoint(worldCenter);
                Vector3 scale = box.transform.lossyScale;
                float radiusX = worldRadius / Mathf.Max(Mathf.Abs(scale.x), 1e-6f);
                float radiusZ = worldRadius / Mathf.Max(Mathf.Abs(scale.z), 1e-6f);
                Vector3 half = box.size * 0.5f;
                return local.x - radiusX >= box.center.x - half.x
                    && local.x + radiusX <= box.center.x + half.x
                    && local.z - radiusZ >= box.center.z - half.z
                    && local.z + radiusZ <= box.center.z + half.z;
            }

            Bounds bounds = panelCollider.bounds;
            return worldCenter.x - worldRadius >= bounds.min.x
                && worldCenter.x + worldRadius <= bounds.max.x
                && worldCenter.z - worldRadius >= bounds.min.z
                && worldCenter.z + worldRadius <= bounds.max.z;
        }

        public static Vector3 PreGraspPoint(Vector3 targetPosition)
        {
            return targetPosition + Vector3.up * PreGraspHeight;
        }

        public static float PlanarDistance(Vector3 first, Vector3 second)
        {
            if (!IsFinite(first) || !IsFinite(second)) return float.PositiveInfinity;
            return new Vector2(first.x - second.x, first.z - second.z).magnitude;
        }

        public static bool CanEnterDescend(float waypointDistance, float floorClearance)
        {
            return IsFinite(waypointDistance) && IsFinite(floorClearance)
                && waypointDistance <= TransitWaypointTolerance
                && floorClearance >= MinimumTransitClearance;
        }

        public static bool IsPrematureDescent(
            ReachPhase phase,
            float planarDistance,
            float floorClearance)
        {
            if (!IsFinite(planarDistance) || !IsFinite(floorClearance)) return true;
            if (floorClearance >= MinimumTransitClearance) return false;
            return phase == ReachPhase.Transit
                || planarDistance > MaximumDescendPlanarDistance;
        }

        public static float PalmFacingAlignment(Vector3 palmForward, Vector3 palmToTarget)
        {
            if (!IsFinite(palmForward) || !IsFinite(palmToTarget)
                || palmForward.sqrMagnitude <= 1e-12f
                || palmToTarget.sqrMagnitude <= 1e-12f) return -1f;
            return Mathf.Clamp(
                Vector3.Dot(palmForward.normalized, palmToTarget.normalized),
                -1f,
                1f);
        }

        public static float UpperConeAlignment(
            Vector3 palmPosition,
            Vector3 targetPosition)
        {
            Vector3 targetToPalm = palmPosition - targetPosition;
            if (!IsFinite(targetToPalm) || targetToPalm.sqrMagnitude <= 1e-12f)
                return -1f;
            return Mathf.Clamp(
                Vector3.Dot(targetToPalm.normalized, Vector3.up),
                -1f,
                1f);
        }

        public static bool MeetsLockState(
            float distance,
            float pointSpeed,
            float palmAlignment,
            float upperConeAlignment)
        {
            return IsFinite(distance) && IsFinite(pointSpeed)
                && IsFinite(palmAlignment) && IsFinite(upperConeAlignment)
                && distance >= 0f && pointSpeed >= 0f
                && distance <= SuccessDistance
                && pointSpeed <= MaximumSuccessPointSpeed
                && palmAlignment >= MinimumPalmAlignment
                && upperConeAlignment >= MinimumUpperConeAlignment;
        }

        public static float NextLockHoldSeconds(
            float previousHoldSeconds,
            float distance,
            float pointSpeed,
            float palmAlignment,
            float upperConeAlignment,
            float deltaSeconds)
        {
            if (!MeetsLockState(
                    distance,
                    pointSpeed,
                    palmAlignment,
                    upperConeAlignment)
                || !IsFinite(previousHoldSeconds) || !IsFinite(deltaSeconds)
                || deltaSeconds < 0f) return 0f;
            return Mathf.Max(0f, previousHoldSeconds) + deltaSeconds;
        }

        public static float LockHoldProgress(float holdSeconds)
        {
            if (!IsFinite(holdSeconds)) return 0f;
            return Mathf.Clamp01(
                Mathf.Max(0f, holdSeconds) / RequiredSuccessHoldSeconds);
        }

        public static bool HasCompletedLockHold(float holdSeconds)
        {
            return IsFinite(holdSeconds)
                && holdSeconds >= RequiredSuccessHoldSeconds;
        }

        public static float DecisionReward(
            float previousDistance,
            float currentDistance,
            ReachPhase phase,
            float previousAlignment,
            float currentAlignment)
        {
            float reward = DecisionTimePenalty;
            if (IsFinite(previousDistance) && IsFinite(currentDistance))
                reward += ProgressRewardScale * (previousDistance - currentDistance);
            if (phase == ReachPhase.Descend
                && IsFinite(previousAlignment) && IsFinite(currentAlignment))
            {
                reward += AlignmentRewardScale
                    * (currentAlignment - previousAlignment);
            }
            return reward;
        }

        public static float FailurePenalty(string reason)
        {
            return string.Equals(reason, "Timeout", StringComparison.Ordinal)
                ? TimeoutPenalty
                : SafetyPenalty;
        }

        public static float ArmDeltaDeg(float distance)
        {
            return IsFinite(distance) && distance <= PrecisionDeltaDistance
                ? PrecisionArmDeltaDegPerDecision
                : TrainingArmDeltaDegPerDecision;
        }

        public static float AccumulateJointTarget(
            float currentTargetDeg,
            float normalizedAction,
            float maximumDeltaDeg,
            float lowerLimitDeg,
            float upperLimitDeg)
        {
            if (!IsFinite(currentTargetDeg) || !IsFinite(normalizedAction)
                || !IsFinite(maximumDeltaDeg))
                return ClampJointTarget(0f, lowerLimitDeg, upperLimitDeg);
            float allowedDelta = Mathf.Min(
                MaximumArmDeltaDegPerDecision,
                Mathf.Max(0f, maximumDeltaDeg));
            return ClampJointTarget(
                currentTargetDeg
                    + Mathf.Clamp(normalizedAction, -1f, 1f) * allowedDelta,
                lowerLimitDeg,
                upperLimitDeg);
        }

        public static float ClampJointTarget(
            float valueDeg,
            float lowerLimitDeg,
            float upperLimitDeg)
        {
            if (!IsFinite(valueDeg) || !IsFinite(lowerLimitDeg)
                || !IsFinite(upperLimitDeg)) return 0f;
            return Mathf.Clamp(
                valueDeg,
                Mathf.Min(lowerLimitDeg, upperLimitDeg),
                Mathf.Max(lowerLimitDeg, upperLimitDeg));
        }

        public static float NormalizeJoint(float valueDeg, float lowerDeg, float upperDeg)
        {
            if (!IsFinite(valueDeg) || !IsFinite(lowerDeg) || !IsFinite(upperDeg)
                || upperDeg <= lowerDeg) return 0f;
            return Mathf.Clamp(
                (valueDeg - lowerDeg) / (upperDeg - lowerDeg) * 2f - 1f,
                -1f,
                1f);
        }

        public static Vector3 PointVelocity(
            Vector3 centerOfMassVelocity,
            Vector3 angularVelocity,
            Vector3 worldCenterOfMass,
            Vector3 worldPoint)
        {
            if (!IsFinite(centerOfMassVelocity) || !IsFinite(angularVelocity)
                || !IsFinite(worldCenterOfMass) || !IsFinite(worldPoint))
                return Vector3.zero;
            return centerOfMassVelocity
                + Vector3.Cross(angularVelocity, worldPoint - worldCenterOfMass);
        }

        public static bool ReachedEpisodeTimeout(float elapsedSeconds)
        {
            return IsFinite(elapsedSeconds)
                && elapsedSeconds >= EpisodeTimeoutSeconds - 1e-5f;
        }

        public static bool IsWithinWorkspace(Vector3 localPosition)
        {
            return IsFinite(localPosition)
                && localPosition.magnitude <= WorkspaceRadius
                && localPosition.y >= WorkspaceMinimumY
                && localPosition.y <= WorkspaceMaximumY;
        }

        public static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        public static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }
    }
}
