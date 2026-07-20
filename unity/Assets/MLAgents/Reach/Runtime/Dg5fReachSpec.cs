using System;
using UnityEngine;

namespace KDT.ReachTraining
{
    /// <summary>
    /// Versioned policy, episode, and reward contract for positioning one logical
    /// GraspPoint with the six UR5e arm joints. Hand joints are intentionally absent.
    /// </summary>
    public static class Dg5fReachSpec
    {
        public const string SpecVersion = "1.0.0";
        public const string BehaviorName = "DG5FGraspPointReach";

        public const int ArmJointCount = 6;
        public const int ObservationSize = 26;
        public const int ActionSize = 6;
        public const int DecisionPeriod = 5;

        public const int ArmPositionObservationOffset = 0;
        public const int ArmVelocityObservationOffset = 6;
        public const int ArmTargetObservationOffset = 12;
        public const int TargetOffsetObservationOffset = 18;
        public const int DistanceObservationIndex = 21;
        public const int GraspPointVelocityObservationOffset = 22;
        public const int HoldProgressObservationIndex = 25;

        public const float MaximumArmDeltaDegPerDecision = 4f;
        public const float MinimumTargetRadius = 0.20f;
        public const float MaximumTargetRadius = 0.85f;
        public const float TargetRadius = 0.02f;
        public const float MinimumInitialCenterDistance = 0.10f;
        public const int MaximumSpawnAttempts = 256;
        public const float PanelWidth = 1.80f;
        public const float PanelDepth = 1.80f;
        public const float PanelThickness = 0.25f;

        public const float SuccessDistance = 0.01f;
        public const float MaximumSuccessPointSpeed = 0.05f;
        public const float RequiredSuccessHoldSeconds = 0.25f;
        public const float EpisodeTimeoutSeconds = 20f;

        public const float WorkspaceRadius = 1.05f;
        public const float WorkspaceMinimumY = -0.15f;
        public const float WorkspaceMaximumY = 1.05f;

        public const float ProgressRewardScale = 2f;
        public const float DecisionTimePenalty = -0.001f;
        public const float TimeoutPenalty = -1f;
        public const float SafetyPenalty = -2f;

        // Generated from the source DG5F prefab. This is one logical policy point,
        // not an average or a set of hand action channels.
        public static readonly Vector3 CalibratedGraspPointLocalPosition =
            new Vector3(0.0170203224f, 0.152462155f, 0.0135399457f);

        public static readonly string[] ArmLinks =
        {
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link"
        };

        public static readonly float[] ArmSafeMinDeg =
        {
            -180f, -120f, 20f, -180f, -150f, -180f
        };

        public static readonly float[] ArmSafeMaxDeg =
        {
            180f, -20f, 140f, 0f, -30f, 180f
        };

        /// <summary>
        /// Samples radius and azimuth uniformly, then rejects a target too close to
        /// the episode's initial GraspPoint. The returned position is robot-base local.
        /// </summary>
        public static Vector3 SpawnTargetLocalPosition(
            System.Random random,
            Vector3 initialGraspPointLocal,
            float targetCenterY = TargetRadius)
        {
            return SpawnTargetLocalPosition(
                random,
                initialGraspPointLocal,
                targetCenterY,
                null);
        }

        /// <summary>
        /// Samples with an additional scene-level acceptance predicate. This lets the
        /// Agent reject panel-edge and robot-overlap candidates without weakening the
        /// deterministic 256-attempt bound in the policy contract.
        /// </summary>
        public static Vector3 SpawnTargetLocalPosition(
            System.Random random,
            Vector3 initialGraspPointLocal,
            float targetCenterY,
            Predicate<Vector3> acceptsCandidate)
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
                || !IsFinite(targetCenterY))
            {
                return false;
            }

            float planarRadius = new Vector2(
                targetLocalPosition.x,
                targetLocalPosition.z).magnitude;
            return planarRadius >= MinimumTargetRadius - 1e-5f
                && planarRadius <= MaximumTargetRadius + 1e-5f
                && Mathf.Abs(targetLocalPosition.y - targetCenterY) <= 1e-5f
                && Vector3.Distance(targetLocalPosition, initialGraspPointLocal)
                    >= MinimumInitialCenterDistance - 1e-5f;
        }

        /// <summary>
        /// Checks that a target sphere footprint remains inside the panel collider.
        /// Box colliders are evaluated in panel-local coordinates, so rotation and
        /// non-uniform scale do not turn the test into a world-AABB approximation.
        /// </summary>
        public static bool IsTargetSphereWithinPanel(
            Vector3 worldCenter,
            float worldRadius,
            Collider panelCollider)
        {
            if (panelCollider == null
                || !panelCollider.enabled
                || !panelCollider.gameObject.activeInHierarchy
                || !IsFinite(worldCenter)
                || !IsFinite(worldRadius)
                || worldRadius < 0f)
            {
                return false;
            }

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

        public static float AccumulateJointTarget(
            float currentTargetDeg,
            float normalizedAction,
            float maximumDeltaDeg,
            float lowerLimitDeg,
            float upperLimitDeg)
        {
            if (!IsFinite(currentTargetDeg) || !IsFinite(normalizedAction)
                || !IsFinite(maximumDeltaDeg))
            {
                return ClampJointTarget(0f, lowerLimitDeg, upperLimitDeg);
            }

            float allowedDelta = Mathf.Min(
                MaximumArmDeltaDegPerDecision,
                Mathf.Max(0f, maximumDeltaDeg));
            float next = currentTargetDeg
                + Mathf.Clamp(normalizedAction, -1f, 1f) * allowedDelta;
            return ClampJointTarget(next, lowerLimitDeg, upperLimitDeg);
        }

        public static float ClampJointTarget(
            float valueDeg,
            float lowerLimitDeg,
            float upperLimitDeg)
        {
            if (!IsFinite(valueDeg) || !IsFinite(lowerLimitDeg) || !IsFinite(upperLimitDeg))
                return 0f;
            return Mathf.Clamp(
                valueDeg,
                Mathf.Min(lowerLimitDeg, upperLimitDeg),
                Mathf.Max(lowerLimitDeg, upperLimitDeg));
        }

        public static float NormalizeJoint(float valueDeg, float lowerDeg, float upperDeg)
        {
            if (!IsFinite(valueDeg) || !IsFinite(lowerDeg) || !IsFinite(upperDeg)
                || upperDeg <= lowerDeg)
            {
                return 0f;
            }
            return Mathf.Clamp(
                (valueDeg - lowerDeg) / (upperDeg - lowerDeg) * 2f - 1f,
                -1f,
                1f);
        }

        /// <summary>
        /// Velocity of a rigidly attached point using v(point) = v(COM) + w x r.
        /// Angular velocity is expressed in radians per second.
        /// </summary>
        public static Vector3 PointVelocity(
            Vector3 centerOfMassVelocity,
            Vector3 angularVelocity,
            Vector3 worldCenterOfMass,
            Vector3 worldPoint)
        {
            if (!IsFinite(centerOfMassVelocity) || !IsFinite(angularVelocity)
                || !IsFinite(worldCenterOfMass) || !IsFinite(worldPoint))
            {
                return Vector3.zero;
            }
            return centerOfMassVelocity
                + Vector3.Cross(angularVelocity, worldPoint - worldCenterOfMass);
        }

        public static float ProgressReward(float previousDistance, float currentDistance)
        {
            if (!IsFinite(previousDistance) || !IsFinite(currentDistance)) return 0f;
            return ProgressRewardScale
                * (Mathf.Max(0f, previousDistance) - Mathf.Max(0f, currentDistance));
        }

        public static float DecisionReward(float previousDistance, float currentDistance)
        {
            return ProgressReward(previousDistance, currentDistance) + DecisionTimePenalty;
        }

        public static float SuccessReward(float elapsedSeconds, float finalDistance)
        {
            float timeFraction = IsFinite(elapsedSeconds)
                ? 1f - Mathf.Clamp01(Mathf.Max(0f, elapsedSeconds) / EpisodeTimeoutSeconds)
                : 0f;
            float precisionFraction = IsFinite(finalDistance)
                ? 1f - Mathf.Clamp01(Mathf.Max(0f, finalDistance) / SuccessDistance)
                : 0f;
            return 2f + 2f * timeFraction + 2f * precisionFraction;
        }

        public static float FailurePenalty(string failureReason)
        {
            return string.Equals(failureReason, "Timeout", StringComparison.Ordinal)
                ? TimeoutPenalty
                : SafetyPenalty;
        }

        public static bool MeetsSuccessState(float centerDistance, float pointSpeed)
        {
            return IsFinite(centerDistance)
                && IsFinite(pointSpeed)
                && centerDistance >= 0f
                && pointSpeed >= 0f
                && centerDistance <= SuccessDistance
                && pointSpeed <= MaximumSuccessPointSpeed;
        }

        public static float NextSuccessHoldSeconds(
            float previousHoldSeconds,
            float centerDistance,
            float pointSpeed,
            float deltaSeconds)
        {
            if (!MeetsSuccessState(centerDistance, pointSpeed)
                || !IsFinite(previousHoldSeconds)
                || !IsFinite(deltaSeconds)
                || deltaSeconds < 0f)
            {
                return 0f;
            }
            return Mathf.Max(0f, previousHoldSeconds) + deltaSeconds;
        }

        public static float SuccessHoldProgress(float holdSeconds)
        {
            if (!IsFinite(holdSeconds)) return 0f;
            return Mathf.Clamp01(
                Mathf.Max(0f, holdSeconds) / RequiredSuccessHoldSeconds);
        }

        public static bool HasCompletedSuccessHold(float holdSeconds)
        {
            return IsFinite(holdSeconds)
                && holdSeconds >= RequiredSuccessHoldSeconds;
        }

        public static bool ReachedEpisodeTimeout(float elapsedSeconds)
        {
            return IsFinite(elapsedSeconds)
                && elapsedSeconds >= EpisodeTimeoutSeconds;
        }

        public static bool IsWithinWorkspace(Vector3 graspPointLocalPosition)
        {
            if (!IsFinite(graspPointLocalPosition)) return false;
            float planarRadius = new Vector2(
                graspPointLocalPosition.x,
                graspPointLocalPosition.z).magnitude;
            return planarRadius <= WorkspaceRadius
                && graspPointLocalPosition.y >= WorkspaceMinimumY
                && graspPointLocalPosition.y <= WorkspaceMaximumY;
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
