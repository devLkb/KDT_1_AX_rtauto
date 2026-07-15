using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// Versioned, pure-data contract shared by the Agent, scene builder, and tests.
    /// Changing observation order or action semantics requires a new spec version.
    /// </summary>
    public static class Dg5fGraspSpec
    {
        public const string SpecVersion = "2.1.0";
        public const string BehaviorName = "DG5FGrasp";
        public const int ObservationSize = 43;
        public const int ActionSize = 7;
        public const int ArmJointCount = 6;
        public const int HandJointCount = 20;
        public const int FingerCount = 5;

        public const float MinimumSpawnRadius = 0.25f;
        public const float MaximumSpawnRadius = 0.70f;
        public const float MinimumPedestalTopHeight = 0.25f;
        public const float MaximumPedestalTopHeight = 0.65f;
        public const float MaximumSpawnBallDistance = 0.80f;
        public const float MinimumRobotSpawnClearance = 0.05f;
        public const float MaximumBallDistance = 0.85f;
        public const float DistanceRewardScale = -0.01f;
        public const float ContactSuccessSeconds = 1f;
        public const float PenetrationDepthMeters = 0.01f;
        public const float PenetrationFailureSeconds = 0.2f;
        public const float StagnationProgressMeters = 0.02f;
        public const float StagnationFailureSeconds = 60f;

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

        // Validated DG5F probe pose, mirrored for the left-hand URDF.
        // Channel order: finger 1..5, joint 1..4.
        public static readonly float[] LeftFistDeg =
        {
            -40f, 80f, -60f, -60f,
              0f, 100f, 80f, 70f,
              0f, 100f, 80f, 70f,
              0f, 95f, 80f, 70f,
              0f, 0f, 80f, 70f
        };

        public static float GripTargetDeg(int channel, float closure)
        {
            if (channel < 0 || channel >= HandJointCount)
                throw new System.ArgumentOutOfRangeException(nameof(channel));
            return Mathf.Lerp(0f, LeftFistDeg[channel], Mathf.Clamp01(closure));
        }

        public static float NormalizeJoint(float valueDeg, float lowerDeg, float upperDeg)
        {
            if (upperDeg <= lowerDeg) return 0f;
            return Mathf.Clamp((valueDeg - lowerDeg) / (upperDeg - lowerDeg) * 2f - 1f, -1f, 1f);
        }

        public static float DistanceReward(float graspDistance)
        {
            if (!IsFinite(graspDistance)) return DistanceRewardScale;
            return DistanceRewardScale * Mathf.Clamp01(graspDistance / MaximumBallDistance);
        }

        public static float AreaUniformRadius(float unitSample)
        {
            float minimumSquared = MinimumSpawnRadius * MinimumSpawnRadius;
            float maximumSquared = MaximumSpawnRadius * MaximumSpawnRadius;
            return Mathf.Sqrt(Mathf.Lerp(minimumSquared, maximumSquared, Mathf.Clamp01(unitSample)));
        }

        public static Vector3 SpawnBallLocalPosition(
            float radiusUnitSample,
            float azimuthUnitSample,
            float heightUnitSample,
            float ballRadius)
        {
            float horizontalRadius = AreaUniformRadius(radiusUnitSample);
            float azimuth = Mathf.Clamp01(azimuthUnitSample) * Mathf.PI * 2f;
            float topHeight = Mathf.Lerp(
                MinimumPedestalTopHeight,
                MaximumPedestalTopHeight,
                Mathf.Clamp01(heightUnitSample));
            return new Vector3(
                Mathf.Cos(azimuth) * horizontalRadius,
                topHeight + Mathf.Max(0f, ballRadius),
                Mathf.Sin(azimuth) * horizontalRadius);
        }

        public static bool IsValidSpawn(Vector3 ballLocalPosition, float ballRadius)
        {
            if (!IsFinite(ballLocalPosition)) return false;
            float horizontalRadius = new Vector2(ballLocalPosition.x, ballLocalPosition.z).magnitude;
            float pedestalTopHeight = ballLocalPosition.y - Mathf.Max(0f, ballRadius);
            return horizontalRadius >= MinimumSpawnRadius
                && horizontalRadius <= MaximumSpawnRadius
                && pedestalTopHeight >= MinimumPedestalTopHeight
                && pedestalTopHeight <= MaximumPedestalTopHeight
                && ballLocalPosition.magnitude <= MaximumSpawnBallDistance;
        }

        public static bool HasMinimumRobotSpawnClearance(
            float centerToRobotSurfaceDistance,
            float ballRadius)
        {
            if (!IsFinite(centerToRobotSurfaceDistance) || !IsFinite(ballRadius)) return false;
            return centerToRobotSurfaceDistance
                >= Mathf.Max(0f, ballRadius) + MinimumRobotSpawnClearance;
        }

        public static bool HasGraspContact(bool thumbContact, bool opposingContact)
        {
            return thumbContact && opposingContact;
        }

        public static float UpdateHoldSeconds(float currentSeconds, bool condition, float deltaTime)
        {
            return condition ? currentSeconds + Mathf.Max(0f, deltaTime) : 0f;
        }

        public static bool ReachedDuration(float elapsedSeconds, float requiredSeconds)
        {
            return elapsedSeconds >= requiredSeconds - 1e-5f;
        }

        public static bool ShouldResetForBall(Vector3 ballLocalPosition, float pedestalTopHeight)
        {
            return !IsFinite(ballLocalPosition)
                || ballLocalPosition.magnitude > MaximumBallDistance
                || ballLocalPosition.y < pedestalTopHeight;
        }

        public static bool UpdateStagnation(
            float graspDistance,
            float deltaTime,
            ref float bestDistance,
            ref float meaningfulProgressDistance,
            ref float stagnationSeconds)
        {
            if (IsFinite(graspDistance)) bestDistance = Mathf.Min(bestDistance, graspDistance);

            if (meaningfulProgressDistance - bestDistance >= StagnationProgressMeters - 1e-6f)
            {
                meaningfulProgressDistance = bestDistance;
                stagnationSeconds = 0f;
            }
            else
            {
                stagnationSeconds += Mathf.Max(0f, deltaTime);
            }

            return ReachedDuration(stagnationSeconds, StagnationFailureSeconds);
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
