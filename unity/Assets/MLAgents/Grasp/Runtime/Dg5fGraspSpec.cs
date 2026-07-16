using System;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// Forward-compatible policy shape and v1 reach-task contract.
    /// Observation and action shapes stay fixed while later stages add rewards.
    /// </summary>
    public static class Dg5fGraspSpec
    {
        public const string SpecVersion = "1.2.0";
        public const string BehaviorName = "DG5FGrasp";
        public const int ObservationSize = 57;
        public const int ActionSize = 7;
        public const int ArmJointCount = 6;
        public const int HandJointCount = 20;
        public const int FingerCount = 5;

        public const float EpisodeTimeoutSeconds = 20f;
        public const float DecisionTimePenalty = -0.001f;
        public const float ApproachPotentialMaximum = 1f;
        public const float ApproachSuccessDistance = 0.05f;
        public const float ApproachSuccessReward = 1f;

        public const float V1MinimumSpawnRadius = 0.35f;
        public const float V1MaximumSpawnRadius = 0.70f;
        public const float SupportTopHeight = 0f;
        public const float PanelWidth = 1.80f;
        public const float PanelDepth = 1.80f;
        public const float PanelThickness = 0.25f;
        public const float MaximumSpawnBallDistance = 0.80f;
        public const float MaximumBallDistance = 0.85f;

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
                throw new ArgumentOutOfRangeException(nameof(channel));
            return Mathf.Lerp(0f, LeftFistDeg[channel], Mathf.Clamp01(closure));
        }

        public static float NormalizeJoint(float valueDeg, float lowerDeg, float upperDeg)
        {
            if (upperDeg <= lowerDeg) return 0f;
            return Mathf.Clamp((valueDeg - lowerDeg) / (upperDeg - lowerDeg) * 2f - 1f, -1f, 1f);
        }

        public static float ApproachPotential(float graspDistance)
        {
            if (!IsFinite(graspDistance)) return 0f;
            return ApproachPotentialMaximum
                * (1f - Mathf.Clamp01(Mathf.Max(0f, graspDistance) / MaximumBallDistance));
        }

        public static float PotentialDelta(float previousPotential, float currentPotential)
        {
            if (!IsFinite(previousPotential) || !IsFinite(currentPotential)) return 0f;
            return currentPotential - previousPotential;
        }

        public static bool HasReachedApproachTarget(float graspDistance)
        {
            return IsFinite(graspDistance)
                && graspDistance <= ApproachSuccessDistance + 1e-6f;
        }

        public static float AreaUniformRadius(float unitSample)
        {
            float minimumSquared = V1MinimumSpawnRadius * V1MinimumSpawnRadius;
            float maximumSquared = V1MaximumSpawnRadius * V1MaximumSpawnRadius;
            return Mathf.Sqrt(Mathf.Lerp(
                minimumSquared,
                maximumSquared,
                Mathf.Clamp01(unitSample)));
        }

        public static Vector3 SpawnBallLocalPosition(
            float radiusUnitSample,
            float azimuthUnitSample,
            float ballRadius)
        {
            float horizontalRadius = AreaUniformRadius(radiusUnitSample);
            float azimuth = Mathf.Clamp01(azimuthUnitSample) * 2f * Mathf.PI;
            return new Vector3(
                Mathf.Cos(azimuth) * horizontalRadius,
                SupportTopHeight + Mathf.Max(0f, ballRadius),
                Mathf.Sin(azimuth) * horizontalRadius);
        }

        public static bool IsValidSpawn(Vector3 ballLocalPosition, float ballRadius)
        {
            if (!IsFinite(ballLocalPosition)) return false;
            float horizontalRadius = new Vector2(ballLocalPosition.x, ballLocalPosition.z).magnitude;
            float nonNegativeBallRadius = Mathf.Max(0f, ballRadius);
            float pedestalTopHeight = ballLocalPosition.y - nonNegativeBallRadius;
            return horizontalRadius >= V1MinimumSpawnRadius - 1e-6f
                && horizontalRadius <= V1MaximumSpawnRadius + 1e-6f
                && Mathf.Abs(ballLocalPosition.x) + nonNegativeBallRadius <= PanelWidth * 0.5f
                && Mathf.Abs(ballLocalPosition.z) + nonNegativeBallRadius <= PanelDepth * 0.5f
                && Mathf.Approximately(pedestalTopHeight, SupportTopHeight)
                && ballLocalPosition.magnitude <= MaximumSpawnBallDistance;
        }

        public static bool ReachedEpisodeTimeout(float elapsedSeconds)
        {
            return IsFinite(elapsedSeconds)
                && elapsedSeconds >= EpisodeTimeoutSeconds - 1e-5f;
        }

        public static bool ShouldResetForBall(Vector3 ballLocalPosition, float pedestalTopHeight)
        {
            return !IsFinite(ballLocalPosition)
                || ballLocalPosition.magnitude > MaximumBallDistance
                || ballLocalPosition.y < pedestalTopHeight;
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
