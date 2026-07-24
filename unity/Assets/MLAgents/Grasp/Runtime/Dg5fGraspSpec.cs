using System;
using Unity.MLAgents;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// Forward-compatible policy shape and v1 reach-task contract.
    /// Observation and action shapes stay fixed while later stages add rewards.
    /// </summary>
    public static class Dg5fGraspSpec
    {
        public const string SpecVersion = "1.5.0";
        public const string BehaviorName = "DG5FGrasp";
        public const int ObservationSize = 57;
        public const int ActionSize = 7;
        public const int ArmJointCount = 6;
        public const int HandJointCount = 20;
        public const int FingerCount = 5;

        public const float EpisodeTimeoutSeconds = 20f;
        public const float DecisionTimePenalty = -0.001f;
        public const float ApproachPotentialMaximum = 1f;
        // The training ball has a 0.02 m world-space radius, so the legacy
        // 0.05 m center-distance boundary is exactly 0.03 m from its surface.
        // Keep the legacy constant and tensor slot stable for transfer while
        // expressing the new success contract in surface-relative terms.
        public const float ApproachSuccessDistance = 0.05f;
        public const float TargetSurfaceClearance = 0.03f;
        public const float HoldDurationSeconds = 3f;
        public const float HoldPositionTolerance = 0.01f;
        public const float HoldPotentialMaximum = 0.5f;
        public const string HoldStageParameterName = "hold_stage";
        public const int FirstHoldStage = 1;
        public const int FinalHoldStage = 5;
        public const float NearTargetControlClearance = 0.05f;
        public const float NearTargetActionPenaltyScale = -0.002f;
        public const float ApproachSuccessReward = 1f;

        public const float V1MinimumSpawnRadius = 0.35f;
        public const float V1MaximumSpawnRadius = 0.70f;
        public const float SupportTopHeight = 0f;
        public const float PanelWidth = 1.80f;
        public const float PanelDepth = 1.80f;
        public const float PanelThickness = 0.25f;
        public const float MaximumSpawnBallDistance = 0.80f;
        public const float MaximumBallDistance = 0.85f;
        public const float MinimumTransitClearance = 0.10f;
        public const float MaximumLowClearancePlanarDistance = 0.05f;
        public const float SafetyPenalty = -2f;

        // Palm-local center of the full-hand grasp volume. The palm surface ends
        // near +Z 0.03 m, so this leaves the requested 0.01 m outward clearance.
        public static readonly Vector3 FullHandGraspPointLocalPosition =
            new Vector3(0f, 0.05f, 0.04f);

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

        static int _holdStage = FirstHoldStage;

        public static int CurrentHoldStage => _holdStage;

        public static float RequiredHoldSeconds
        {
            get
            {
                switch (_holdStage)
                {
                    case 1: return 0.25f;
                    case 2: return 0.50f;
                    case 3: return 1.00f;
                    case 4: return 2.00f;
                    default: return HoldDurationSeconds;
                }
            }
        }

        public static float CurrentHoldPositionTolerance
        {
            get
            {
                switch (_holdStage)
                {
                    case 1: return 0.03f;
                    case 2: return 0.025f;
                    case 3: return 0.02f;
                    case 4: return 0.015f;
                    default: return HoldPositionTolerance;
                }
            }
        }

        public static float NearTargetArmDeltaScale
        {
            get
            {
                switch (_holdStage)
                {
                    case 1: return 0.25f;
                    case 2: return 0.20f;
                    case 3: return 0.15f;
                    case 4: return 0.10f;
                    default: return 0.05f;
                }
            }
        }

        public static void RefreshHoldStage()
        {
            SetHoldStage(Academy.Instance.EnvironmentParameters.GetWithDefault(
                HoldStageParameterName,
                FinalHoldStage));
        }

        public static void SetHoldStage(float stage)
        {
            _holdStage = IsFinite(stage)
                ? Mathf.Clamp(Mathf.RoundToInt(stage), FirstHoldStage, FinalHoldStage)
                : FirstHoldStage;
        }

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

        public static float PalmFacingAlignment(Vector3 palmForward, Vector3 palmToBall)
        {
            if (!IsFinite(palmForward)
                || !IsFinite(palmToBall)
                || palmForward.sqrMagnitude <= 1e-12f
                || palmToBall.sqrMagnitude <= 1e-12f)
            {
                return -1f;
            }

            return Mathf.Clamp(Vector3.Dot(palmForward.normalized, palmToBall.normalized), -1f, 1f);
        }

        public static bool IsPalmFacingBall(float palmFacingAlignment)
        {
            // A positive dot product places the ball in the palm-facing half-space.
            // Zero (exactly edge-on) is rejected so the back/side boundary cannot score.
            return IsFinite(palmFacingAlignment) && palmFacingAlignment > 0f;
        }

        public static float DirectionalApproachPotential(
            float graspDistance,
            float palmFacingAlignment)
        {
            return IsPalmFacingBall(palmFacingAlignment)
                ? ApproachPotential(graspDistance)
                : 0f;
        }

        public static bool HasReachedApproachTarget(
            float graspDistance,
            float palmFacingAlignment)
        {
            return IsFinite(graspDistance)
                && graspDistance <= ApproachSuccessDistance + 1e-6f
                && IsPalmFacingBall(palmFacingAlignment);
        }

        public static float SurfaceClearance(float centerDistance, float ballRadius)
        {
            if (!IsFinite(centerDistance) || !IsFinite(ballRadius))
                return float.PositiveInfinity;
            return Mathf.Max(0f, centerDistance - Mathf.Max(0f, ballRadius));
        }

        public static bool IsWithinSurfaceApproachTarget(
            float centerDistance,
            float ballRadius,
            float palmFacingAlignment)
        {
            return SurfaceClearance(centerDistance, ballRadius)
                    <= TargetSurfaceClearance + 1e-6f
                && IsPalmFacingBall(palmFacingAlignment);
        }

        public static bool IsStableHoldPosition(Vector3 graspPosition, Vector3 anchorPosition)
        {
            return IsFinite(graspPosition)
                && IsFinite(anchorPosition)
                && Vector3.Distance(graspPosition, anchorPosition)
                    <= CurrentHoldPositionTolerance + 1e-6f;
        }

        public static float HoldProgress(float holdSeconds)
        {
            if (!IsFinite(holdSeconds)) return 0f;
            return Mathf.Clamp01(
                Mathf.Max(0f, holdSeconds) / RequiredHoldSeconds);
        }

        public static float HoldPotential(float holdSeconds)
        {
            return HoldPotentialMaximum * HoldProgress(holdSeconds);
        }

        public static bool HasCompletedHold(float holdSeconds)
        {
            return IsFinite(holdSeconds)
                && holdSeconds >= RequiredHoldSeconds - 1e-5f;
        }

        public static float HoldAnchorErrorNormalized(
            Vector3 graspPosition,
            Vector3 anchorPosition,
            bool holdActive)
        {
            if (!holdActive) return 0f;
            if (!IsFinite(graspPosition) || !IsFinite(anchorPosition)) return 1f;
            return Mathf.Clamp01(
                Vector3.Distance(graspPosition, anchorPosition)
                / CurrentHoldPositionTolerance);
        }

        public static float HoldStageNormalized()
        {
            return (_holdStage - FirstHoldStage)
                / (float)(FinalHoldStage - FirstHoldStage);
        }

        public static bool UsesNearTargetControl(float surfaceClearance)
        {
            return IsFinite(surfaceClearance)
                && surfaceClearance <= NearTargetControlClearance + 1e-6f;
        }

        public static float NearTargetActionPenalty(float sumSquaredArmActions)
        {
            if (!IsFinite(sumSquaredArmActions)) return 0f;
            return NearTargetActionPenaltyScale
                * Mathf.Max(0f, sumSquaredArmActions)
                / ArmJointCount;
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

        public static float PlanarDistance(Vector3 first, Vector3 second)
        {
            if (!IsFinite(first) || !IsFinite(second))
                return float.PositiveInfinity;
            return new Vector2(first.x - second.x, first.z - second.z).magnitude;
        }

        public static bool IsUnsafeLowClearanceMotion(
            float planarDistance,
            float floorClearance)
        {
            if (!IsFinite(planarDistance) || !IsFinite(floorClearance)) return true;
            return floorClearance < MinimumTransitClearance
                && planarDistance > MaximumLowClearancePlanarDistance;
        }

        public static float FailurePenalty(string reason)
        {
            return string.Equals(reason, "UnsafeSurfaceContact", StringComparison.Ordinal)
                || string.Equals(reason, "PrematureDescent", StringComparison.Ordinal)
                ? SafetyPenalty
                : 0f;
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
