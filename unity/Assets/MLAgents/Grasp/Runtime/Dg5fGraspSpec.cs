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
        public const string SpecVersion = "1.7.0";
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
        // Fixed pre-grasp dwell used by every stage (see RequiredHoldSeconds).
        public const float PreGraspHoldSeconds = 0.3f;
        public const float HoldPotentialMaximum = 0.5f;
        // Weight of the per-step sustain reward (see HoldDwellReward). Small so
        // a full continuous hold accumulates on the order of the one-shot
        // HoldPotential rather than dwarfing the +1 success reward.
        public const float HoldDwellRewardScale = 0.01f;
        // Raised from 0.25: the top-down approach angle is the primary training
        // objective (park the hand vertically above the object for grasping), so
        // its alignment potential is now the dominant shaping term.
        public const float TopDownAlignmentPotentialMaximum = 0.5f;
        public const float TopDownAlignmentRewardDistance = 0.15f;
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
        // Premature/misaligned descents abort a navigation attempt rather than
        // physically striking the panel. The old -2 cliff made the expected
        // value of ever descending negative at low success rates, so the policy
        // collapsed to a risk-averse hover ~7 cm above the surface and never
        // reached the hold contract. A softer abort penalty keeps descents
        // discouraged without dominating the gradient. Real surface contact
        // (UnsafeSurfaceContact) still receives the full SafetyPenalty.
        public const float DescentAbortPenalty = -0.5f;

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

        // Pre-grasp hold is decoupled from the curriculum stage. The training
        // goal is to park the hand in a graspable top-down pose and hand off to
        // the grasp controller, not to hold for seconds. The stage now tightens
        // only the approach angle / position tolerance (see below), while the
        // hold stays a short fixed dwell that just confirms the pre-grasp pose
        // is stable before handoff. This removes the seconds-long hold wall that
        // blocked the angle from tightening.
        public static float RequiredHoldSeconds => PreGraspHoldSeconds;

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

        public static float CurrentMaximumTopDownAngleDegrees
        {
            get
            {
                switch (_holdStage)
                {
                    case 1: return 80f;
                    case 2: return 60f;
                    case 3: return 45f;
                    case 4: return 30f;
                    default: return 15f;
                }
            }
        }

        public static float CurrentTopDownRewardEntryAngleDegrees
        {
            get
            {
                switch (_holdStage)
                {
                    case 1: return 100f;
                    case 2: return 85f;
                    case 3: return 65f;
                    case 4: return 50f;
                    default: return 35f;
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

        public static float TopDownAlignment(Vector3 graspForward, Vector3 robotUp)
        {
            if (!IsFinite(graspForward)
                || !IsFinite(robotUp)
                || graspForward.sqrMagnitude <= 1e-12f
                || robotUp.sqrMagnitude <= 1e-12f)
            {
                return -1f;
            }

            return Mathf.Clamp(
                Vector3.Dot(graspForward.normalized, -robotUp.normalized),
                -1f,
                1f);
        }

        public static float TopDownAngleDegrees(float topDownAlignment)
        {
            if (!IsFinite(topDownAlignment)) return 180f;
            return Mathf.Acos(Mathf.Clamp(topDownAlignment, -1f, 1f))
                * Mathf.Rad2Deg;
        }

        public static bool IsTopDownAligned(float topDownAlignment)
        {
            if (!IsFinite(topDownAlignment)) return false;
            float minimumAlignment = Mathf.Cos(
                CurrentMaximumTopDownAngleDegrees * Mathf.Deg2Rad);
            return topDownAlignment >= minimumAlignment - 1e-6f;
        }

        public static float TopDownAlignmentPotential(
            float graspDistance,
            float heightAboveBall,
            float topDownAlignment)
        {
            if (!IsFinite(graspDistance)
                || !IsFinite(heightAboveBall)
                || !IsFinite(topDownAlignment)
                || graspDistance > TopDownAlignmentRewardDistance + 1e-6f
                || heightAboveBall < -1e-6f)
            {
                return 0f;
            }

            float entryAngle = CurrentTopDownRewardEntryAngleDegrees;
            float targetAngle = CurrentMaximumTopDownAngleDegrees;
            float progress = Mathf.Clamp01(
                (entryAngle - TopDownAngleDegrees(topDownAlignment))
                / (entryAngle - targetAngle));

            // Squaring keeps the signal monotonic while concentrating more of
            // the finite episode reward near the current lesson's target.
            return TopDownAlignmentPotentialMaximum * progress * progress;
        }

        public static float NewBestPotentialDelta(
            float previousBestPotential,
            float currentPotential)
        {
            if (!IsFinite(previousBestPotential) || !IsFinite(currentPotential))
                return 0f;
            return Mathf.Max(0f, currentPotential - previousBestPotential);
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
            float palmFacingAlignment,
            float topDownAlignment)
        {
            return SurfaceClearance(centerDistance, ballRadius)
                    <= TargetSurfaceClearance + 1e-6f
                && IsPalmFacingBall(palmFacingAlignment)
                && IsTopDownAligned(topDownAlignment);
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

        // Per-decision dwell reward paid every step the hold is active, scaled
        // by the current continuous-hold fraction. The new-best HoldPotential
        // delta alone gave no incentive to *sustain* a hold once a brief best
        // was banked, so policies plateaued around a ~0.1-0.2 s tap and never
        // approached the 0.5 s+ contract. This term rewards long uninterrupted
        // holds far more than repeated short taps (the fraction restarts at 0
        // on any anchor break), so it pulls dwell time up without opening a
        // stationary reward-farming loop.
        public static float HoldDwellReward(float holdSeconds)
        {
            return HoldDwellRewardScale * HoldProgress(holdSeconds);
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
            float distributionUnitSample,
            float azimuthUnitSample,
            float ballRadius)
        {
            float horizontalRadius = AreaUniformRadius(radiusUnitSample);
            float azimuth = SpawnAzimuthRadians(
                distributionUnitSample,
                azimuthUnitSample);
            return new Vector3(
                Mathf.Cos(azimuth) * horizontalRadius,
                SupportTopHeight + Mathf.Max(0f, ballRadius),
                Mathf.Sin(azimuth) * horizontalRadius);
        }

        public static float SpawnAzimuthRadians(
            float distributionUnitSample,
            float azimuthUnitSample)
        {
            float distribution = Mathf.Clamp01(distributionUnitSample);
            float azimuth = Mathf.Clamp01(azimuthUnitSample);
            if (distribution < 0.5f)
                return azimuth * 2f * Mathf.PI;

            // Robot-local +Z is forward and +X is right. Half the samples stay
            // globally uniform; the other half is split evenly between the
            // forward and right 90-degree sectors.
            float sectorCenter = distribution < 0.75f
                ? 0.5f * Mathf.PI
                : 0f;
            return sectorCenter + (azimuth - 0.5f) * 0.5f * Mathf.PI;
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

        public static bool IsMisalignedDescent(
            float planarDistance,
            float floorClearance,
            float topDownAlignment)
        {
            if (!IsFinite(planarDistance)
                || !IsFinite(floorClearance)
                || !IsFinite(topDownAlignment))
            {
                return true;
            }

            return floorClearance < MinimumTransitClearance
                && planarDistance <= MaximumLowClearancePlanarDistance + 1e-6f
                && !IsTopDownAligned(topDownAlignment);
        }

        public static float FailurePenalty(string reason)
        {
            if (string.Equals(reason, "UnsafeSurfaceContact", StringComparison.Ordinal))
                return SafetyPenalty;
            if (string.Equals(reason, "PrematureDescent", StringComparison.Ordinal)
                || string.Equals(reason, "MisalignedDescent", StringComparison.Ordinal))
                return DescentAbortPenalty;
            return 0f;
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
