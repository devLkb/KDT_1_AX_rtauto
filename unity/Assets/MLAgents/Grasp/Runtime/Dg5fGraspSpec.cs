using System;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// Forward-compatible joint-policy shape and v2 grasp-task contract.
    /// Observation and action shapes stay fixed for v2, v3, and v4.
    /// </summary>
    public static class Dg5fGraspSpec
    {
        public const string SpecVersion = "2.1.0";
        public const string BehaviorName = "DG5FGraspJoint";
        public const int ObservationSize = 116;
        public const int ActionSize = 26;
        public const int ArmJointCount = 6;
        public const int HandJointCount = 20;
        public const int FingerCount = 5;
        public const int HandActionOffset = ArmJointCount;
        public const float MaximumHandDeltaDegPerDecision = 4f;
        public const string CurriculumParameterName = "joint26_stage";
        public const int FirstCurriculumStage = 1;
        public const int FinalCurriculumStage = 3;
        public const float PreGraspFraction = 0.35f;

        public const float EpisodeTimeoutSeconds = 20f;
        public const float DecisionTimePenalty = -0.001f;
        public const float ApproachPotentialMaximum = 1f;
        public const float ApproachRewardScale = 0.25f;
        public const float ApproachSuccessDistance = 0.05f;
        public const int ThumbFingerIndex = 0;
        public const int FirstOpposingFingerIndex = 1;
        public const float ThumbContactPotential = 0.25f;
        public const float DualContactPotential = 0.5f;
        public const float ContactHoldPotentialMaximum = 0.5f;
        public const float RequiredContactHoldSeconds = 0.5f;
        public const float GraspSuccessReward = 2f;

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

        // Fixed 35% pre-grasp pose. It is only a curriculum reset pose, not an
        // action-space interpolation target. Channel order: finger 1..5, joint 1..4.
        public static readonly float[] PreGrasp35Deg =
        {
            -14f, 28f, -21f, -21f,
              0f, 35f, 28f, 24.5f,
              0f, 35f, 28f, 24.5f,
              0f, 33.25f, 28f, 24.5f,
              0f, 0f, 28f, 24.5f
        };

        public static int HandActionIndex(int handJointIndex)
        {
            if (handJointIndex < 0 || handJointIndex >= HandJointCount)
                throw new ArgumentOutOfRangeException(nameof(handJointIndex));
            return HandActionOffset + handJointIndex;
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
            float delta = Mathf.Clamp(normalizedAction, -1f, 1f)
                * Mathf.Max(0f, maximumDeltaDeg);
            return ClampJointTarget(currentTargetDeg + delta, lowerLimitDeg, upperLimitDeg);
        }

        public static float ClampJointTarget(float valueDeg, float lowerLimitDeg, float upperLimitDeg)
        {
            if (!IsFinite(valueDeg) || !IsFinite(lowerLimitDeg) || !IsFinite(upperLimitDeg))
                return 0f;
            float minimum = Mathf.Min(lowerLimitDeg, upperLimitDeg);
            float maximum = Mathf.Max(lowerLimitDeg, upperLimitDeg);
            return Mathf.Clamp(valueDeg, minimum, maximum);
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

        public static float FailurePotentialSettlement(
            float approachPotential,
            float contactPotential,
            float holdPotential)
        {
            return ApproachRewardScale * PotentialDelta(approachPotential, 0f)
                + PotentialDelta(contactPotential, 0f)
                + PotentialDelta(holdPotential, 0f);
        }

        public static float FailurePenalty(string failureReason)
        {
            return failureReason == "BallOutOfBounds" || failureReason == "NonFinitePhysics"
                ? -1f
                : 0f;
        }

        public static bool HasThumbContact(bool[] fingerContacts)
        {
            return fingerContacts != null
                && fingerContacts.Length == FingerCount
                && fingerContacts[ThumbFingerIndex];
        }

        public static bool HasOpposingContact(bool[] fingerContacts)
        {
            if (fingerContacts == null || fingerContacts.Length != FingerCount) return false;
            for (int index = FirstOpposingFingerIndex; index < FingerCount; index++)
                if (fingerContacts[index]) return true;
            return false;
        }

        public static bool HasDualContact(bool[] fingerContacts)
        {
            return HasThumbContact(fingerContacts) && HasOpposingContact(fingerContacts);
        }

        public static float ContactPotential(bool thumbContact, bool opposingContact)
        {
            if (!thumbContact) return 0f;
            return opposingContact ? DualContactPotential : ThumbContactPotential;
        }

        public static float ContactHoldPotential(float contactHoldSeconds)
        {
            if (!IsFinite(contactHoldSeconds)) return 0f;
            return ContactHoldPotentialMaximum
                * Mathf.Clamp01(Mathf.Max(0f, contactHoldSeconds) / RequiredContactHoldSeconds);
        }

        public static float NextContactHoldSeconds(
            float previousContactHoldSeconds,
            bool hasDualContact,
            float deltaSeconds)
        {
            if (!hasDualContact) return 0f;
            if (!IsFinite(previousContactHoldSeconds) || !IsFinite(deltaSeconds)) return 0f;
            return Mathf.Max(0f, previousContactHoldSeconds)
                + Mathf.Max(0f, deltaSeconds);
        }

        public static bool HasHeldDualContact(float contactHoldSeconds)
        {
            return IsFinite(contactHoldSeconds)
                && contactHoldSeconds >= RequiredContactHoldSeconds - 1e-6f;
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
