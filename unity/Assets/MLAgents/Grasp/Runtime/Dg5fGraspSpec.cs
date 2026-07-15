using System;
using UnityEngine;

namespace KDT.GraspTraining
{
    public enum Dg5fGraspPhase
    {
        Reach,
        Grasp,
        Lift,
        Hold
    }

    public enum Dg5fFailureReason
    {
        None,
        GripLost,
        Dropped,
        WorkspaceExit,
        Penetration,
        NonFinitePhysics,
        Timeout
    }

    public readonly struct Dg5fCurriculumStage
    {
        public Dg5fCurriculumStage(
            int index,
            float halfAngleDegrees,
            float minimumSpawnRadius,
            float maximumSpawnRadius,
            float liftTargetMeters,
            float holdTargetSeconds,
            float movementPenaltyLambda)
        {
            Index = index;
            HalfAngleDegrees = halfAngleDegrees;
            MinimumSpawnRadius = minimumSpawnRadius;
            MaximumSpawnRadius = maximumSpawnRadius;
            LiftTargetMeters = liftTargetMeters;
            HoldTargetSeconds = holdTargetSeconds;
            MovementPenaltyLambda = movementPenaltyLambda;
        }

        public int Index { get; }
        public float HalfAngleDegrees { get; }
        public float MinimumSpawnRadius { get; }
        public float MaximumSpawnRadius { get; }
        public float LiftTargetMeters { get; }
        public float HoldTargetSeconds { get; }
        public float MovementPenaltyLambda { get; }

        public float HoldMinimumLiftMeters =>
            Mathf.Max(0f, LiftTargetMeters - Dg5fGraspSpec.HoldHeightToleranceMeters);
    }

    public readonly struct Dg5fEpisodeInput
    {
        public Dg5fEpisodeInput(
            bool graspContact,
            float liftMeters,
            float ballSpeed,
            bool workspaceValid,
            bool penetration,
            bool physicsFinite)
        {
            GraspContact = graspContact;
            LiftMeters = liftMeters;
            BallSpeed = ballSpeed;
            WorkspaceValid = workspaceValid;
            Penetration = penetration;
            PhysicsFinite = physicsFinite;
        }

        public bool GraspContact { get; }
        public float LiftMeters { get; }
        public float BallSpeed { get; }
        public bool WorkspaceValid { get; }
        public bool Penetration { get; }
        public bool PhysicsFinite { get; }
    }

    public readonly struct Dg5fEpisodeStepResult
    {
        internal Dg5fEpisodeStepResult(
            bool stableGraspReached,
            bool liftTargetReached,
            bool succeeded,
            Dg5fFailureReason failureReason)
        {
            StableGraspReached = stableGraspReached;
            LiftTargetReached = liftTargetReached;
            Succeeded = succeeded;
            FailureReason = failureReason;
        }

        public bool StableGraspReached { get; }
        public bool LiftTargetReached { get; }
        public bool Succeeded { get; }
        public Dg5fFailureReason FailureReason { get; }
        public bool Failed => FailureReason != Dg5fFailureReason.None;
    }

    /// <summary>
    /// Pure episode state used by the Agent and boundary-focused EditMode tests.
    /// A timestep is credited to at most one timed phase, preventing grasp time from
    /// being reused as hold time when two phase boundaries are crossed together.
    /// </summary>
    public struct Dg5fEpisodeState
    {
        public Dg5fGraspPhase Phase { get; private set; }
        public float GraspSeconds { get; private set; }
        public float HoldSeconds { get; private set; }
        public float EpisodeSeconds { get; private set; }
        public float PenetrationSeconds { get; private set; }
        public float MaximumLiftMeters { get; private set; }
        public float MaximumHoldSeconds { get; private set; }
        public bool LiftTargetWasReached { get; private set; }
        public bool IsTerminal { get; private set; }
        public bool Succeeded { get; private set; }
        public Dg5fFailureReason FailureReason { get; private set; }

        public void Reset()
        {
            Phase = Dg5fGraspPhase.Reach;
            GraspSeconds = 0f;
            HoldSeconds = 0f;
            EpisodeSeconds = 0f;
            PenetrationSeconds = 0f;
            MaximumLiftMeters = 0f;
            MaximumHoldSeconds = 0f;
            LiftTargetWasReached = false;
            IsTerminal = false;
            Succeeded = false;
            FailureReason = Dg5fFailureReason.None;
        }

        public Dg5fEpisodeStepResult Advance(
            Dg5fEpisodeInput input,
            Dg5fCurriculumStage stage,
            float deltaTime)
        {
            if (IsTerminal) return default;

            float dt = Mathf.Max(0f, deltaTime);
            EpisodeSeconds += dt;
            if (Dg5fGraspSpec.IsFinite(input.LiftMeters))
                MaximumLiftMeters = Mathf.Max(MaximumLiftMeters, input.LiftMeters);

            if (!input.PhysicsFinite
                || !Dg5fGraspSpec.IsFinite(input.LiftMeters)
                || !Dg5fGraspSpec.IsFinite(input.BallSpeed))
            {
                return Fail(Dg5fFailureReason.NonFinitePhysics);
            }

            if (!input.WorkspaceValid) return Fail(Dg5fFailureReason.WorkspaceExit);

            PenetrationSeconds = Dg5fGraspSpec.UpdateHoldSeconds(
                PenetrationSeconds,
                input.Penetration,
                dt);
            if (Dg5fGraspSpec.ReachedDuration(
                PenetrationSeconds,
                Dg5fGraspSpec.PenetrationFailureSeconds))
            {
                return Fail(Dg5fFailureReason.Penetration);
            }

            if (Dg5fGraspSpec.ReachedEpisodeTimeout(EpisodeSeconds))
                return Fail(Dg5fFailureReason.Timeout);

            bool stableGrasp = Phase == Dg5fGraspPhase.Lift || Phase == Dg5fGraspPhase.Hold;
            if (stableGrasp)
            {
                if (!input.GraspContact) return Fail(Dg5fFailureReason.GripLost);
                if (LiftTargetWasReached
                    && MaximumLiftMeters > Dg5fGraspSpec.DropFailureLiftMeters + 1e-5f
                    && Dg5fGraspSpec.ShouldFailForDrop(input.LiftMeters, stage))
                {
                    return Fail(Dg5fFailureReason.Dropped);
                }
            }

            switch (Phase)
            {
                case Dg5fGraspPhase.Reach:
                    if (!input.GraspContact) return default;
                    Phase = Dg5fGraspPhase.Grasp;
                    GraspSeconds = dt;
                    if (Dg5fGraspSpec.IsStableGrasp(GraspSeconds))
                    {
                        Phase = Dg5fGraspPhase.Lift;
                        return new Dg5fEpisodeStepResult(true, false, false, Dg5fFailureReason.None);
                    }
                    return default;

                case Dg5fGraspPhase.Grasp:
                    GraspSeconds = Dg5fGraspSpec.UpdateHoldSeconds(
                        GraspSeconds,
                        input.GraspContact,
                        dt);
                    if (!input.GraspContact)
                    {
                        Phase = Dg5fGraspPhase.Reach;
                        return default;
                    }
                    if (Dg5fGraspSpec.IsStableGrasp(GraspSeconds))
                    {
                        Phase = Dg5fGraspPhase.Lift;
                        return new Dg5fEpisodeStepResult(true, false, false, Dg5fFailureReason.None);
                    }
                    return default;

                case Dg5fGraspPhase.Lift:
                    if (!Dg5fGraspSpec.ReachedLiftTarget(input.LiftMeters, stage)) return default;
                    Phase = Dg5fGraspPhase.Hold;
                    HoldSeconds = 0f;
                    LiftTargetWasReached = true;
                    return new Dg5fEpisodeStepResult(false, true, false, Dg5fFailureReason.None);

                case Dg5fGraspPhase.Hold:
                    HoldSeconds = Dg5fGraspSpec.UpdateHoldSeconds(
                        HoldSeconds,
                        Dg5fGraspSpec.IsValidHold(input, stage),
                        dt);
                    MaximumHoldSeconds = Mathf.Max(MaximumHoldSeconds, HoldSeconds);
                    if (!Dg5fGraspSpec.ReachedDuration(HoldSeconds, stage.HoldTargetSeconds))
                        return default;
                    IsTerminal = true;
                    Succeeded = true;
                    return new Dg5fEpisodeStepResult(false, false, true, Dg5fFailureReason.None);

                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        Dg5fEpisodeStepResult Fail(Dg5fFailureReason reason)
        {
            IsTerminal = true;
            Succeeded = false;
            FailureReason = reason;
            return new Dg5fEpisodeStepResult(false, false, false, reason);
        }
    }

    /// <summary>
    /// Versioned contract shared by the Agent, scene builder, and tests.
    /// Observation, action, reward, and episode semantics are checkpoint-breaking.
    /// </summary>
    public static class Dg5fGraspSpec
    {
        public const string SpecVersion = "4.1.0";
        public const string BehaviorName = "DG5FGraspV4";
        public const int ObservationSize = 57;
        public const int ActionSize = 7;
        public const int ArmJointCount = 6;
        public const int HandJointCount = 20;
        public const int FingerCount = 5;

        public const float StableGraspSeconds = 0.25f;
        public const float HoldHeightToleranceMeters = 0.01f;
        public const float MaximumHoldBallSpeed = 0.05f;
        public const float DropFailureLiftMeters = 0.02f;
        public const float EpisodeTimeoutSeconds = 20f;

        public const float DecisionTimePenalty = -0.001f;
        public const float ReachPotentialMaximum = 1f;
        public const float ContactPotentialThumb = 0.25f;
        public const float ContactPotentialOpposing = 0.25f;
        public const float StableGraspReward = 0.5f;
        public const float LiftPotentialMaximum = 1f;
        public const float LiftTargetReward = 1f;
        public const float HoldPotentialMaximum = 1f;
        public const float SuccessReward = 3f;
        public const float TimeoutFailureReward = -0.1f;
        public const float DropFailureReward = -0.5f;
        public const float SafetyFailureReward = -1f;

        public const float MinimumSpawnRadius = 0.25f;
        public const float MaximumSpawnRadius = 0.70f;
        public const float SupportTopHeight = 0f;
        public const float PanelWidth = 1.80f;
        public const float PanelDepth = 1.80f;
        public const float PanelThickness = 0.25f;
        public const float MaximumSpawnBallDistance = 0.80f;
        public const float MinimumRobotSpawnClearance = 0.05f;
        public const float MaximumBallDistance = 0.85f;
        public const float PenetrationDepthMeters = 0.01f;
        public const float PenetrationFailureSeconds = 0.2f;

        public static readonly Dg5fCurriculumStage[] CurriculumStages =
        {
            new Dg5fCurriculumStage(0, 15f, 0.25f, 0.35f, 0.02f, 0.5f, 0f),
            new Dg5fCurriculumStage(1, 30f, 0.25f, 0.45f, 0.05f, 1f, 0f),
            new Dg5fCurriculumStage(2, 60f, 0.25f, 0.55f, 0.10f, 2f, 0.01f),
            new Dg5fCurriculumStage(3, 120f, 0.25f, 0.65f, 0.10f, 3f, 0.01f),
            new Dg5fCurriculumStage(4, 180f, 0.25f, 0.70f, 0.10f, 5f, 0.02f)
        };

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

        public static Dg5fCurriculumStage GetCurriculumStage(int index)
        {
            return CurriculumStages[Mathf.Clamp(index, 0, CurriculumStages.Length - 1)];
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

        public static float ReachPotential(float graspDistance)
        {
            if (!IsFinite(graspDistance)) return 0f;
            return ReachPotentialMaximum
                * (1f - Mathf.Clamp01(Mathf.Max(0f, graspDistance) / MaximumBallDistance));
        }

        public static float LiftPotential(float liftMeters, float liftTargetMeters)
        {
            if (!IsFinite(liftMeters) || !IsFinite(liftTargetMeters) || liftTargetMeters <= 0f)
                return 0f;
            return LiftPotentialMaximum * Mathf.Clamp01(liftMeters / liftTargetMeters);
        }

        public static float ContactPotential(bool thumbContact, bool opposingContact)
        {
            return (thumbContact ? ContactPotentialThumb : 0f)
                + (opposingContact ? ContactPotentialOpposing : 0f);
        }

        public static float HoldPotential(float holdSeconds, float holdTargetSeconds)
        {
            if (!IsFinite(holdSeconds) || !IsFinite(holdTargetSeconds) || holdTargetSeconds <= 0f)
                return 0f;
            return HoldPotentialMaximum
                * Mathf.Clamp01(Mathf.Max(0f, holdSeconds) / holdTargetSeconds);
        }

        public static float PotentialDelta(float previousPotential, float currentPotential)
        {
            if (!IsFinite(previousPotential) || !IsFinite(currentPotential)) return 0f;
            return currentPotential - previousPotential;
        }

        public static float NormalizedArmMovement(float[] previousJointDeg, float[] currentJointDeg)
        {
            if (previousJointDeg == null) throw new ArgumentNullException(nameof(previousJointDeg));
            if (currentJointDeg == null) throw new ArgumentNullException(nameof(currentJointDeg));
            if (previousJointDeg.Length != ArmJointCount || currentJointDeg.Length != ArmJointCount)
                throw new ArgumentException($"Expected {ArmJointCount} arm joint values.");

            float movement = 0f;
            for (int i = 0; i < ArmJointCount; i++)
            {
                if (!IsFinite(previousJointDeg[i]) || !IsFinite(currentJointDeg[i])) continue;
                float range = ArmSafeMaxDeg[i] - ArmSafeMinDeg[i];
                if (range > 0f)
                    movement += Mathf.Abs(currentJointDeg[i] - previousJointDeg[i]) / range;
            }
            return movement;
        }

        public static float MovementPenalty(float normalizedMovement, float lambda)
        {
            if (!IsFinite(normalizedMovement) || !IsFinite(lambda)) return 0f;
            return -Mathf.Max(0f, lambda) * Mathf.Max(0f, normalizedMovement);
        }

        public static float TerminalReward(bool success, Dg5fFailureReason failureReason)
        {
            if (success) return SuccessReward;
            switch (failureReason)
            {
                case Dg5fFailureReason.Timeout:
                    // Timeout only means the task was not finished; a heavy penalty
                    // here drowns the shaping gradient during early exploration.
                    return TimeoutFailureReward;
                case Dg5fFailureReason.GripLost:
                case Dg5fFailureReason.Dropped:
                    return DropFailureReward;
                default:
                    return SafetyFailureReward;
            }
        }

        public static float AreaUniformRadius(float unitSample)
        {
            return AreaUniformRadius(unitSample, MinimumSpawnRadius, MaximumSpawnRadius);
        }

        public static float AreaUniformRadius(float unitSample, float minimumRadius, float maximumRadius)
        {
            float minimum = Mathf.Max(0f, Mathf.Min(minimumRadius, maximumRadius));
            float maximum = Mathf.Max(minimum, Mathf.Max(minimumRadius, maximumRadius));
            float minimumSquared = minimum * minimum;
            float maximumSquared = maximum * maximum;
            return Mathf.Sqrt(Mathf.Lerp(minimumSquared, maximumSquared, Mathf.Clamp01(unitSample)));
        }

        public static Vector3 SpawnBallLocalPosition(
            float radiusUnitSample,
            float azimuthUnitSample,
            float ballRadius)
        {
            float horizontalRadius = AreaUniformRadius(radiusUnitSample);
            float azimuth = Mathf.Clamp01(azimuthUnitSample) * Mathf.PI * 2f;
            return SpawnPosition(horizontalRadius, azimuth, ballRadius);
        }

        public static Vector3 SpawnBallLocalPosition(
            float radiusUnitSample,
            float azimuthUnitSample,
            float ballRadius,
            Dg5fCurriculumStage stage,
            float centerAzimuthDegrees)
        {
            float horizontalRadius = AreaUniformRadius(
                radiusUnitSample,
                stage.MinimumSpawnRadius,
                stage.MaximumSpawnRadius);
            float offsetDegrees = Mathf.Lerp(
                -stage.HalfAngleDegrees,
                stage.HalfAngleDegrees,
                Mathf.Clamp01(azimuthUnitSample));
            float azimuth = (centerAzimuthDegrees + offsetDegrees) * Mathf.Deg2Rad;
            return SpawnPosition(horizontalRadius, azimuth, ballRadius);
        }

        static Vector3 SpawnPosition(float horizontalRadius, float azimuth, float ballRadius)
        {
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
            return horizontalRadius >= MinimumSpawnRadius
                && horizontalRadius <= MaximumSpawnRadius
                && Mathf.Abs(ballLocalPosition.x) + nonNegativeBallRadius <= PanelWidth * 0.5f
                && Mathf.Abs(ballLocalPosition.z) + nonNegativeBallRadius <= PanelDepth * 0.5f
                && Mathf.Approximately(pedestalTopHeight, SupportTopHeight)
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

        public static bool IsStableGrasp(float graspSeconds)
        {
            return ReachedDuration(graspSeconds, StableGraspSeconds);
        }

        public static bool ReachedLiftTarget(float liftMeters, Dg5fCurriculumStage stage)
        {
            return IsFinite(liftMeters)
                && liftMeters >= stage.LiftTargetMeters - 1e-5f;
        }

        public static bool IsValidHold(Dg5fEpisodeInput input, Dg5fCurriculumStage stage)
        {
            return input.GraspContact
                && IsFinite(input.LiftMeters)
                && input.LiftMeters >= stage.HoldMinimumLiftMeters - 1e-5f
                && IsFinite(input.BallSpeed)
                && Mathf.Abs(input.BallSpeed) <= MaximumHoldBallSpeed + 1e-5f;
        }

        public static bool ShouldFailForDrop(float liftMeters, Dg5fCurriculumStage stage)
        {
            // The first lesson's target is itself 2 cm, so its relaxed task cannot
            // simultaneously treat the target boundary as a drop failure.
            return stage.LiftTargetMeters > DropFailureLiftMeters + 1e-5f
                && liftMeters <= DropFailureLiftMeters + 1e-5f;
        }

        public static bool ReachedEpisodeTimeout(float elapsedSeconds)
        {
            return ReachedDuration(elapsedSeconds, EpisodeTimeoutSeconds);
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
