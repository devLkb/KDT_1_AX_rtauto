using NUnit.Framework;
using UnityEngine;

namespace KDT.GraspTraining.Tests
{
    public sealed class Dg5fGraspSpecTests
    {
        [SetUp]
        public void ResetHoldStage()
        {
            Dg5fGraspSpec.SetHoldStage(Dg5fGraspSpec.FirstHoldStage);
        }

        [Test]
        public void V1KeepsTheForwardCompatiblePolicyShape()
        {
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("1.5.0"));
            Assert.That(Dg5fGraspSpec.BehaviorName, Is.EqualTo("DG5FGrasp"));
            Assert.That(Dg5fGraspSpec.ObservationSize, Is.EqualTo(57));
            Assert.That(Dg5fGraspSpec.ActionSize, Is.EqualTo(7));
            Assert.That(Dg5fGraspSpec.ArmJointCount, Is.EqualTo(6));
            Assert.That(Dg5fGraspSpec.HandJointCount, Is.EqualTo(20));
            Assert.That(Dg5fGraspSpec.FingerCount, Is.EqualTo(5));
            Assert.That(Dg5fGraspSpec.FullHandGraspPointLocalPosition,
                Is.EqualTo(new Vector3(0f, 0.05f, 0.04f)));
        }

        [Test]
        public void ApproachPotentialRewardsProgressAndReversesOnRetreat()
        {
            float far = Dg5fGraspSpec.ApproachPotential(0.60f);
            float near = Dg5fGraspSpec.ApproachPotential(0.30f);

            Assert.That(Dg5fGraspSpec.ApproachPotential(Dg5fGraspSpec.MaximumBallDistance), Is.Zero);
            Assert.That(Dg5fGraspSpec.ApproachPotential(0f),
                Is.EqualTo(Dg5fGraspSpec.ApproachPotentialMaximum));
            Assert.That(Dg5fGraspSpec.PotentialDelta(far, near), Is.GreaterThan(0f));
            Assert.That(Dg5fGraspSpec.PotentialDelta(near, far),
                Is.EqualTo(-Dg5fGraspSpec.PotentialDelta(far, near)).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.PotentialDelta(near, near), Is.Zero);
            Assert.That(Dg5fGraspSpec.ApproachPotential(float.NaN), Is.Zero);
            Assert.That(Dg5fGraspSpec.PotentialDelta(float.NaN, near), Is.Zero);
        }

        [Test]
        public void ApproachSuccessUsesTheExactFiveCentimeterBoundaryOnThePalmSide()
        {
            Assert.That(Dg5fGraspSpec.ApproachSuccessDistance, Is.EqualTo(0.05f));
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0.05001f, 1f), Is.False);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0.05f, 1f), Is.True);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0f, 1f), Is.True);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0.05f, 0f), Is.False);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0.05f, -1f), Is.False);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(float.NaN, 1f), Is.False);

            float configuredPointAlignment = Dg5fGraspSpec.PalmFacingAlignment(
                Vector3.forward,
                Dg5fGraspSpec.FullHandGraspPointLocalPosition);
            Assert.That(configuredPointAlignment, Is.GreaterThan(0f),
                "The configured full-hand point must remain in front of the palm plane.");
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0f, configuredPointAlignment),
                Is.True);
        }

        [Test]
        public void SurfaceTargetAndHoldCurriculumKeepTheTransferShape()
        {
            const float ballRadius = 0.02f;
            Assert.That(Dg5fGraspSpec.ObservationSize, Is.EqualTo(57));
            Assert.That(Dg5fGraspSpec.ActionSize, Is.EqualTo(7));
            Assert.That(Dg5fGraspSpec.TargetSurfaceClearance, Is.EqualTo(0.03f));
            Assert.That(Dg5fGraspSpec.HoldDurationSeconds, Is.EqualTo(3f));
            Assert.That(Dg5fGraspSpec.HoldPositionTolerance, Is.EqualTo(0.01f));
            Assert.That(Dg5fGraspSpec.RequiredHoldSeconds, Is.EqualTo(0.25f));
            Assert.That(Dg5fGraspSpec.CurrentHoldPositionTolerance, Is.EqualTo(0.03f));
            Assert.That(Dg5fGraspSpec.NearTargetArmDeltaScale, Is.EqualTo(0.25f));
            Assert.That(Dg5fGraspSpec.SurfaceClearance(0.05f, ballRadius),
                Is.EqualTo(0.03f).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.IsWithinSurfaceApproachTarget(
                0.05f, ballRadius, 1f), Is.True);
            Assert.That(Dg5fGraspSpec.IsWithinSurfaceApproachTarget(
                0.05001f, ballRadius, 1f), Is.False);
            Assert.That(Dg5fGraspSpec.IsWithinSurfaceApproachTarget(
                0.05f, ballRadius, 0f), Is.False);
            Assert.That(Dg5fGraspSpec.IsStableHoldPosition(
                Vector3.zero, new Vector3(0.03f, 0f, 0f)), Is.True);
            Assert.That(Dg5fGraspSpec.IsStableHoldPosition(
                Vector3.zero, new Vector3(0.03001f, 0f, 0f)), Is.False);
            Assert.That(Dg5fGraspSpec.HoldPotential(0f), Is.Zero);
            Assert.That(Dg5fGraspSpec.HoldPotential(0.125f),
                Is.EqualTo(Dg5fGraspSpec.HoldPotentialMaximum * 0.5f));
            Assert.That(Dg5fGraspSpec.HoldPotential(0.25f),
                Is.EqualTo(Dg5fGraspSpec.HoldPotentialMaximum));
            Assert.That(Dg5fGraspSpec.HasCompletedHold(0.24f), Is.False);
            Assert.That(Dg5fGraspSpec.HasCompletedHold(0.25f), Is.True);

            Dg5fGraspSpec.SetHoldStage(5f);
            Assert.That(Dg5fGraspSpec.RequiredHoldSeconds, Is.EqualTo(3f));
            Assert.That(Dg5fGraspSpec.CurrentHoldPositionTolerance, Is.EqualTo(0.01f));
            Assert.That(Dg5fGraspSpec.NearTargetArmDeltaScale, Is.EqualTo(0.05f));
            Assert.That(Dg5fGraspSpec.HoldPotential(1.5f),
                Is.EqualTo(Dg5fGraspSpec.HoldPotentialMaximum * 0.5f));
            Assert.That(Dg5fGraspSpec.HasCompletedHold(2.99f), Is.False);
            Assert.That(Dg5fGraspSpec.HasCompletedHold(3f), Is.True);
        }

        [Test]
        public void CurriculumSignalsAndNearTargetControlAreBounded()
        {
            Dg5fGraspSpec.SetHoldStage(float.NaN);
            Assert.That(Dg5fGraspSpec.CurrentHoldStage, Is.EqualTo(1));
            Assert.That(Dg5fGraspSpec.HoldStageNormalized(), Is.Zero);
            Assert.That(Dg5fGraspSpec.HoldAnchorErrorNormalized(
                Vector3.zero, Vector3.one, false), Is.Zero);
            Assert.That(Dg5fGraspSpec.HoldAnchorErrorNormalized(
                Vector3.zero, new Vector3(0.015f, 0f, 0f), true),
                Is.EqualTo(0.5f).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.UsesNearTargetControl(0.05f), Is.True);
            Assert.That(Dg5fGraspSpec.UsesNearTargetControl(0.05001f), Is.False);
            Assert.That(Dg5fGraspSpec.NearTargetActionPenalty(6f),
                Is.EqualTo(Dg5fGraspSpec.NearTargetActionPenaltyScale));

            Dg5fGraspSpec.SetHoldStage(99f);
            Assert.That(Dg5fGraspSpec.CurrentHoldStage, Is.EqualTo(5));
            Assert.That(Dg5fGraspSpec.HoldStageNormalized(), Is.EqualTo(1f));
        }

        [Test]
        public void PalmFacingAlignmentRejectsBackOfHandApproachRewards()
        {
            float front = Dg5fGraspSpec.PalmFacingAlignment(Vector3.forward, Vector3.forward);
            float side = Dg5fGraspSpec.PalmFacingAlignment(Vector3.forward, Vector3.right);
            float back = Dg5fGraspSpec.PalmFacingAlignment(Vector3.forward, Vector3.back);

            Assert.That(front, Is.EqualTo(1f).Within(1e-6f));
            Assert.That(side, Is.Zero.Within(1e-6f));
            Assert.That(back, Is.EqualTo(-1f).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.PalmFacingAlignment(Vector3.zero, Vector3.forward),
                Is.EqualTo(-1f));
            Assert.That(Dg5fGraspSpec.PalmFacingAlignment(Vector3.forward, Vector3.zero),
                Is.EqualTo(-1f));

            float distancePotential = Dg5fGraspSpec.ApproachPotential(0.04f);
            Assert.That(Dg5fGraspSpec.DirectionalApproachPotential(0.04f, front),
                Is.EqualTo(distancePotential));
            Assert.That(Dg5fGraspSpec.DirectionalApproachPotential(0.04f, side), Is.Zero);
            Assert.That(Dg5fGraspSpec.DirectionalApproachPotential(0.04f, back), Is.Zero);
        }

        [Test]
        public void V1SpawnIsAreaUniformAcrossAllCardinalDirections()
        {
            const float ballRadius = 0.02f;
            Assert.That(Dg5fGraspSpec.V1MinimumSpawnRadius, Is.EqualTo(0.35f));
            Assert.That(Dg5fGraspSpec.V1MaximumSpawnRadius, Is.EqualTo(0.70f));
            Assert.That(Dg5fGraspSpec.AreaUniformRadius(0f),
                Is.EqualTo(Dg5fGraspSpec.V1MinimumSpawnRadius).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.AreaUniformRadius(1f),
                Is.EqualTo(Dg5fGraspSpec.V1MaximumSpawnRadius).Within(1e-6f));

            Vector3 east = Dg5fGraspSpec.SpawnBallLocalPosition(0f, 0f, ballRadius);
            Vector3 north = Dg5fGraspSpec.SpawnBallLocalPosition(0.25f, 0.25f, ballRadius);
            Vector3 west = Dg5fGraspSpec.SpawnBallLocalPosition(0.5f, 0.5f, ballRadius);
            Vector3 south = Dg5fGraspSpec.SpawnBallLocalPosition(1f, 0.75f, ballRadius);

            Assert.That(Azimuth(east), Is.EqualTo(0f).Within(1e-3f));
            Assert.That(Azimuth(north), Is.EqualTo(90f).Within(1e-3f));
            Assert.That(Mathf.Abs(Azimuth(west)), Is.EqualTo(180f).Within(1e-3f));
            Assert.That(Azimuth(south), Is.EqualTo(-90f).Within(1e-3f));
            Assert.That(new Vector2(south.x, south.z).magnitude,
                Is.EqualTo(Dg5fGraspSpec.V1MaximumSpawnRadius).Within(1e-6f));
            Assert.That(east.y, Is.EqualTo(Dg5fGraspSpec.SupportTopHeight + ballRadius));
            Assert.That(Dg5fGraspSpec.IsValidSpawn(east, ballRadius), Is.True);
            Assert.That(Dg5fGraspSpec.IsValidSpawn(north, ballRadius), Is.True);
            Assert.That(Dg5fGraspSpec.IsValidSpawn(west, ballRadius), Is.True);
            Assert.That(Dg5fGraspSpec.IsValidSpawn(south, ballRadius), Is.True);
        }

        [Test]
        public void GripProfileInterpolatesAndClampsClosure()
        {
            Assert.That(Dg5fGraspSpec.GripTargetDeg(0, 0f), Is.Zero);
            Assert.That(Dg5fGraspSpec.GripTargetDeg(0, 0.5f), Is.EqualTo(-20f));
            Assert.That(Dg5fGraspSpec.GripTargetDeg(0, 2f), Is.EqualTo(-40f));
            Assert.That(() => Dg5fGraspSpec.GripTargetDeg(-1, 0f),
                Throws.TypeOf<System.ArgumentOutOfRangeException>());
            Assert.That(() => Dg5fGraspSpec.GripTargetDeg(20, 0f),
                Throws.TypeOf<System.ArgumentOutOfRangeException>());
        }

        [Test]
        public void ArmSafeRangesAndNormalizationRemainStableForTransferLearning()
        {
            Assert.That(Dg5fGraspSpec.ArmSafeMinDeg, Is.EqualTo(new[]
            {
                -180f, -120f, 20f, -180f, -150f, -180f
            }));
            Assert.That(Dg5fGraspSpec.ArmSafeMaxDeg, Is.EqualTo(new[]
            {
                180f, -20f, 140f, 0f, -30f, 180f
            }));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(-180f, -180f, 180f), Is.EqualTo(-1f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(0f, -180f, 180f), Is.Zero);
            Assert.That(Dg5fGraspSpec.NormalizeJoint(180f, -180f, 180f), Is.EqualTo(1f));
        }

        [Test]
        public void EpisodeTimeoutAndBallSafetyRetainExactBoundaries()
        {
            Assert.That(Dg5fGraspSpec.ReachedEpisodeTimeout(19.999f), Is.False);
            Assert.That(Dg5fGraspSpec.ReachedEpisodeTimeout(20f), Is.True);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(0.85f, 0f, 0f), -1f), Is.False);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(0.851f, 0f, 0f), -1f), Is.True);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(float.NaN, 0f, 0f), -1f), Is.True);
        }

        [Test]
        public void LowClearanceSafetyBlocksFloorSweepsWithoutChangingPolicyShape()
        {
            Assert.That(Dg5fGraspSpec.ObservationSize, Is.EqualTo(57));
            Assert.That(Dg5fGraspSpec.ActionSize, Is.EqualTo(7));
            Assert.That(Dg5fGraspSpec.MinimumTransitClearance, Is.EqualTo(0.10f));
            Assert.That(Dg5fGraspSpec.MaximumLowClearancePlanarDistance,
                Is.EqualTo(0.05f));

            Assert.That(Dg5fGraspSpec.IsUnsafeLowClearanceMotion(0.051f, 0.099f),
                Is.True);
            Assert.That(Dg5fGraspSpec.IsUnsafeLowClearanceMotion(0.05f, 0.099f),
                Is.False);
            Assert.That(Dg5fGraspSpec.IsUnsafeLowClearanceMotion(0.051f, 0.10f),
                Is.False);
            Assert.That(Dg5fGraspSpec.IsUnsafeLowClearanceMotion(float.NaN, 0.10f),
                Is.True);
        }

        [Test]
        public void OnlyPanelSafetyFailuresReceiveTheDemoPenalty()
        {
            Assert.That(Dg5fGraspSpec.SafetyPenalty, Is.EqualTo(-2f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("UnsafeSurfaceContact"),
                Is.EqualTo(-2f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("PrematureDescent"),
                Is.EqualTo(-2f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("Timeout"), Is.Zero);
            Assert.That(Dg5fGraspSpec.FailurePenalty("BallOutOfBounds"), Is.Zero);
        }

        static float Azimuth(Vector3 position)
        {
            return Mathf.Atan2(position.z, position.x) * Mathf.Rad2Deg;
        }
    }
}
