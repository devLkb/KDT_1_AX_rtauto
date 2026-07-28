using KDT.GraspLiftTraining;
using NUnit.Framework;
using UnityEngine;

namespace KDT.GraspLiftTraining.Tests
{
    public sealed class Dg5fGraspLiftSpecTests
    {
        [TearDown]
        public void ResetStage()
        {
            Dg5fGraspLiftSpec.SetGraspStage(Dg5fGraspLiftSpec.FinalGraspStage);
            Dg5fGraspLiftSpec.SetBlockWidth(Dg5fGraspLiftSpec.BlockWidth);
            Dg5fGraspLiftSpec.SetBlockHeight(Dg5fGraspLiftSpec.BlockHeight);
            Dg5fGraspLiftSpec.SetToppleLimit(Dg5fGraspLiftSpec.ToppleLimitDegrees);
            Dg5fGraspLiftSpec.SetBlockComHeightFraction(
                Dg5fGraspLiftSpec.BlockComHeightFraction);
            Dg5fGraspLiftSpec.SetTopDownAlignmentPotentialMax(
                Dg5fGraspLiftSpec.TopDownAlignmentPotentialMax);
            Dg5fGraspLiftSpec.SetActionRatePenaltyScale(
                Dg5fGraspLiftSpec.ActionRatePenaltyScale);
            Dg5fGraspLiftSpec.SetHandSurfacePenaltyPerSecond(
                Dg5fGraspLiftSpec.HandSurfacePenaltyPerSecond);
            Dg5fGraspLiftSpec.SetGraspPosturePenaltyScale(
                Dg5fGraspLiftSpec.GraspPosturePenaltyScale);
        }

        // --- block size parameter -------------------------------------------

        [Test]
        public void BlockWidthIsClampedToASensibleRange()
        {
            Dg5fGraspLiftSpec.SetBlockWidth(0.001f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MinimumBlockWidth, Dg5fGraspLiftSpec.CurrentBlockWidth, 1e-6f);
            Dg5fGraspLiftSpec.SetBlockWidth(10f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MaximumBlockWidth, Dg5fGraspLiftSpec.CurrentBlockWidth, 1e-6f);
            Dg5fGraspLiftSpec.SetBlockWidth(float.NaN);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.BlockWidth, Dg5fGraspLiftSpec.CurrentBlockWidth, 1e-6f);
        }

        [Test]
        public void BlockHeightIsClampedAndRejectsNonFiniteValues()
        {
            Dg5fGraspLiftSpec.SetBlockHeight(0.001f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MinimumBlockHeight,
                Dg5fGraspLiftSpec.CurrentBlockHeight,
                1e-6f);
            Dg5fGraspLiftSpec.SetBlockHeight(10f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MaximumBlockHeight,
                Dg5fGraspLiftSpec.CurrentBlockHeight,
                1e-6f);
            Dg5fGraspLiftSpec.SetBlockHeight(float.NaN);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.BlockHeight,
                Dg5fGraspLiftSpec.CurrentBlockHeight,
                1e-6f);
            Dg5fGraspLiftSpec.SetBlockHeight(float.PositiveInfinity);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.BlockHeight,
                Dg5fGraspLiftSpec.CurrentBlockHeight,
                1e-6f);
        }

        [Test]
        public void CurrentBlockHalfHeightTracksCurrentBlockHeight()
        {
            Dg5fGraspLiftSpec.SetBlockHeight(0.12f);
            Assert.AreEqual(0.06f, Dg5fGraspLiftSpec.CurrentBlockHalfHeight, 1e-6f);
            Assert.AreEqual(
                0.04f,
                Dg5fGraspLiftSpec.CurrentGraspTargetHeightOffset,
                1e-6f,
                "the grasp target must retain its 2 cm inset below the top face");
        }

        [Test]
        public void DefaultBlockHeightMatchesTheExistingGeometry()
        {
            Dg5fGraspLiftSpec.SetBlockHeight(Dg5fGraspLiftSpec.BlockHeight);
            Assert.AreEqual(0.09f, Dg5fGraspLiftSpec.CurrentBlockHeight, 1e-6f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.GraspTargetHeightOffset,
                Dg5fGraspLiftSpec.CurrentGraspTargetHeightOffset,
                0f);
        }

        [Test]
        public void BlockMassFollowsBlockVolume()
        {
            Dg5fGraspLiftSpec.SetBlockWidth(0.03f);
            Dg5fGraspLiftSpec.SetBlockHeight(0.09f);
            float smallWidthAndHeight = Dg5fGraspLiftSpec.CurrentBlockMass;
            Dg5fGraspLiftSpec.SetBlockWidth(0.05f);
            float largeWidth = Dg5fGraspLiftSpec.CurrentBlockMass;
            Assert.Greater(largeWidth, smallWidthAndHeight);
            Dg5fGraspLiftSpec.SetBlockHeight(0.12f);
            float largeWidthAndHeight = Dg5fGraspLiftSpec.CurrentBlockMass;
            Assert.Greater(largeWidthAndHeight, largeWidth);
            Assert.AreEqual(
                0.05f * 0.05f * 0.12f * Dg5fGraspLiftSpec.BlockDensity,
                largeWidthAndHeight,
                1e-6f);
        }

        [Test]
        public void DefaultBlockWidthFitsTheMeasuredHandAperture()
        {
            // GraspLiftHandGeometryProbe measured the closed hand's thumb-to-finger
            // separation bottoming out near 0.031-0.036 m. A block wider than that
            // cannot be opposed, which is why the original 0.055 m block was never
            // retained. Keep the default inside the aperture.
            Assert.LessOrEqual(Dg5fGraspLiftSpec.BlockWidth, 0.045f);
            Assert.GreaterOrEqual(Dg5fGraspLiftSpec.BlockWidth, 0.025f);
        }

        [Test]
        public void BlockCenterOfMassHeightFractionIsClampedAndMappedToLocalSpace()
        {
            Dg5fGraspLiftSpec.SetBlockComHeightFraction(0f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MinimumBlockComHeightFraction,
                Dg5fGraspLiftSpec.CurrentBlockComHeightFraction,
                1e-6f);
            Dg5fGraspLiftSpec.SetBlockComHeightFraction(1f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MaximumBlockComHeightFraction,
                Dg5fGraspLiftSpec.CurrentBlockComHeightFraction,
                1e-6f);
            Dg5fGraspLiftSpec.SetBlockComHeightFraction(float.NaN);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.BlockComHeightFraction,
                Dg5fGraspLiftSpec.CurrentBlockComHeightFraction,
                1e-6f);

            float fraction = 0.25f;
            Dg5fGraspLiftSpec.SetBlockComHeightFraction(fraction);
            Assert.AreEqual(
                fraction - 0.5f,
                Dg5fGraspLiftSpec.CurrentBlockCenterOfMassLocal.y,
                1e-6f);
        }

        // --- contract -------------------------------------------------------

        [Test]
        public void PolicyShapeMatchesTheTransferableReachContract()
        {
            Assert.AreEqual(57, Dg5fGraspLiftSpec.ObservationSize);
            Assert.AreEqual(7, Dg5fGraspLiftSpec.ActionSize);
            Assert.AreEqual(6, Dg5fGraspLiftSpec.ContactPointCount);
            Assert.AreEqual(5, Dg5fGraspLiftSpec.PalmContactIndex);
            Assert.AreEqual(20, Dg5fGraspLiftSpec.LeftFistDeg.Length);
            Assert.AreEqual(6, Dg5fGraspLiftSpec.ArmLinks.Length);
        }

        // --- curriculum -----------------------------------------------------

        [Test]
        public void GraspStageIsClampedAndRounded()
        {
            Dg5fGraspLiftSpec.SetGraspStage(-4f);
            Assert.AreEqual(Dg5fGraspLiftSpec.FirstGraspStage, Dg5fGraspLiftSpec.CurrentGraspStage);
            Dg5fGraspLiftSpec.SetGraspStage(99f);
            Assert.AreEqual(Dg5fGraspLiftSpec.FinalGraspStage, Dg5fGraspLiftSpec.CurrentGraspStage);
            Dg5fGraspLiftSpec.SetGraspStage(1.6f);
            Assert.AreEqual(2, Dg5fGraspLiftSpec.CurrentGraspStage);
            Dg5fGraspLiftSpec.SetGraspStage(float.NaN);
            Assert.AreEqual(Dg5fGraspLiftSpec.FirstGraspStage, Dg5fGraspLiftSpec.CurrentGraspStage);
        }

        [Test]
        public void CurriculumMonotonicallyTightensTheLiftContract()
        {
            Dg5fGraspLiftSpec.SetGraspStage(1);
            float easyHeight = Dg5fGraspLiftSpec.CurrentLiftTargetHeight;
            float easyHold = Dg5fGraspLiftSpec.CurrentLiftHoldSeconds;
            Dg5fGraspLiftSpec.SetGraspStage(3);
            Assert.Greater(Dg5fGraspLiftSpec.CurrentLiftTargetHeight, easyHeight);
            Assert.Greater(Dg5fGraspLiftSpec.CurrentLiftHoldSeconds, easyHold);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.LiftTargetHeight,
                Dg5fGraspLiftSpec.CurrentLiftTargetHeight,
                1e-6f);
        }

        // --- approach shaping -----------------------------------------------

        [Test]
        public void ApproachPotentialRisesAsDistanceFalls()
        {
            Assert.AreEqual(
                Dg5fGraspLiftSpec.ApproachPotentialMaximum,
                Dg5fGraspLiftSpec.ApproachPotential(0f),
                1e-5f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.ApproachPotential(10f), 1e-5f);
            Assert.Greater(
                Dg5fGraspLiftSpec.ApproachPotential(0.1f),
                Dg5fGraspLiftSpec.ApproachPotential(0.4f));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.ApproachPotential(float.NaN));
        }

        [Test]
        public void ApproachShapingHasRealGradientInTheLastFewCentimetres()
        {
            // The regression this guards: with only the workspace-scaled coarse term,
            // closing from 9 cm to 1 cm was worth +0.09 and the policy parked 8 cm
            // away — outside the hand's ~4 cm grasp volume — so it never grasped.
            float far = Dg5fGraspLiftSpec.DirectionalApproachPotential(0.09f, 1f);
            float near = Dg5fGraspLiftSpec.DirectionalApproachPotential(0.01f, 1f);
            Assert.Greater(near - far, 0.5f,
                "the last 8 cm must be worth substantially more than the coarse term alone");

            // And it must still be monotonic in distance.
            float previous = float.MaxValue;
            for (int i = 0; i <= 20; i++)
            {
                float value = Dg5fGraspLiftSpec.DirectionalApproachPotential(i * 0.01f, 1f);
                Assert.LessOrEqual(value, previous + 1e-6f);
                previous = value;
            }
        }

        [Test]
        public void FineApproachPotentialSaturatesOutsideItsRange()
        {
            Assert.AreEqual(
                Dg5fGraspLiftSpec.FineApproachPotentialMaximum,
                Dg5fGraspLiftSpec.FineApproachPotential(0f),
                1e-5f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.FineApproachPotential(
                    Dg5fGraspLiftSpec.FineApproachDistance),
                1e-5f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.FineApproachPotential(1f), 1e-5f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.FineApproachPotential(float.NaN));
        }

        [Test]
        public void ClosingOnlyCountsInsideTheHandsGraspVolume()
        {
            // Measured grasp volume is ~4 cm across, so the ready radius must not be
            // so generous that the hand can bank the closing reward in mid air.
            Assert.LessOrEqual(Dg5fGraspLiftSpec.GraspReadyDistance, 0.05f);
        }

        [Test]
        public void ApproachPotentialIsGatedOnThePalmFacingTheObject()
        {
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.DirectionalApproachPotential(0.1f, -0.9f));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.DirectionalApproachPotential(0.1f, 0f));
            Assert.Greater(Dg5fGraspLiftSpec.DirectionalApproachPotential(0.1f, 0.9f), 0f);
        }

        [Test]
        public void TopDownAlignmentIsOneWhenTheGraspAxisPointsStraightDown()
        {
            float alignment = Dg5fGraspLiftSpec.TopDownAlignment(Vector3.down, Vector3.up);
            Assert.AreEqual(1f, alignment, 1e-5f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.TopDownAngleDegrees(alignment), 1e-3f);
            Assert.IsTrue(Dg5fGraspLiftSpec.IsTopDownAligned(alignment));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsTopDownAligned(
                Dg5fGraspLiftSpec.TopDownAlignment(Vector3.forward, Vector3.up)));
        }

        [Test]
        public void TopDownPotentialOnlyPaysCloseToAndAboveTheObject()
        {
            float aligned = Dg5fGraspLiftSpec.TopDownAlignment(Vector3.down, Vector3.up);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.TopDownAlignmentPotential(0.5f, 0.05f, aligned),
                1e-6f,
                "far from the object the wrist pose is meaningless");
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.TopDownAlignmentPotential(0.05f, -0.05f, aligned),
                1e-6f,
                "below the object a top-down pose cannot grasp it");
            Assert.Greater(
                Dg5fGraspLiftSpec.TopDownAlignmentPotential(0.05f, 0.05f, aligned), 0f);
        }

        [Test]
        public void TopDownPotentialMaximumIsTunableClampedAndRejectsNonFinite()
        {
            float aligned = Dg5fGraspLiftSpec.TopDownAlignment(Vector3.down, Vector3.up);
            Dg5fGraspLiftSpec.SetTopDownAlignmentPotentialMax(2f);
            Assert.AreEqual(
                2f,
                Dg5fGraspLiftSpec.TopDownAlignmentPotential(0.05f, 0.05f, aligned),
                1e-6f);

            Dg5fGraspLiftSpec.SetTopDownAlignmentPotentialMax(-1f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MinimumTopDownAlignmentPotentialMax,
                Dg5fGraspLiftSpec.CurrentTopDownAlignmentPotentialMax,
                1e-6f);
            Dg5fGraspLiftSpec.SetTopDownAlignmentPotentialMax(10f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MaximumTopDownAlignmentPotentialMax,
                Dg5fGraspLiftSpec.CurrentTopDownAlignmentPotentialMax,
                1e-6f);
            Dg5fGraspLiftSpec.SetTopDownAlignmentPotentialMax(float.NaN);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.TopDownAlignmentPotentialMax,
                Dg5fGraspLiftSpec.CurrentTopDownAlignmentPotentialMax,
                1e-6f);
        }

        // --- grip shaping ----------------------------------------------------

        [Test]
        public void ClosingOnlyEarnsPotentialNextToTheObject()
        {
            Assert.Greater(Dg5fGraspLiftSpec.ClosurePotential(1f, true), 0f);
            Assert.Greater(
                Dg5fGraspLiftSpec.ClosurePotential(1f, true),
                Dg5fGraspLiftSpec.ClosurePotential(0.3f, true));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.ClosurePotential(1f, false), 1e-6f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.ClosurePotential(0f, true), 1e-6f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.ClosurePotential(float.NaN, true), 1e-6f);
        }

        [Test]
        public void ClosingAwayFromTheObjectIsPenalisedAndOpeningIsFree()
        {
            Assert.Less(Dg5fGraspLiftSpec.ClosureFarPenalty(0.1f, false), 0f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.ClosureFarPenalty(-0.1f, false), 1e-6f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.ClosureFarPenalty(0.1f, true), 1e-6f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.ClosureFarPenalty(float.NaN, false), 1e-6f);
        }

        [Test]
        public void ClosingRewardSaturatesAtTheBestGripClosure()
        {
            // Past EffectiveGripClosure the fingers curl past one another and the grip
            // re-opens, so squeezing harder must not pay more.
            float atBestGrip =
                Dg5fGraspLiftSpec.ClosurePotential(Dg5fGraspLiftSpec.EffectiveGripClosure, true);
            Assert.AreEqual(Dg5fGraspLiftSpec.CloseNearObjectReward, atBestGrip, 1e-5f);
            Assert.AreEqual(atBestGrip, Dg5fGraspLiftSpec.ClosurePotential(1f, true), 1e-5f);
            Assert.Less(
                Dg5fGraspLiftSpec.ClosurePotential(
                    Dg5fGraspLiftSpec.EffectiveGripClosure * 0.5f, true),
                atBestGrip);
        }

        [Test]
        public void ClosingIsWorthMoreThanTheRiskOfNudgingTheBlock()
        {
            // The failure that killed the first runs: closing could earn at most +0.5
            // while brushing the block cost -1.0 and ended the episode, so hovering
            // with an open hand was optimal.
            Assert.GreaterOrEqual(
                Dg5fGraspLiftSpec.CloseNearObjectReward,
                Mathf.Abs(Dg5fGraspLiftSpec.PushAwayPenalty));
        }

        [Test]
        public void PumpingTheFingersCannotFarmReward()
        {
            // Total credit for closing is bounded by the new-best delta of the
            // closure potential, so an open/close cycle pays for the first close and
            // nothing thereafter.
            float best = 0f;
            float earned = 0f;
            for (int cycle = 0; cycle < 5; cycle++)
            {
                foreach (float closure in new[] { 0.25f, 0.5f, 0.75f, 1f, 0.5f, 0f })
                {
                    float potential = Dg5fGraspLiftSpec.ClosurePotential(closure, true);
                    earned += Dg5fGraspLiftSpec.NewBestPotentialDelta(best, potential);
                    best = Mathf.Max(best, potential);
                }
            }
            Assert.AreEqual(Dg5fGraspLiftSpec.CloseNearObjectReward, earned, 1e-5f);
        }

        // --- grasp confirmation ----------------------------------------------

        static Vector3[] Directions(params Vector3[] values)
        {
            var buffer = new Vector3[Dg5fGraspLiftSpec.ContactPointCount];
            for (int i = 0; i < values.Length; i++) buffer[i] = values[i];
            return buffer;
        }

        [Test]
        public void OppositionAngleFindsTheWidestContactPair()
        {
            Vector3[] opposed = Directions(Vector3.right, Vector3.left, Vector3.up);
            Assert.AreEqual(
                180f,
                Dg5fGraspLiftSpec.MaximumOppositionAngleDegrees(opposed, 3),
                1e-3f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.MaximumOppositionAngleDegrees(opposed, 1),
                1e-6f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.MaximumOppositionAngleDegrees(null, 3));
        }

        [Test]
        public void PokingFromOneSideIsNotAGrasp()
        {
            // Three fingertips all bunched on the same face: enough contacts, but no
            // opposition, so it must not register as a grasp.
            Vector3[] sameSide = Directions(
                new Vector3(1f, 0f, 0f),
                new Vector3(1f, 0.1f, 0f),
                new Vector3(1f, -0.1f, 0f));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsForceClosureLike(sameSide, 3));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsGraspCandidate(
                3, sameSide, Vector3.zero, Vector3.zero, 1f));
        }

        [Test]
        public void OpposedContactsAroundTheObjectAreAGrasp()
        {
            Vector3[] opposed = Directions(Vector3.right, Vector3.left, Vector3.forward);
            Assert.IsTrue(Dg5fGraspLiftSpec.IsForceClosureLike(opposed, 3));
            Assert.IsTrue(Dg5fGraspLiftSpec.IsGraspCandidate(
                3, opposed, Vector3.zero, Vector3.zero, 1f));
        }

        [Test]
        public void TooFewContactsIsNeverAGrasp()
        {
            Vector3[] opposed = Directions(Vector3.right, Vector3.left);
            Assert.IsFalse(Dg5fGraspLiftSpec.IsGraspCandidate(
                2, opposed, Vector3.zero, Vector3.zero, 1f));
        }

        [Test]
        public void AnOpenHandIsNeverAGrasp()
        {
            Vector3[] opposed = Directions(Vector3.right, Vector3.left, Vector3.forward);
            Assert.IsFalse(Dg5fGraspLiftSpec.IsGraspCandidate(
                3, opposed, Vector3.zero, Vector3.zero, 0f));
        }

        [Test]
        public void ObjectMustSitInsideTheContactCage()
        {
            Vector3[] opposed = Directions(Vector3.right, Vector3.left, Vector3.forward);
            Vector3 farCentroid = new Vector3(0.5f, 0f, 0f);
            Assert.IsFalse(Dg5fGraspLiftSpec.IsGraspGeometryValid(Vector3.zero, farCentroid));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsGraspCandidate(
                3, opposed, Vector3.zero, farCentroid, 1f));
        }

        [Test]
        public void GraspIsOnlyConfirmedAfterTheFullDwell()
        {
            Assert.IsFalse(Dg5fGraspLiftSpec.IsGraspConfirmed(0.02f));
            Assert.IsTrue(Dg5fGraspLiftSpec.IsGraspConfirmed(
                Dg5fGraspLiftSpec.GraspConfirmSeconds));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.GraspProgress(0f), 1e-6f);
            Assert.AreEqual(
                1f,
                Dg5fGraspLiftSpec.GraspProgress(Dg5fGraspLiftSpec.GraspConfirmSeconds * 2f),
                1e-6f);
        }

        // --- lift -------------------------------------------------------------

        [Test]
        public void LiftPotentialTracksHeightAndSaturatesAtTheTarget()
        {
            Dg5fGraspLiftSpec.SetGraspStage(Dg5fGraspLiftSpec.FinalGraspStage);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.LiftPotential(-0.05f), 1e-6f);
            Assert.Greater(
                Dg5fGraspLiftSpec.LiftPotential(0.08f),
                Dg5fGraspLiftSpec.LiftPotential(0.04f));
            Assert.AreEqual(
                Dg5fGraspLiftSpec.LiftPotentialMaximum,
                Dg5fGraspLiftSpec.LiftPotential(1f),
                1e-6f);
        }

        [Test]
        public void AFlyingObjectIsNotAStableLift()
        {
            float height = Dg5fGraspLiftSpec.LiftTargetHeight + 0.01f;
            Assert.IsTrue(Dg5fGraspLiftSpec.IsStableLift(height, 0.1f));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsStableLift(height, 5f));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsStableLift(0.01f, 0.1f));
        }

        [Test]
        public void LiftHeightIsMeasuredAgainstTheSpawnHeight()
        {
            Assert.AreEqual(0.07f, Dg5fGraspLiftSpec.LiftHeight(0.37f, 0.30f), 1e-5f);
            Assert.AreEqual(-0.02f, Dg5fGraspLiftSpec.LiftHeight(0.28f, 0.30f), 1e-5f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.LiftHeight(float.NaN, 0.3f));
        }

        // --- penalties / termination ------------------------------------------

        [Test]
        public void ArmActionRatePenaltyIsProportionalAndNormalisedByArmJointCount()
        {
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.ArmActionRatePenalty(0f), 1e-6f);

            float onePerJoint =
                Dg5fGraspLiftSpec.ArmActionRatePenalty(Dg5fGraspLiftSpec.ArmJointCount);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.ActionRatePenaltyScale,
                onePerJoint,
                1e-6f);
            Assert.AreEqual(
                onePerJoint * 2f,
                Dg5fGraspLiftSpec.ArmActionRatePenalty(
                    Dg5fGraspLiftSpec.ArmJointCount * 2f),
                1e-6f);
            Assert.Less(onePerJoint, 0f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.ArmActionRatePenalty(float.NaN),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.ArmActionRatePenalty(float.PositiveInfinity),
                1e-6f);
        }

        [Test]
        public void ActionRatePenaltyScaleClampsRejectsNonFiniteAndCanBeDisabled()
        {
            Dg5fGraspLiftSpec.SetActionRatePenaltyScale(-2f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MinimumActionRatePenaltyScale,
                Dg5fGraspLiftSpec.CurrentActionRatePenaltyScale,
                1e-6f);
            Dg5fGraspLiftSpec.SetActionRatePenaltyScale(-0.1f);
            Assert.AreEqual(
                -0.1f,
                Dg5fGraspLiftSpec.CurrentActionRatePenaltyScale,
                1e-6f,
                "the widened range must accept a value rejected by the old -0.02 clamp");
            Dg5fGraspLiftSpec.SetActionRatePenaltyScale(1f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MaximumActionRatePenaltyScale,
                Dg5fGraspLiftSpec.CurrentActionRatePenaltyScale,
                1e-6f);
            Dg5fGraspLiftSpec.SetActionRatePenaltyScale(0f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.ArmActionRatePenalty(
                    Dg5fGraspLiftSpec.ArmJointCount),
                1e-6f);
            Dg5fGraspLiftSpec.SetActionRatePenaltyScale(float.PositiveInfinity);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.ActionRatePenaltyScale,
                Dg5fGraspLiftSpec.CurrentActionRatePenaltyScale,
                1e-6f);
        }

        [Test]
        public void HandSurfaceContactPenaltyOnlyChargesPreGraspPositiveFiniteTime()
        {
            float oneSecond = Dg5fGraspLiftSpec.HandSurfaceContactPenalty(1f, false);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.HandSurfacePenaltyPerSecond,
                oneSecond,
                1e-6f);
            Assert.AreEqual(
                oneSecond * 2f,
                Dg5fGraspLiftSpec.HandSurfaceContactPenalty(2f, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.HandSurfaceContactPenalty(1f, true),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.HandSurfaceContactPenalty(0f, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.HandSurfaceContactPenalty(-1f, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.HandSurfaceContactPenalty(float.NaN, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.HandSurfaceContactPenalty(
                    float.PositiveInfinity,
                    false),
                1e-6f);
        }

        [Test]
        public void HandSurfacePenaltyScaleClampsRejectsNonFiniteAndCanBeDisabled()
        {
            Dg5fGraspLiftSpec.SetHandSurfacePenaltyPerSecond(-10f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MinimumHandSurfacePenaltyPerSecond,
                Dg5fGraspLiftSpec.CurrentHandSurfacePenaltyPerSecond,
                1e-6f);
            Dg5fGraspLiftSpec.SetHandSurfacePenaltyPerSecond(-2f);
            Assert.AreEqual(
                -2f,
                Dg5fGraspLiftSpec.CurrentHandSurfacePenaltyPerSecond,
                1e-6f,
                "the widened range must accept a value rejected by the old -1 clamp");
            Dg5fGraspLiftSpec.SetHandSurfacePenaltyPerSecond(1f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MaximumHandSurfacePenaltyPerSecond,
                Dg5fGraspLiftSpec.CurrentHandSurfacePenaltyPerSecond,
                1e-6f);
            Dg5fGraspLiftSpec.SetHandSurfacePenaltyPerSecond(0f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.HandSurfaceContactPenalty(1f, false),
                1e-6f);
            Dg5fGraspLiftSpec.SetHandSurfacePenaltyPerSecond(float.NegativeInfinity);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.HandSurfacePenaltyPerSecond,
                Dg5fGraspLiftSpec.CurrentHandSurfacePenaltyPerSecond,
                1e-6f);
        }

        [Test]
        public void GraspPosturePenaltyOnlyChargesBadUnconfirmedPoseAtTheBlock()
        {
            const float scale = -0.8f;
            Dg5fGraspLiftSpec.SetGraspPosturePenaltyScale(scale);
            float near = Dg5fGraspLiftSpec.GraspReadyDistance;
            float acceptable = Dg5fGraspLiftSpec.MaximumTopDownAngleDegrees;
            float halfway = (acceptable + 90f) * 0.5f;

            Assert.AreEqual(
                0f, Dg5fGraspLiftSpec.GraspPosturePenalty(90f, near, true), 1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.GraspPosturePenalty(90f, near + 0.001f, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.GraspPosturePenalty(acceptable, near, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.GraspPosturePenalty(acceptable - 10f, near, false),
                1e-6f);
            Assert.AreEqual(
                scale,
                Dg5fGraspLiftSpec.GraspPosturePenalty(90f, near, false),
                1e-6f);
            Assert.AreEqual(
                scale * 0.5f,
                Dg5fGraspLiftSpec.GraspPosturePenalty(halfway, near, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.GraspPosturePenalty(float.NaN, near, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.GraspPosturePenalty(
                    90f, float.PositiveInfinity, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.GraspPosturePenalty(
                    float.NegativeInfinity, near, false),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.GraspPosturePenalty(90f, float.NaN, false),
                1e-6f);
        }

        [Test]
        public void DefaultGraspPosturePenaltyIsInertAtEveryAngle()
        {
            Dg5fGraspLiftSpec.SetGraspPosturePenaltyScale(
                Dg5fGraspLiftSpec.GraspPosturePenaltyScale);
            for (int angle = 0; angle <= 180; angle += 5)
            {
                Assert.AreEqual(
                    0f,
                    Dg5fGraspLiftSpec.GraspPosturePenalty(
                        angle, Dg5fGraspLiftSpec.GraspReadyDistance, false),
                    1e-6f);
            }
        }

        [Test]
        public void GraspPosturePenaltyScaleClampsAndRejectsNonFinite()
        {
            Dg5fGraspLiftSpec.SetGraspPosturePenaltyScale(-2f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MinimumGraspPosturePenaltyScale,
                Dg5fGraspLiftSpec.CurrentGraspPosturePenaltyScale,
                1e-6f);
            Dg5fGraspLiftSpec.SetGraspPosturePenaltyScale(1f);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.MaximumGraspPosturePenaltyScale,
                Dg5fGraspLiftSpec.CurrentGraspPosturePenaltyScale,
                1e-6f);
            Dg5fGraspLiftSpec.SetGraspPosturePenaltyScale(-0.5f);
            Dg5fGraspLiftSpec.SetGraspPosturePenaltyScale(float.NegativeInfinity);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.GraspPosturePenaltyScale,
                Dg5fGraspLiftSpec.CurrentGraspPosturePenaltyScale,
                1e-6f);
        }

        [Test]
        public void ClosedEmptyHandAscentIsFlaggedOnlyWithoutAGrasp()
        {
            float high = Dg5fGraspLiftSpec.ClosedHandAscentHeight + 0.05f;
            Assert.IsTrue(Dg5fGraspLiftSpec.IsClosedHandAscent(high, 0.9f, false));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsClosedHandAscent(high, 0.9f, true),
                "carrying a grasped block upward is the goal, not a violation");
            Assert.IsFalse(Dg5fGraspLiftSpec.IsClosedHandAscent(high, 0.05f, false),
                "an open hand moving up is just navigation");
            Assert.IsFalse(Dg5fGraspLiftSpec.IsClosedHandAscent(0.01f, 0.9f, false));
        }

        [Test]
        public void ShovingTheBlockAcrossThePanelEndsTheAttempt()
        {
            Vector3 spawn = new Vector3(0.4f, 0.05f, 0.2f);
            Vector3 shoved = spawn + new Vector3(0.3f, 0f, 0f);
            Assert.IsTrue(Dg5fGraspLiftSpec.IsPushedAway(shoved, spawn, false));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsPushedAway(shoved, spawn, true),
                "once grasped the block is supposed to move with the hand");
            Assert.IsFalse(Dg5fGraspLiftSpec.IsPushedAway(spawn, spawn, false));
        }

        [Test]
        public void ObjectTiltMeasuresAlignmentAndRejectsInvalidVectors()
        {
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.ObjectTiltDegrees(Vector3.up, Vector3.up),
                1e-3f);
            Assert.AreEqual(
                90f,
                Dg5fGraspLiftSpec.ObjectTiltDegrees(Vector3.right, Vector3.up),
                1e-3f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.ObjectTiltDegrees(Vector3.zero, Vector3.up),
                1e-6f);
            Assert.AreEqual(
                0f,
                Dg5fGraspLiftSpec.ObjectTiltDegrees(
                    new Vector3(float.NaN, 0f, 0f),
                    Vector3.up),
                1e-6f);
        }

        [Test]
        public void ConfirmedGraspMayReorientPastTheToppleLimit()
        {
            Assert.IsFalse(Dg5fGraspLiftSpec.IsToppled(180f, true));
        }

        [Test]
        public void ObjectTopplesAtOrAboveTheConfiguredLimit()
        {
            Dg5fGraspLiftSpec.SetToppleLimit(45f);
            Assert.IsFalse(Dg5fGraspLiftSpec.IsToppled(44f, false));
            Assert.IsTrue(Dg5fGraspLiftSpec.IsToppled(45f, false));
            Assert.IsTrue(Dg5fGraspLiftSpec.IsToppled(90f, false));
        }

        [Test]
        public void ToppleLimitIsClampedAndCanDisableTheRule()
        {
            Dg5fGraspLiftSpec.SetToppleLimit(0f);
            Assert.AreEqual(5f, Dg5fGraspLiftSpec.CurrentToppleLimitDegrees, 1e-6f);
            Dg5fGraspLiftSpec.SetToppleLimit(200f);
            Assert.AreEqual(180f, Dg5fGraspLiftSpec.CurrentToppleLimitDegrees, 1e-6f);
            Assert.IsFalse(Dg5fGraspLiftSpec.IsToppled(90f, false));
            Dg5fGraspLiftSpec.SetToppleLimit(float.PositiveInfinity);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.ToppleLimitDegrees,
                Dg5fGraspLiftSpec.CurrentToppleLimitDegrees,
                1e-6f);
        }

        [Test]
        public void OutOfBoundsCoversFallingOffAndFlyingAway()
        {
            float half = Dg5fGraspLiftSpec.BlockHalfHeight;
            Assert.IsFalse(Dg5fGraspLiftSpec.IsOutOfBounds(
                new Vector3(0.4f, half, 0.2f), 0f, half));
            Assert.IsTrue(Dg5fGraspLiftSpec.IsOutOfBounds(
                new Vector3(0.4f, -0.5f, 0.2f), 0f, half), "fell off the panel");
            Assert.IsTrue(Dg5fGraspLiftSpec.IsOutOfBounds(
                new Vector3(2f, half, 0f), 0f, half), "thrown out of the workspace");
            Assert.IsTrue(Dg5fGraspLiftSpec.IsOutOfBounds(
                new Vector3(float.NaN, 0f, 0f), 0f, half));
        }

        [Test]
        public void FailurePenaltiesAreOrderedBySeverity()
        {
            Assert.AreEqual(
                Dg5fGraspLiftSpec.UnsafeSurfacePenalty,
                Dg5fGraspLiftSpec.FailurePenalty("UnsafeSurfaceContact"));
            Assert.AreEqual(
                Dg5fGraspLiftSpec.DropPenalty, Dg5fGraspLiftSpec.FailurePenalty("Dropped"));
            Assert.AreEqual(
                Dg5fGraspLiftSpec.PushAwayPenalty,
                Dg5fGraspLiftSpec.FailurePenalty("ObjectPushedAway"));
            Assert.AreEqual(
                Dg5fGraspLiftSpec.TopplePenalty,
                Dg5fGraspLiftSpec.FailurePenalty("ObjectToppled"));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.FailurePenalty("Timeout"));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.FailurePenalty("NonFinitePhysics"));
            Assert.Less(
                Dg5fGraspLiftSpec.FailurePenalty("UnsafeSurfaceContact"),
                Dg5fGraspLiftSpec.FailurePenalty("Dropped"));
        }

        [Test]
        public void EpisodeTimesOutAtTheContractedLimit()
        {
            Assert.IsFalse(Dg5fGraspLiftSpec.ReachedEpisodeTimeout(
                Dg5fGraspLiftSpec.EpisodeTimeoutSeconds - 1f));
            Assert.IsTrue(Dg5fGraspLiftSpec.ReachedEpisodeTimeout(
                Dg5fGraspLiftSpec.EpisodeTimeoutSeconds));
        }

        // --- spawn --------------------------------------------------------------

        [Test]
        public void EverySampledSpawnIsValidAtEveryStage()
        {
            for (int stage = Dg5fGraspLiftSpec.FirstGraspStage;
                 stage <= Dg5fGraspLiftSpec.FinalGraspStage;
                 stage++)
            {
                Dg5fGraspLiftSpec.SetGraspStage(stage);
                for (int i = 0; i <= 40; i++)
                {
                    float u = i / 40f;
                    Vector3 spawn = Dg5fGraspLiftSpec.SpawnBlockLocalPosition(
                        u, u, 1f - u, Dg5fGraspLiftSpec.BlockHeight);
                    Assert.IsTrue(
                        Dg5fGraspLiftSpec.IsValidSpawn(
                            spawn,
                            Dg5fGraspLiftSpec.BlockWidth,
                            Dg5fGraspLiftSpec.BlockHeight),
                        $"stage {stage} sample {u} produced an invalid spawn {spawn}");
                }
            }
        }

        [Test]
        public void TallerBlockSpawnStillValidates()
        {
            Dg5fGraspLiftSpec.SetBlockHeight(0.12f);
            Vector3 spawn = Dg5fGraspLiftSpec.SpawnBlockLocalPosition(
                0.5f, 0.1f, 0.3f, Dg5fGraspLiftSpec.CurrentBlockHeight);
            Assert.IsTrue(Dg5fGraspLiftSpec.IsValidSpawn(
                spawn,
                Dg5fGraspLiftSpec.CurrentBlockWidth,
                Dg5fGraspLiftSpec.CurrentBlockHeight));
            Assert.AreEqual(
                Dg5fGraspLiftSpec.SupportTopHeight
                    + Dg5fGraspLiftSpec.CurrentBlockHalfHeight,
                spawn.y,
                1e-5f);
        }

        [Test]
        public void SpawnsRestOnThePanelTop()
        {
            Vector3 spawn = Dg5fGraspLiftSpec.SpawnBlockLocalPosition(
                0.5f, 0.1f, 0.3f, Dg5fGraspLiftSpec.BlockHeight);
            Assert.AreEqual(
                Dg5fGraspLiftSpec.SupportTopHeight + Dg5fGraspLiftSpec.BlockHalfHeight,
                spawn.y,
                1e-5f);
        }

        [Test]
        public void SpawnRadiusStaysInsideTheCurrentStageAnnulus()
        {
            Dg5fGraspLiftSpec.SetGraspStage(1);
            for (int i = 0; i <= 20; i++)
            {
                float radius = Dg5fGraspLiftSpec.AreaUniformRadius(i / 20f);
                Assert.GreaterOrEqual(radius, Dg5fGraspLiftSpec.CurrentMinimumSpawnRadius - 1e-5f);
                Assert.LessOrEqual(radius, Dg5fGraspLiftSpec.CurrentMaximumSpawnRadius + 1e-5f);
            }
        }

        // --- numeric hygiene -----------------------------------------------------

        [Test]
        public void NonFiniteInputsNeverProduceNonFiniteRewards()
        {
            float nan = float.NaN;
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.PotentialDelta(nan, 1f));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.NewBestPotentialDelta(1f, nan));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.NearObjectActionPenalty(nan));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.LiftProgress(nan));
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.GraspProgress(nan));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsGraspConfirmed(nan));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsLiftComplete(nan));
            Assert.IsFalse(Dg5fGraspLiftSpec.IsTopDownAligned(nan));
        }

        [Test]
        public void NormalizeJointMapsLimitsToMinusOneAndOne()
        {
            Assert.AreEqual(-1f, Dg5fGraspLiftSpec.NormalizeJoint(-90f, -90f, 90f), 1e-5f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.NormalizeJoint(0f, -90f, 90f), 1e-5f);
            Assert.AreEqual(1f, Dg5fGraspLiftSpec.NormalizeJoint(90f, -90f, 90f), 1e-5f);
            Assert.AreEqual(0f, Dg5fGraspLiftSpec.NormalizeJoint(5f, 10f, 10f), 1e-5f);
        }
    }
}
