using NUnit.Framework;
using UnityEngine;

namespace KDT.GraspTraining.Tests
{
    public sealed class Dg5fGraspSpecTests
    {
        static readonly Dg5fCurriculumStage FinalStage = Dg5fGraspSpec.GetCurriculumStage(4);

        [Test]
        public void VersionFourCreatesANewObservationAndBehaviorContract()
        {
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("4.1.0"));
            Assert.That(Dg5fGraspSpec.BehaviorName, Is.EqualTo("DG5FGraspV4"));
            Assert.That(Dg5fGraspSpec.ObservationSize, Is.EqualTo(57));
            Assert.That(Dg5fGraspSpec.ActionSize, Is.EqualTo(7));
            Assert.That(Dg5fGraspSpec.LeftFistDeg, Has.Length.EqualTo(20));
        }

        [Test]
        public void CurriculumStagesMatchTheFivePlannedTasks()
        {
            Assert.That(Dg5fGraspSpec.CurriculumStages, Has.Length.EqualTo(5));

            float[] halfAngles = { 15f, 30f, 60f, 120f, 180f };
            float[] maximumRadii = { 0.35f, 0.45f, 0.55f, 0.65f, 0.70f };
            float[] lifts = { 0.02f, 0.05f, 0.10f, 0.10f, 0.10f };
            float[] holds = { 0.5f, 1f, 2f, 3f, 5f };
            float[] lambdas = { 0f, 0f, 0.01f, 0.01f, 0.02f };

            for (int i = 0; i < Dg5fGraspSpec.CurriculumStages.Length; i++)
            {
                Dg5fCurriculumStage stage = Dg5fGraspSpec.GetCurriculumStage(i);
                Assert.That(stage.Index, Is.EqualTo(i));
                Assert.That(stage.HalfAngleDegrees, Is.EqualTo(halfAngles[i]));
                Assert.That(stage.MinimumSpawnRadius, Is.EqualTo(0.25f));
                Assert.That(stage.MaximumSpawnRadius, Is.EqualTo(maximumRadii[i]));
                Assert.That(stage.LiftTargetMeters, Is.EqualTo(lifts[i]));
                Assert.That(stage.HoldTargetSeconds, Is.EqualTo(holds[i]));
                Assert.That(stage.MovementPenaltyLambda, Is.EqualTo(lambdas[i]));
            }

            Assert.That(Dg5fGraspSpec.GetCurriculumStage(-1).Index, Is.Zero);
            Assert.That(Dg5fGraspSpec.GetCurriculumStage(99).Index, Is.EqualTo(4));
        }

        [Test]
        public void GripProfileInterpolatesAndClampsClosure()
        {
            Assert.That(Dg5fGraspSpec.GripTargetDeg(0, 0f), Is.EqualTo(0f));
            Assert.That(Dg5fGraspSpec.GripTargetDeg(0, 0.5f), Is.EqualTo(-20f));
            Assert.That(Dg5fGraspSpec.GripTargetDeg(5, 1f), Is.EqualTo(100f));
            Assert.That(Dg5fGraspSpec.GripTargetDeg(5, 2f), Is.EqualTo(100f));
        }

        [Test]
        public void ArmSafeRangesAndNormalizationRemainStable()
        {
            float[] expectedMinimum = { -180f, -120f, 20f, -180f, -150f, -180f };
            float[] expectedMaximum = { 180f, -20f, 140f, 0f, -30f, 180f };
            Assert.That(Dg5fGraspSpec.ArmSafeMinDeg, Is.EqualTo(expectedMinimum));
            Assert.That(Dg5fGraspSpec.ArmSafeMaxDeg, Is.EqualTo(expectedMaximum));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(-180f, -180f, 180f), Is.EqualTo(-1f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(0f, -180f, 180f), Is.EqualTo(0f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(180f, -180f, 180f), Is.EqualTo(1f));
        }

        [Test]
        public void PotentialRewardsAreBoundedAndReverseOnRetreatOrDescent()
        {
            Assert.That(Dg5fGraspSpec.ReachPotential(Dg5fGraspSpec.MaximumBallDistance), Is.Zero);
            Assert.That(Dg5fGraspSpec.ReachPotential(0f), Is.EqualTo(1f));
            Assert.That(Dg5fGraspSpec.ContactPotential(false, false), Is.Zero);
            Assert.That(Dg5fGraspSpec.ContactPotential(true, false), Is.EqualTo(0.25f));
            Assert.That(Dg5fGraspSpec.ContactPotential(false, true), Is.EqualTo(0.25f));
            Assert.That(Dg5fGraspSpec.ContactPotential(true, true), Is.EqualTo(0.5f));
            Assert.That(Dg5fGraspSpec.LiftPotential(0f, 0.10f), Is.Zero);
            Assert.That(Dg5fGraspSpec.LiftPotential(0.05f, 0.10f), Is.EqualTo(0.5f));
            Assert.That(Dg5fGraspSpec.LiftPotential(0.10f, 0.10f), Is.EqualTo(1f));
            Assert.That(Dg5fGraspSpec.LiftPotential(0.20f, 0.10f), Is.EqualTo(1f));
            Assert.That(Dg5fGraspSpec.HoldPotential(0.25f, 0.5f), Is.EqualTo(0.5f));
            Assert.That(Dg5fGraspSpec.HoldPotential(5f, 5f), Is.EqualTo(1f));

            float approaching = Dg5fGraspSpec.PotentialDelta(
                Dg5fGraspSpec.ReachPotential(0.6f),
                Dg5fGraspSpec.ReachPotential(0.4f));
            float retreating = Dg5fGraspSpec.PotentialDelta(
                Dg5fGraspSpec.ReachPotential(0.4f),
                Dg5fGraspSpec.ReachPotential(0.6f));
            float descending = Dg5fGraspSpec.PotentialDelta(
                Dg5fGraspSpec.LiftPotential(0.10f, 0.10f),
                Dg5fGraspSpec.LiftPotential(0.05f, 0.10f));
            float brokenHold = Dg5fGraspSpec.PotentialDelta(
                Dg5fGraspSpec.HoldPotential(2f, 5f),
                Dg5fGraspSpec.HoldPotential(0f, 5f));
            Assert.That(approaching, Is.GreaterThan(0f));
            Assert.That(retreating, Is.EqualTo(-approaching).Within(1e-6f));
            Assert.That(descending, Is.EqualTo(-0.5f));
            Assert.That(brokenHold, Is.EqualTo(-0.4f).Within(1e-6f));
        }

        [Test]
        public void NormalizedArmMovementUsesEachJointRange()
        {
            float[] before = { 0f, -100f, 20f, -180f, -150f, -180f };
            float[] after = { 36f, -90f, 32f, -162f, -138f, -144f };

            float movement = Dg5fGraspSpec.NormalizedArmMovement(before, after);

            Assert.That(movement, Is.EqualTo(0.6f).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.MovementPenalty(movement, 0f), Is.Zero);
            Assert.That(Dg5fGraspSpec.MovementPenalty(movement, 0.02f),
                Is.EqualTo(-0.012f).Within(1e-6f));
        }

        [Test]
        public void CurriculumSpawnUsesAreaUniformRadiusAndGraspPointDirection()
        {
            Dg5fCurriculumStage stage = Dg5fGraspSpec.GetCurriculumStage(0);
            const float ballRadius = 0.02f;
            Vector3 left = Dg5fGraspSpec.SpawnBallLocalPosition(0f, 0f, ballRadius, stage, 90f);
            Vector3 right = Dg5fGraspSpec.SpawnBallLocalPosition(1f, 1f, ballRadius, stage, 90f);

            Assert.That(new Vector2(left.x, left.z).magnitude, Is.EqualTo(0.25f).Within(1e-6f));
            Assert.That(new Vector2(right.x, right.z).magnitude, Is.EqualTo(0.35f).Within(1e-6f));
            Assert.That(Mathf.Atan2(left.z, left.x) * Mathf.Rad2Deg, Is.EqualTo(75f).Within(1e-4f));
            Assert.That(Mathf.Atan2(right.z, right.x) * Mathf.Rad2Deg, Is.EqualTo(105f).Within(1e-4f));
            Assert.That(left.y, Is.EqualTo(Dg5fGraspSpec.SupportTopHeight + ballRadius));
        }

        [Test]
        public void StableGraspRequiresContinuousQuarterSecond()
        {
            var state = NewState();

            Dg5fEpisodeStepResult result = state.Advance(Input(true), FinalStage, 0.24f);
            Assert.That(result.StableGraspReached, Is.False);
            Assert.That(state.Phase, Is.EqualTo(Dg5fGraspPhase.Grasp));

            state.Advance(Input(false), FinalStage, 0.01f);
            Assert.That(state.Phase, Is.EqualTo(Dg5fGraspPhase.Reach));
            Assert.That(state.GraspSeconds, Is.Zero);

            state.Advance(Input(true), FinalStage, 0.23f);
            result = state.Advance(Input(true), FinalStage, 0.02f);
            Assert.That(result.StableGraspReached, Is.True);
            Assert.That(state.Phase, Is.EqualTo(Dg5fGraspPhase.Lift));
            Assert.That(state.GraspSeconds, Is.EqualTo(0.25f).Within(1e-6f));
        }

        [Test]
        public void LiftTransitionsAtExactTenCentimeterBoundary()
        {
            var state = StableGraspState();

            Dg5fEpisodeStepResult result = state.Advance(Input(true, 0.099f), FinalStage, 0.02f);
            Assert.That(result.LiftTargetReached, Is.False);
            Assert.That(state.Phase, Is.EqualTo(Dg5fGraspPhase.Lift));

            result = state.Advance(Input(true, 0.10f), FinalStage, 0.02f);
            Assert.That(result.LiftTargetReached, Is.True);
            Assert.That(state.LiftTargetWasReached, Is.True);
            Assert.That(state.Phase, Is.EqualTo(Dg5fGraspPhase.Hold));
        }

        [Test]
        public void HoldHeightOrSpeedBreakResetsTimer()
        {
            var state = HoldState();
            state.Advance(Input(true, 0.10f, 0.01f), FinalStage, 1f);
            Assert.That(state.HoldSeconds, Is.EqualTo(1f));

            Dg5fEpisodeStepResult result = state.Advance(Input(true, 0.08f, 0.01f), FinalStage, 0.1f);
            Assert.That(result.Failed, Is.False);
            Assert.That(state.HoldSeconds, Is.Zero);

            state.Advance(Input(true, 0.10f, 0.01f), FinalStage, 0.1f);
            Assert.That(state.HoldSeconds, Is.EqualTo(0.1f));

            state.Advance(Input(true, 0.10f, 0.051f), FinalStage, 0.1f);
            Assert.That(state.HoldSeconds, Is.Zero, "Excess ball speed must reset the hold.");
        }

        [Test]
        public void HoldSucceedsAtExactlyFiveContinuousSeconds()
        {
            var state = HoldState();

            Dg5fEpisodeStepResult result = state.Advance(Input(true, 0.09f, 0.05f), FinalStage, 4.98f);
            Assert.That(result.Succeeded, Is.False);
            Assert.That(state.HoldSeconds, Is.EqualTo(4.98f).Within(1e-6f));

            result = state.Advance(Input(true, 0.09f, 0.05f), FinalStage, 0.02f);
            Assert.That(result.Succeeded, Is.True);
            Assert.That(state.IsTerminal, Is.True);
            Assert.That(state.Succeeded, Is.True);
            Assert.That(state.HoldSeconds, Is.EqualTo(5f).Within(1e-6f));
        }

        [Test]
        public void GripLossFailsImmediatelyAfterStableGrasp()
        {
            var state = HoldState();
            Dg5fEpisodeStepResult result = state.Advance(Input(false, 0.10f), FinalStage, 0.02f);

            Assert.That(result.Failed, Is.True);
            Assert.That(result.FailureReason, Is.EqualTo(Dg5fFailureReason.GripLost));
        }

        [Test]
        public void DropFailureIsArmedOnlyAfterTheLiftTargetWasReached()
        {
            var lifting = StableGraspState();
            Assert.That(lifting.Advance(Input(true, 0.01f), FinalStage, 0.02f).Failed, Is.False);

            var held = HoldState();
            Dg5fEpisodeStepResult result = held.Advance(Input(true, 0.02f), FinalStage, 0.02f);

            Assert.That(result.Failed, Is.True);
            Assert.That(result.FailureReason, Is.EqualTo(Dg5fFailureReason.Dropped));
        }

        [Test]
        public void EpisodeTimesOutAtExactlyTwentySecondsWithFailureReward()
        {
            var state = NewState();
            Assert.That(state.Advance(Input(false), FinalStage, 19.98f).Failed, Is.False);

            Dg5fEpisodeStepResult result = state.Advance(Input(false), FinalStage, 0.02f);

            Assert.That(result.Failed, Is.True);
            Assert.That(result.FailureReason, Is.EqualTo(Dg5fFailureReason.Timeout));
            Assert.That(
                Dg5fGraspSpec.TerminalReward(false, Dg5fFailureReason.Timeout),
                Is.EqualTo(-0.1f));
            Assert.That(
                Dg5fGraspSpec.TerminalReward(false, Dg5fFailureReason.GripLost),
                Is.EqualTo(-0.5f));
            Assert.That(
                Dg5fGraspSpec.TerminalReward(false, Dg5fFailureReason.Dropped),
                Is.EqualTo(-0.5f));
            Assert.That(
                Dg5fGraspSpec.TerminalReward(false, Dg5fFailureReason.Penetration),
                Is.EqualTo(-1f));
            Assert.That(
                Dg5fGraspSpec.TerminalReward(true, Dg5fFailureReason.None),
                Is.EqualTo(3f));
        }

        [Test]
        public void PenetrationAndNonFinitePhysicsHaveExplicitFailureReasons()
        {
            var penetration = NewState();
            Assert.That(penetration.Advance(Input(false, penetration: true), FinalStage, 0.18f).Failed, Is.False);
            Assert.That(
                penetration.Advance(Input(false, penetration: true), FinalStage, 0.02f).FailureReason,
                Is.EqualTo(Dg5fFailureReason.Penetration));

            var nonFinite = NewState();
            Assert.That(
                nonFinite.Advance(Input(false, float.NaN), FinalStage, 0.02f).FailureReason,
                Is.EqualTo(Dg5fFailureReason.NonFinitePhysics));
        }

        [Test]
        public void SpawnAndBallSafetyHelpersRetainTheirBoundaryContracts()
        {
            const float ballRadius = 0.02f;
            float height = Dg5fGraspSpec.SupportTopHeight + ballRadius;
            Assert.That(Dg5fGraspSpec.AreaUniformRadius(0f), Is.EqualTo(0.25f).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.AreaUniformRadius(1f), Is.EqualTo(0.70f).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.IsValidSpawn(new Vector3(0.25f, height, 0f), ballRadius), Is.True);
            Assert.That(Dg5fGraspSpec.HasMinimumRobotSpawnClearance(0.07f, ballRadius), Is.True);
            Assert.That(Dg5fGraspSpec.HasMinimumRobotSpawnClearance(0.0699f, ballRadius), Is.False);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(0.85f, 0f, 0f), -1f), Is.False);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(0.851f, 0f, 0f), -1f), Is.True);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(float.NaN, 0f, 0f), -1f), Is.True);
        }

        static Dg5fEpisodeState NewState()
        {
            var state = new Dg5fEpisodeState();
            state.Reset();
            return state;
        }

        static Dg5fEpisodeState StableGraspState()
        {
            Dg5fEpisodeState state = NewState();
            Dg5fEpisodeStepResult result = state.Advance(Input(true), FinalStage, 0.25f);
            Assert.That(result.StableGraspReached, Is.True);
            return state;
        }

        static Dg5fEpisodeState HoldState()
        {
            Dg5fEpisodeState state = StableGraspState();
            Dg5fEpisodeStepResult result = state.Advance(Input(true, 0.10f), FinalStage, 0.02f);
            Assert.That(result.LiftTargetReached, Is.True);
            return state;
        }

        static Dg5fEpisodeInput Input(
            bool graspContact,
            float liftMeters = 0f,
            float ballSpeed = 0f,
            bool workspaceValid = true,
            bool penetration = false,
            bool physicsFinite = true)
        {
            return new Dg5fEpisodeInput(
                graspContact,
                liftMeters,
                ballSpeed,
                workspaceValid,
                penetration,
                physicsFinite);
        }
    }
}
