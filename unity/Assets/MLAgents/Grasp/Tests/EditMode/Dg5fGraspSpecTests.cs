using NUnit.Framework;
using UnityEngine;

namespace KDT.GraspTraining.Tests
{
    public sealed class Dg5fGraspSpecTests
    {
        [Test]
        public void VersionTwoKeepsObservationActionAndClosureContracts()
        {
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("2.1.0"));
            Assert.That(Dg5fGraspSpec.ObservationSize, Is.EqualTo(43));
            Assert.That(Dg5fGraspSpec.ActionSize, Is.EqualTo(7));
            Assert.That(Dg5fGraspSpec.LeftFistDeg, Has.Length.EqualTo(20));
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
        public void OnlyShoulderPanSafeRangeExpands()
        {
            float[] expectedMinimum = { -180f, -120f, 20f, -180f, -150f, -180f };
            float[] expectedMaximum = { 180f, -20f, 140f, 0f, -30f, 180f };
            Assert.That(Dg5fGraspSpec.ArmSafeMinDeg, Is.EqualTo(expectedMinimum));
            Assert.That(Dg5fGraspSpec.ArmSafeMaxDeg, Is.EqualTo(expectedMaximum));
        }

        [Test]
        public void ShoulderPanNormalizationUsesExpandedSafeRange()
        {
            Assert.That(Dg5fGraspSpec.NormalizeJoint(-180f, -180f, 180f), Is.EqualTo(-1f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(0f, -180f, 180f), Is.EqualTo(0f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(180f, -180f, 180f), Is.EqualTo(1f));
        }

        [TestCase(0f, 0f)]
        [TestCase(0.425f, -0.005f)]
        [TestCase(0.85f, -0.01f)]
        [TestCase(1.2f, -0.01f)]
        public void DistanceRewardMatchesVersionTwoBoundaries(float distance, float expected)
        {
            Assert.That(Dg5fGraspSpec.DistanceReward(distance), Is.EqualTo(expected).Within(1e-7f));
        }

        [Test]
        public void SpawnRadiusIsUniformInAreaRatherThanRadius()
        {
            Assert.That(Dg5fGraspSpec.AreaUniformRadius(0f), Is.EqualTo(0.25f).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.AreaUniformRadius(1f), Is.EqualTo(0.70f).Within(1e-6f));

            float midpoint = Dg5fGraspSpec.AreaUniformRadius(0.5f);
            float expectedSquared = (0.25f * 0.25f + 0.70f * 0.70f) * 0.5f;
            Assert.That(midpoint * midpoint, Is.EqualTo(expectedSquared).Within(1e-6f));
        }

        [TestCase(0.07f, 0.02f, true)]
        [TestCase(0.0699f, 0.02f, false)]
        [TestCase(0f, 0.02f, false)]
        [TestCase(float.NaN, 0.02f, false)]
        [TestCase(0.07f, float.PositiveInfinity, false)]
        public void RobotSpawnClearanceUsesBallSurfaceGap(
            float centerToRobotSurfaceDistance,
            float ballRadius,
            bool expected)
        {
            Assert.That(
                Dg5fGraspSpec.HasMinimumRobotSpawnClearance(
                    centerToRobotSurfaceDistance,
                    ballRadius),
                Is.EqualTo(expected));
        }

        [Test]
        public void ContactMustRemainContinuousForOneSimulationSecond()
        {
            float held = Dg5fGraspSpec.UpdateHoldSeconds(0f, true, 0.98f);
            Assert.That(Dg5fGraspSpec.ReachedDuration(held, Dg5fGraspSpec.ContactSuccessSeconds), Is.False);

            held = Dg5fGraspSpec.UpdateHoldSeconds(held, false, 0.01f);
            Assert.That(held, Is.Zero, "Any contact break must clear the continuous hold.");

            held = Dg5fGraspSpec.UpdateHoldSeconds(0.98f, true, 0.02f);
            Assert.That(Dg5fGraspSpec.ReachedDuration(held, Dg5fGraspSpec.ContactSuccessSeconds), Is.True);
        }

        [Test]
        public void BallResetCoversWorkspaceDropAndNonFiniteState()
        {
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(0.85f, 0f, 0f), -1f), Is.False);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(0.851f, 0f, 0f), -1f), Is.True);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(0.2f, 0.29f, 0f), 0.30f), Is.True);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(float.NaN, 0.3f, 0f), 0.25f), Is.True);
            Assert.That(Dg5fGraspSpec.ShouldResetForBall(new Vector3(float.PositiveInfinity, 0.3f, 0f), 0.25f), Is.True);
        }

        [Test]
        public void PenetrationRequiresBothDepthAndDurationThresholds()
        {
            float held = 0f;
            held = Dg5fGraspSpec.UpdateHoldSeconds(
                held,
                0.0099f >= Dg5fGraspSpec.PenetrationDepthMeters,
                1f);
            Assert.That(held, Is.Zero);

            held = Dg5fGraspSpec.UpdateHoldSeconds(
                held,
                0.01f >= Dg5fGraspSpec.PenetrationDepthMeters,
                0.18f);
            Assert.That(Dg5fGraspSpec.ReachedDuration(held, Dg5fGraspSpec.PenetrationFailureSeconds), Is.False);

            held = Dg5fGraspSpec.UpdateHoldSeconds(
                held,
                0.01f >= Dg5fGraspSpec.PenetrationDepthMeters,
                0.02f);
            Assert.That(Dg5fGraspSpec.ReachedDuration(held, Dg5fGraspSpec.PenetrationFailureSeconds), Is.True);
        }

        [Test]
        public void StagnationResetsOnlyAfterTwoCentimetersOfProgress()
        {
            float best = 0.5f;
            float meaningful = best;
            float stagnant = 0f;

            bool timedOut = Dg5fGraspSpec.UpdateStagnation(
                0.5f, 59.9f, ref best, ref meaningful, ref stagnant);
            Assert.That(timedOut, Is.False);
            timedOut = Dg5fGraspSpec.UpdateStagnation(
                0.5f, 0.1f, ref best, ref meaningful, ref stagnant);
            Assert.That(timedOut, Is.True);

            best = meaningful = 0.5f;
            stagnant = 30f;
            timedOut = Dg5fGraspSpec.UpdateStagnation(
                0.481f, 1f, ref best, ref meaningful, ref stagnant);
            Assert.That(stagnant, Is.EqualTo(31f), "Less than 2 cm must not reset stagnation.");
            timedOut = Dg5fGraspSpec.UpdateStagnation(
                0.48f, 1f, ref best, ref meaningful, ref stagnant);
            Assert.That(timedOut, Is.False);
            Assert.That(stagnant, Is.Zero, "A full 2 cm improvement restarts the timer.");
        }

        [Test]
        public void SafeRangesContainBundledRobotStartPose()
        {
            float[] initial = { 0f, -60f, 60f, -90f, -90f, 0f };
            for (int i = 0; i < initial.Length; i++)
                Assert.That(initial[i], Is.InRange(Dg5fGraspSpec.ArmSafeMinDeg[i], Dg5fGraspSpec.ArmSafeMaxDeg[i]));
        }
    }
}
