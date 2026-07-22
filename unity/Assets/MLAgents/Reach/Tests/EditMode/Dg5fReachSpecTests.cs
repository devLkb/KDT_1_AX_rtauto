using System;
using System.Linq;
using NUnit.Framework;
using UnityEngine;

namespace KDT.ReachTraining.Tests
{
    public sealed class Dg5fReachSpecTests
    {
        [Test]
        public void PolicyContract_IsFreshAndContiguous()
        {
            Assert.That(Dg5fReachSpec.SpecVersion, Is.EqualTo("2.0.0"));
            Assert.That(Dg5fReachSpec.BehaviorName,
                Is.EqualTo("DG5FGraspReadyReach"));
            Assert.That(Dg5fReachSpec.ArmJointCount, Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.HandJointCount, Is.EqualTo(20));
            Assert.That(Dg5fReachSpec.ActionSize, Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.ObservationSize, Is.EqualTo(37));
            Assert.That(Dg5fReachSpec.DecisionPeriod, Is.EqualTo(5));

            Assert.That(Dg5fReachSpec.ArmPositionObservationOffset, Is.EqualTo(0));
            Assert.That(Dg5fReachSpec.ArmVelocityObservationOffset, Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.ArmTargetObservationOffset, Is.EqualTo(12));
            Assert.That(Dg5fReachSpec.TargetOffsetObservationOffset, Is.EqualTo(18));
            Assert.That(Dg5fReachSpec.ActiveWaypointOffsetObservationOffset,
                Is.EqualTo(21));
            Assert.That(Dg5fReachSpec.DistanceObservationIndex, Is.EqualTo(24));
            Assert.That(Dg5fReachSpec.PlanarDistanceObservationIndex, Is.EqualTo(25));
            Assert.That(Dg5fReachSpec.FloorClearanceObservationIndex, Is.EqualTo(26));
            Assert.That(Dg5fReachSpec.GraspPointVelocityObservationOffset,
                Is.EqualTo(27));
            Assert.That(Dg5fReachSpec.PalmTargetObservationOffset, Is.EqualTo(30));
            Assert.That(Dg5fReachSpec.PalmAlignmentObservationIndex, Is.EqualTo(33));
            Assert.That(Dg5fReachSpec.UpperConeObservationIndex, Is.EqualTo(34));
            Assert.That(Dg5fReachSpec.PhaseObservationIndex, Is.EqualTo(35));
            Assert.That(Dg5fReachSpec.HoldProgressObservationIndex, Is.EqualTo(36));
        }

        [Test]
        public void JointAndGraspPointContracts_AreCalibrated()
        {
            Assert.That(Dg5fReachSpec.ArmLinks, Has.Length.EqualTo(6));
            Assert.That(Dg5fReachSpec.ArmLinks.Distinct().Count(), Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.ArmSafeMinDeg, Has.Length.EqualTo(6));
            Assert.That(Dg5fReachSpec.ArmSafeMaxDeg, Has.Length.EqualTo(6));
            for (int index = 0; index < 6; index++)
                Assert.That(Dg5fReachSpec.ArmSafeMinDeg[index],
                    Is.LessThan(Dg5fReachSpec.ArmSafeMaxDeg[index]));
            Assert.That(Dg5fReachSpec.CalibratedGraspPointLocalPosition,
                Is.EqualTo(new Vector3(
                    0.0170203224f,
                    0.152462155f,
                    0.0135399457f)));
        }

        [Test]
        public void Spawn_IsDeterministicAndCoversConfiguredAnnulus()
        {
            var first = new System.Random(4312);
            var second = new System.Random(4312);
            Vector3 initial = new Vector3(0f, 2f, 0f);
            var quadrants = new int[4];
            for (int sample = 0; sample < 4000; sample++)
            {
                Vector3 a = Dg5fReachSpec.SpawnTargetLocalPosition(first, initial);
                Vector3 b = Dg5fReachSpec.SpawnTargetLocalPosition(second, initial);
                Assert.That(a, Is.EqualTo(b));
                Assert.That(Dg5fReachSpec.IsValidSpawn(a, initial), Is.True);
                int quadrant = a.x >= 0f
                    ? (a.z >= 0f ? 0 : 3)
                    : (a.z >= 0f ? 1 : 2);
                quadrants[quadrant]++;
            }
            Assert.That(quadrants, Has.All.GreaterThan(850));
        }

        [Test]
        public void Spawn_UsesBoundedScenePredicate()
        {
            int attempts = 0;
            Vector3 point = Dg5fReachSpec.SpawnTargetLocalPosition(
                new System.Random(72),
                new Vector3(0f, 2f, 0f),
                Dg5fReachSpec.TargetRadius,
                candidate => ++attempts == 4);
            Assert.That(attempts, Is.EqualTo(4));
            Assert.That(Dg5fReachSpec.IsValidSpawn(
                point,
                new Vector3(0f, 2f, 0f)), Is.True);

            Assert.Throws<InvalidOperationException>(() =>
                Dg5fReachSpec.SpawnTargetLocalPosition(
                    new System.Random(73),
                    new Vector3(0f, 2f, 0f),
                    Dg5fReachSpec.TargetRadius,
                    candidate => false));
        }

        [Test]
        public void PreGraspWaypoint_IsExactlyTenCentimetersAboveTarget()
        {
            Vector3 target = new Vector3(0.3f, 0.02f, -0.4f);
            Assert.That(Dg5fReachSpec.PreGraspPoint(target),
                Is.EqualTo(target + Vector3.up * 0.10f));
        }

        [Test]
        public void DescendGate_RequiresWaypointToleranceAndClearance()
        {
            Assert.That(Dg5fReachSpec.CanEnterDescend(0.03f, 0.10f), Is.True);
            Assert.That(Dg5fReachSpec.CanEnterDescend(0.03001f, 0.10f), Is.False);
            Assert.That(Dg5fReachSpec.CanEnterDescend(0.03f, 0.09999f), Is.False);
        }

        [Test]
        public void PrematureDescent_BlocksSweepingButAllowsVerticalFinalApproach()
        {
            Assert.That(Dg5fReachSpec.IsPrematureDescent(
                ReachPhase.Transit, 0f, 0.099f), Is.True);
            Assert.That(Dg5fReachSpec.IsPrematureDescent(
                ReachPhase.Descend, 0.051f, 0.099f), Is.True);
            Assert.That(Dg5fReachSpec.IsPrematureDescent(
                ReachPhase.Descend, 0.05f, 0.02f), Is.False);
            Assert.That(Dg5fReachSpec.IsPrematureDescent(
                ReachPhase.Transit, 0.5f, 0.10f), Is.False);
        }

        [Test]
        public void PalmAndUpperConeAlignment_UseExactAngularBoundaries()
        {
            Vector3 fifteenDegrees = Quaternion.AngleAxis(15f, Vector3.up)
                * Vector3.forward;
            float palmAlignment = Dg5fReachSpec.PalmFacingAlignment(
                Vector3.forward,
                fifteenDegrees);
            Assert.That(palmAlignment,
                Is.EqualTo(Dg5fReachSpec.MinimumPalmAlignment).Within(1e-6f));

            Vector3 palmAtFortyFive = Quaternion.AngleAxis(45f, Vector3.forward)
                * Vector3.up;
            float cone = Dg5fReachSpec.UpperConeAlignment(
                palmAtFortyFive,
                Vector3.zero);
            Assert.That(cone,
                Is.EqualTo(Dg5fReachSpec.MinimumUpperConeAlignment).Within(1e-6f));
        }

        [Test]
        public void LockState_RequiresDistanceSpeedOrientationAndUpperCone()
        {
            Assert.That(Dg5fReachSpec.MeetsLockState(
                0.01f,
                0.05f,
                Dg5fReachSpec.MinimumPalmAlignment,
                Dg5fReachSpec.MinimumUpperConeAlignment), Is.True);
            Assert.That(Dg5fReachSpec.MeetsLockState(
                0.01001f, 0.05f, 1f, 1f), Is.False);
            Assert.That(Dg5fReachSpec.MeetsLockState(
                0.01f, 0.05001f, 1f, 1f), Is.False);
            Assert.That(Dg5fReachSpec.MeetsLockState(
                0.01f, 0.05f,
                Dg5fReachSpec.MinimumPalmAlignment - 1e-5f, 1f), Is.False);
            Assert.That(Dg5fReachSpec.MeetsLockState(
                0.01f, 0.05f, 1f,
                Dg5fReachSpec.MinimumUpperConeAlignment - 1e-5f), Is.False);
        }

        [Test]
        public void LockHold_ResetsOnAnyInvalidFrame()
        {
            float hold = 0f;
            for (int step = 0; step < 13; step++)
                hold = Dg5fReachSpec.NextLockHoldSeconds(
                    hold, 0.01f, 0.05f, 1f, 1f, 0.02f);
            Assert.That(hold, Is.EqualTo(0.26f).Within(1e-6f));
            Assert.That(Dg5fReachSpec.HasCompletedLockHold(hold), Is.True);
            Assert.That(Dg5fReachSpec.LockHoldProgress(hold), Is.EqualTo(1f));
            Assert.That(Dg5fReachSpec.NextLockHoldSeconds(
                hold, 0.011f, 0.05f, 1f, 1f, 0.02f), Is.Zero);
        }

        [Test]
        public void Reward_IsPotentialBasedAndPhaseAware()
        {
            float approach = Dg5fReachSpec.DecisionReward(
                0.5f, 0.4f, ReachPhase.Transit, 0f, 1f);
            float retreat = Dg5fReachSpec.DecisionReward(
                0.4f, 0.5f, ReachPhase.Transit, 1f, 0f);
            Assert.That(approach + retreat,
                Is.EqualTo(2f * Dg5fReachSpec.DecisionTimePenalty).Within(1e-6f));

            float aligned = Dg5fReachSpec.DecisionReward(
                0.2f, 0.2f, ReachPhase.Descend, 0.5f, 0.7f);
            Assert.That(aligned,
                Is.EqualTo(Dg5fReachSpec.DecisionTimePenalty + 0.05f)
                    .Within(1e-6f));
            Assert.That(Dg5fReachSpec.FailurePenalty("Timeout"), Is.EqualTo(-1f));
            Assert.That(Dg5fReachSpec.FailurePenalty("PrematureDescent"),
                Is.EqualTo(-2f));
        }

        [Test]
        public void ActionsAndPointVelocity_AreBoundedAndFinite()
        {
            Assert.That(Dg5fReachSpec.AccumulateJointTarget(
                10f, 2f, 99f, -20f, 20f), Is.EqualTo(14f));
            Assert.That(Dg5fReachSpec.AccumulateJointTarget(
                19f, 1f, 4f, -20f, 20f), Is.EqualTo(20f));
            Assert.That(Dg5fReachSpec.ArmDeltaDeg(0.1f), Is.EqualTo(1f));
            Assert.That(Dg5fReachSpec.ArmDeltaDeg(0.1001f), Is.EqualTo(2f));

            Vector3 velocity = Dg5fReachSpec.PointVelocity(
                new Vector3(1f, 2f, 3f),
                new Vector3(0f, 0f, 2f),
                Vector3.zero,
                Vector3.right);
            Assert.That(velocity, Is.EqualTo(new Vector3(1f, 4f, 3f)));
            Assert.That(Dg5fReachSpec.PointVelocity(
                new Vector3(float.NaN, 0f, 0f),
                Vector3.one,
                Vector3.zero,
                Vector3.right), Is.EqualTo(Vector3.zero));
        }

        [Test]
        public void EvaluationEpisodes_AreDistributedWithoutOverlap()
        {
            var episodes = Enumerable.Range(0, ArmReachEvaluationSession.AreaCount)
                .SelectMany(area => Enumerable.Range(
                        0,
                        ArmReachEvaluationSession.EpisodesForArea(
                            500,
                            area,
                            ArmReachEvaluationSession.AreaCount))
                    .Select(local => ArmReachEvaluationSession.EpisodeForArea(
                        area,
                        local,
                        ArmReachEvaluationSession.AreaCount)))
                .ToArray();
            Assert.That(episodes, Has.Length.EqualTo(500));
            Assert.That(episodes.Distinct().Count(), Is.EqualTo(500));
            Assert.That(episodes.Min(), Is.Zero);
            Assert.That(episodes.Max(), Is.EqualTo(499));
        }
    }
}
