using NUnit.Framework;
using UnityEngine;

namespace KDT.GraspTraining.Tests
{
    public sealed class Dg5fGraspSpecTests
    {
        [Test]
        public void V2KeepsTheForwardCompatiblePolicyShape()
        {
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("2.1.0"));
            Assert.That(Dg5fGraspSpec.BehaviorName, Is.EqualTo("DG5FGraspJoint"));
            Assert.That(Dg5fGraspSpec.ObservationSize, Is.EqualTo(116));
            Assert.That(Dg5fGraspSpec.ActionSize, Is.EqualTo(26));
            Assert.That(Dg5fGraspSpec.ArmJointCount, Is.EqualTo(6));
            Assert.That(Dg5fGraspSpec.HandJointCount, Is.EqualTo(20));
            Assert.That(Dg5fGraspSpec.FingerCount, Is.EqualTo(5));
        }

        [Test]
        public void V2RewardPotentialsReverseExactlyWhenStateRetreats()
        {
            float farApproach = Dg5fGraspSpec.ApproachPotential(0.60f)
                * Dg5fGraspSpec.ApproachRewardScale;
            float nearApproach = Dg5fGraspSpec.ApproachPotential(0.20f)
                * Dg5fGraspSpec.ApproachRewardScale;
            Assert.That(
                Dg5fGraspSpec.PotentialDelta(farApproach, nearApproach),
                Is.EqualTo(-Dg5fGraspSpec.PotentialDelta(nearApproach, farApproach))
                    .Within(1e-6f));

            float noContact = Dg5fGraspSpec.ContactPotential(false, false);
            float opposingOnly = Dg5fGraspSpec.ContactPotential(false, true);
            float thumbOnly = Dg5fGraspSpec.ContactPotential(true, false);
            float dualContact = Dg5fGraspSpec.ContactPotential(true, true);
            Assert.That(opposingOnly, Is.Zero);
            Assert.That(thumbOnly, Is.EqualTo(0.25f));
            Assert.That(dualContact, Is.EqualTo(0.5f));
            Assert.That(
                Dg5fGraspSpec.PotentialDelta(noContact, dualContact),
                Is.EqualTo(-Dg5fGraspSpec.PotentialDelta(dualContact, noContact)));

            float noHold = Dg5fGraspSpec.ContactHoldPotential(0f);
            float fullHold = Dg5fGraspSpec.ContactHoldPotential(0.5f);
            Assert.That(fullHold, Is.EqualTo(0.5f));
            Assert.That(
                Dg5fGraspSpec.PotentialDelta(noHold, fullHold),
                Is.EqualTo(-Dg5fGraspSpec.PotentialDelta(fullHold, noHold)));

            float remaining = Dg5fGraspSpec.FailurePotentialSettlement(
                Dg5fGraspSpec.ApproachPotential(0.2f),
                dualContact,
                fullHold);
            Assert.That(remaining, Is.LessThan(0f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("BallOutOfBounds"), Is.EqualTo(-1f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("NonFinitePhysics"), Is.EqualTo(-1f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("Timeout"), Is.Zero);
        }

        [Test]
        public void ThumbAndOpposingContactsUseTheFixedFingerIndices()
        {
            Assert.That(Dg5fGraspSpec.ThumbFingerIndex, Is.Zero);
            Assert.That(Dg5fGraspSpec.FirstOpposingFingerIndex, Is.EqualTo(1));
            Assert.That(Dg5fGraspSpec.HasDualContact(
                new[] { true, false, false, false, false }), Is.False);
            Assert.That(Dg5fGraspSpec.HasDualContact(
                new[] { false, true, true, true, true }), Is.False);

            for (int opposing = 1; opposing < Dg5fGraspSpec.FingerCount; opposing++)
            {
                var contacts = new bool[Dg5fGraspSpec.FingerCount];
                contacts[0] = true;
                contacts[opposing] = true;
                Assert.That(Dg5fGraspSpec.HasThumbContact(contacts), Is.True);
                Assert.That(Dg5fGraspSpec.HasOpposingContact(contacts), Is.True);
                Assert.That(Dg5fGraspSpec.HasDualContact(contacts), Is.True);
            }
        }

        [Test]
        public void DualContactHoldUsesExactHalfSecondBoundaryAndResetsImmediately()
        {
            float hold = 0f;
            for (int step = 0; step < 24; step++)
                hold = Dg5fGraspSpec.NextContactHoldSeconds(hold, true, 0.02f);
            Assert.That(hold, Is.EqualTo(0.48f).Within(1e-5f));
            Assert.That(Dg5fGraspSpec.HasHeldDualContact(hold), Is.False);

            hold = Dg5fGraspSpec.NextContactHoldSeconds(hold, true, 0.02f);
            Assert.That(hold, Is.EqualTo(0.5f).Within(1e-5f));
            Assert.That(Dg5fGraspSpec.HasHeldDualContact(hold), Is.True);

            hold = Dg5fGraspSpec.NextContactHoldSeconds(hold, false, 0.02f);
            Assert.That(hold, Is.Zero);
            Assert.That(Dg5fGraspSpec.HasHeldDualContact(hold), Is.False);
            Assert.That(Dg5fGraspSpec.ContactHoldPotential(hold), Is.Zero);
        }

        [Test]
        public void EvaluationDistributesTwoHundredUniqueEpisodesAcrossTwentyAreas()
        {
            var episodeIds = new System.Collections.Generic.HashSet<int>();
            for (int area = 0; area < Dg5fEvaluationSession.AreaCount; area++)
            {
                Assert.That(
                    Dg5fEvaluationSession.EpisodesForArea(
                        200,
                        area,
                        Dg5fEvaluationSession.AreaCount),
                    Is.EqualTo(10));
                for (int localEpisode = 0; localEpisode < 10; localEpisode++)
                    Assert.That(episodeIds.Add(Dg5fEvaluationSession.EpisodeIdForArea(
                        area,
                        localEpisode,
                        Dg5fEvaluationSession.AreaCount)), Is.True);
            }
            Assert.That(episodeIds.Count, Is.EqualTo(200));
            Assert.That(episodeIds, Is.EquivalentTo(System.Linq.Enumerable.Range(0, 200)));
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
        public void ApproachSuccessUsesTheExactFiveCentimeterBoundary()
        {
            Assert.That(Dg5fGraspSpec.ApproachSuccessDistance, Is.EqualTo(0.05f));
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0.05001f), Is.False);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0.05f), Is.True);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0f), Is.True);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(float.NaN), Is.False);
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
        public void TwentyHandActionsMapOneToOneAndClampToJointLimits()
        {
            var actionIndices = new System.Collections.Generic.HashSet<int>();
            for (int joint = 0; joint < Dg5fGraspSpec.HandJointCount; joint++)
                Assert.That(actionIndices.Add(Dg5fGraspSpec.HandActionIndex(joint)), Is.True);
            Assert.That(actionIndices,
                Is.EquivalentTo(System.Linq.Enumerable.Range(6, 20)));
            Assert.That(Dg5fGraspSpec.PreGrasp35Deg, Has.Length.EqualTo(20));
            Assert.That(Dg5fGraspSpec.AccumulateJointTarget(0f, 1f, 4f, -10f, 3f),
                Is.EqualTo(3f));
            Assert.That(Dg5fGraspSpec.AccumulateJointTarget(0f, -1f, 4f, -3f, 10f),
                Is.EqualTo(-3f));
            Assert.That(Dg5fGraspSpec.AccumulateJointTarget(0f, 0.5f, 4f, -10f, 10f),
                Is.EqualTo(2f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(-10f, -10f, 10f), Is.EqualTo(-1f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(10f, -10f, 10f), Is.EqualTo(1f));
            Assert.That(() => Dg5fGraspSpec.HandActionIndex(-1),
                Throws.TypeOf<System.ArgumentOutOfRangeException>());
            Assert.That(() => Dg5fGraspSpec.HandActionIndex(20),
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

        static float Azimuth(Vector3 position)
        {
            return Mathf.Atan2(position.z, position.x) * Mathf.Rad2Deg;
        }
    }
}
