using System.Linq;
using NUnit.Framework;
using UnityEngine;

namespace KDT.GraspTraining.Tests
{
    public sealed class Dg5fGraspSpecTests
    {
        [Test]
        public void StableGraspKeepsTheTransferPolicyShape()
        {
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("3.0.0"));
            Assert.That(Dg5fGraspSpec.BehaviorName, Is.EqualTo("DG5FStableGrasp"));
            Assert.That(Dg5fGraspSpec.ObservationSize, Is.EqualTo(116));
            Assert.That(Dg5fGraspSpec.ActionSize, Is.EqualTo(26));
            Assert.That(Dg5fGraspSpec.ArmJointCount, Is.EqualTo(6));
            Assert.That(Dg5fGraspSpec.HandJointCount, Is.EqualTo(20));
            Assert.That(Dg5fGraspSpec.FingerCount, Is.EqualTo(5));
        }

        [Test]
        public void GraspPointUsesFiveEqualWeightsAndPalmLocalConversion()
        {
            var palmObject = new GameObject("palm");
            try
            {
                palmObject.transform.SetPositionAndRotation(
                    new Vector3(2f, -1f, 3f),
                    Quaternion.Euler(15f, 30f, -20f));
                palmObject.transform.localScale = new Vector3(1.2f, 0.8f, 1.5f);
                var tips = new Transform[Dg5fGraspSpec.FingerCount];
                var localPositions = new[]
                {
                    new Vector3(0f, 0f, 0f),
                    new Vector3(1f, 0f, 0f),
                    new Vector3(0f, 2f, 0f),
                    new Vector3(0f, 0f, 3f),
                    new Vector3(4f, 5f, 6f)
                };
                for (int index = 0; index < tips.Length; index++)
                {
                    tips[index] = new GameObject($"tip-{index}").transform;
                    tips[index].SetParent(palmObject.transform, false);
                    tips[index].localPosition = localPositions[index];
                }

                Vector3 expectedLocal = localPositions.Aggregate(Vector3.zero, (sum, value) => sum + value)
                    / Dg5fGraspSpec.FingerCount;
                Vector3 local = Dg5fGraspSpec.PalmLocalGraspPoint(
                    palmObject.transform,
                    tips);
                Assert.That(Vector3.Distance(local, expectedLocal), Is.LessThan(1e-5f));

                Vector3 equalWorld = Dg5fGraspSpec.EqualWeightedCenter(
                    tips.Select(tip => tip.position).ToArray());
                Assert.That(
                    Vector3.Distance(palmObject.transform.TransformPoint(local), equalWorld),
                    Is.LessThan(1e-5f));
            }
            finally
            {
                Object.DestroyImmediate(palmObject);
            }
        }

        [Test]
        public void ThumbPlusOneFailsWhileThumbPlusTwoIsStableContact()
        {
            var thumbOnly = new[] { true, false, false, false, false };
            var pinch = new[] { true, true, false, false, false };
            var stable = new[] { true, true, true, false, false };
            var noThumb = new[] { false, true, true, true, true };

            Assert.That(Dg5fGraspSpec.HasStableContact(thumbOnly), Is.False);
            Assert.That(Dg5fGraspSpec.HasStableContact(pinch), Is.False);
            Assert.That(Dg5fGraspSpec.HasStableContact(stable), Is.True);
            Assert.That(Dg5fGraspSpec.HasStableContact(noThumb), Is.False);
            Assert.That(Dg5fGraspSpec.NonThumbContactCount(stable), Is.EqualTo(2));
            Assert.That(Dg5fGraspSpec.ContactPotential(thumbOnly), Is.EqualTo(0.1f));
            Assert.That(Dg5fGraspSpec.ContactPotential(pinch), Is.EqualTo(0.2f));
            Assert.That(Dg5fGraspSpec.ContactPotential(stable), Is.EqualTo(0.3f));
            Assert.That(
                Dg5fGraspSpec.ContactPotential(new[] { true, true, true, true, true }),
                Is.EqualTo(0.5f));
            Assert.That(Dg5fGraspSpec.ContactPotential(noThumb), Is.Zero);
        }

        [Test]
        public void ContactLiftAndHoldPotentialsReverseExactlyOnLoss()
        {
            float noContact = Dg5fGraspSpec.ContactPotential(
                new[] { false, false, false, false, false });
            float fullContact = Dg5fGraspSpec.ContactPotential(
                new[] { true, true, true, true, true });
            AssertSymmetric(noContact, fullContact);

            float noLift = Dg5fGraspSpec.LiftPotential(0f, 0.05f, true);
            float fullLift = Dg5fGraspSpec.LiftPotential(0.05f, 0.05f, true);
            AssertSymmetric(noLift, fullLift);

            float noHold = Dg5fGraspSpec.StableHoldPotential(0f, 1f);
            float fullHold = Dg5fGraspSpec.StableHoldPotential(1f, 1f);
            AssertSymmetric(noHold, fullHold);

            float settlement = Dg5fGraspSpec.FailurePotentialSettlement(
                Dg5fGraspSpec.ApproachPotential(0.2f),
                fullContact,
                fullLift,
                fullHold);
            Assert.That(settlement, Is.LessThan(0f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("Timeout"), Is.EqualTo(-0.1f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("GripLost"), Is.EqualTo(-0.5f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("Drop"), Is.EqualTo(-0.5f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("WorkspaceExit"), Is.EqualTo(-1f));
            Assert.That(Dg5fGraspSpec.FailurePenalty("NonFinitePhysics"), Is.EqualTo(-1f));
        }

        [Test]
        public void StableGraspUsesExactHalfSecondBoundaryAndResetsBeforeAcquisition()
        {
            float hold = 0f;
            for (int step = 0; step < 24; step++)
                hold = Dg5fGraspSpec.NextContactHoldSeconds(hold, true, 0.02f);
            Assert.That(hold, Is.EqualTo(0.48f).Within(1e-5f));
            Assert.That(Dg5fGraspSpec.HasAcquiredStableGrasp(hold), Is.False);

            hold = Dg5fGraspSpec.NextContactHoldSeconds(hold, true, 0.02f);
            Assert.That(hold, Is.EqualTo(0.5f).Within(1e-5f));
            Assert.That(Dg5fGraspSpec.HasAcquiredStableGrasp(hold), Is.True);
            Assert.That(Dg5fGraspSpec.StableContactProgress(hold, false), Is.EqualTo(1f));

            hold = Dg5fGraspSpec.NextContactHoldSeconds(hold, false, 0.02f);
            Assert.That(hold, Is.Zero);
            Assert.That(Dg5fGraspSpec.HasAcquiredStableGrasp(hold), Is.False);
        }

        [Test]
        public void FinalLiftAndLowSpeedHoldUseExactBoundaries()
        {
            Assert.That(Dg5fGraspSpec.RequiredLiftHeight(1), Is.Zero);
            Assert.That(Dg5fGraspSpec.RequiredLiftHeight(2), Is.EqualTo(0.02f));
            Assert.That(Dg5fGraspSpec.RequiredLiftHeight(3), Is.EqualTo(0.05f));
            Assert.That(Dg5fGraspSpec.MinimumHoldHeight(3), Is.EqualTo(0.04f));
            Assert.That(Dg5fGraspSpec.RequiredStableHoldSeconds(3), Is.EqualTo(1f));
            Assert.That(Dg5fGraspSpec.HasReachedLiftTarget(0.0499f, 0.05f), Is.False);
            Assert.That(Dg5fGraspSpec.HasReachedLiftTarget(0.05f, 0.05f), Is.True);

            float hold = 0f;
            for (int step = 0; step < 49; step++)
            {
                hold = Dg5fGraspSpec.NextStableHoldSeconds(
                    hold, true, true, 0.04f, 0.04f, 0.05f, 0.02f);
            }
            Assert.That(hold, Is.EqualTo(0.98f).Within(1e-5f));
            Assert.That(Dg5fGraspSpec.HasCompletedStableHold(hold, 1f), Is.False);
            hold = Dg5fGraspSpec.NextStableHoldSeconds(
                hold, true, true, 0.04f, 0.04f, 0.05f, 0.02f);
            Assert.That(Dg5fGraspSpec.HasCompletedStableHold(hold, 1f), Is.True);

            Assert.That(Dg5fGraspSpec.NextStableHoldSeconds(
                hold, true, true, 0.0399f, 0.04f, 0.05f, 0.02f), Is.Zero);
            Assert.That(Dg5fGraspSpec.NextStableHoldSeconds(
                hold, true, true, 0.04f, 0.04f, 0.0501f, 0.02f), Is.Zero);
            Assert.That(Dg5fGraspSpec.NextStableHoldSeconds(
                hold, false, true, 0.04f, 0.04f, 0f, 0.02f), Is.Zero);
        }

        [Test]
        public void EvaluationDistributesTwoHundredUniqueEpisodesAcrossTwentyAreas()
        {
            var episodeIds = new System.Collections.Generic.HashSet<int>();
            for (int area = 0; area < Dg5fEvaluationSession.AreaCount; area++)
            {
                Assert.That(Dg5fEvaluationSession.EpisodesForArea(
                    200, area, Dg5fEvaluationSession.AreaCount), Is.EqualTo(10));
                for (int localEpisode = 0; localEpisode < 10; localEpisode++)
                    Assert.That(episodeIds.Add(Dg5fEvaluationSession.EpisodeIdForArea(
                        area, localEpisode, Dg5fEvaluationSession.AreaCount)), Is.True);
            }
            Assert.That(episodeIds, Is.EquivalentTo(Enumerable.Range(0, 200)));
        }

        [Test]
        public void ApproachPotentialAndFiveCentimeterMilestoneRemainStable()
        {
            float far = Dg5fGraspSpec.ApproachPotential(0.60f);
            float near = Dg5fGraspSpec.ApproachPotential(0.30f);
            Assert.That(Dg5fGraspSpec.PotentialDelta(far, near), Is.GreaterThan(0f));
            AssertSymmetric(far, near);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0.05001f), Is.False);
            Assert.That(Dg5fGraspSpec.HasReachedApproachTarget(0.05f), Is.True);
        }

        [Test]
        public void V1SpawnIsAreaUniformAcrossAllCardinalDirections()
        {
            const float ballRadius = 0.02f;
            Vector3 east = Dg5fGraspSpec.SpawnBallLocalPosition(0f, 0f, ballRadius);
            Vector3 north = Dg5fGraspSpec.SpawnBallLocalPosition(0.25f, 0.25f, ballRadius);
            Vector3 west = Dg5fGraspSpec.SpawnBallLocalPosition(0.5f, 0.5f, ballRadius);
            Vector3 south = Dg5fGraspSpec.SpawnBallLocalPosition(1f, 0.75f, ballRadius);
            Assert.That(Azimuth(east), Is.EqualTo(0f).Within(1e-3f));
            Assert.That(Azimuth(north), Is.EqualTo(90f).Within(1e-3f));
            Assert.That(Mathf.Abs(Azimuth(west)), Is.EqualTo(180f).Within(1e-3f));
            Assert.That(Azimuth(south), Is.EqualTo(-90f).Within(1e-3f));
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
            Assert.That(actionIndices, Is.EquivalentTo(Enumerable.Range(6, 20)));
            Assert.That(Dg5fGraspSpec.PreGrasp35Deg, Has.Length.EqualTo(20));
            Assert.That(Dg5fGraspSpec.AccumulateJointTarget(0f, 1f, 4f, -10f, 3f),
                Is.EqualTo(3f));
            Assert.That(Dg5fGraspSpec.AccumulateJointTarget(0f, -1f, 4f, -3f, 10f),
                Is.EqualTo(-3f));
        }

        [Test]
        public void EpisodeFailuresUseDistinctBoundaries()
        {
            Assert.That(Dg5fGraspSpec.ReachedEpisodeTimeout(19.999f), Is.False);
            Assert.That(Dg5fGraspSpec.ReachedEpisodeTimeout(20f), Is.True);
            Assert.That(Dg5fGraspSpec.BallFailureReason(
                new Vector3(0.85f, 0f, 0f), -1f), Is.Null);
            Assert.That(Dg5fGraspSpec.BallFailureReason(
                new Vector3(0.851f, 0f, 0f), -1f), Is.EqualTo("WorkspaceExit"));
            Assert.That(Dg5fGraspSpec.BallFailureReason(
                new Vector3(0f, -0.001f, 0f), 0f), Is.EqualTo("Drop"));
            Assert.That(Dg5fGraspSpec.BallFailureReason(
                new Vector3(float.NaN, 0f, 0f), 0f), Is.EqualTo("NonFinitePhysics"));
        }

        static void AssertSymmetric(float first, float second)
        {
            Assert.That(
                Dg5fGraspSpec.PotentialDelta(first, second),
                Is.EqualTo(-Dg5fGraspSpec.PotentialDelta(second, first)).Within(1e-6f));
        }

        static float Azimuth(Vector3 position)
        {
            return Mathf.Atan2(position.z, position.x) * Mathf.Rad2Deg;
        }
    }
}
