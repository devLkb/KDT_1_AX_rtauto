using NUnit.Framework;
using UnityEngine;

namespace KDT.GraspTraining.Tests
{
    public sealed class Dg5fGraspSpecTests
    {
        [Test]
        public void V1KeepsTheForwardCompatiblePolicyShape()
        {
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("1.2.0"));
            Assert.That(Dg5fGraspSpec.BehaviorName, Is.EqualTo("DG5FGrasp"));
            Assert.That(Dg5fGraspSpec.ObservationSize, Is.EqualTo(57));
            Assert.That(Dg5fGraspSpec.ActionSize, Is.EqualTo(7));
            Assert.That(Dg5fGraspSpec.ArmJointCount, Is.EqualTo(6));
            Assert.That(Dg5fGraspSpec.HandJointCount, Is.EqualTo(20));
            Assert.That(Dg5fGraspSpec.FingerCount, Is.EqualTo(5));
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

        static float Azimuth(Vector3 position)
        {
            return Mathf.Atan2(position.z, position.x) * Mathf.Rad2Deg;
        }
    }
}
