using NUnit.Framework;

namespace KDT.GraspTraining.Tests
{
    public sealed class Dg5fGraspSpecTests
    {
        [Test]
        public void ContractSizesRemainFrozenForVersionOne()
        {
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("1.0.0"));
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
        public void JointNormalizationMapsLimitsToUnitInterval()
        {
            Assert.That(Dg5fGraspSpec.NormalizeJoint(-90f, -90f, 90f), Is.EqualTo(-1f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(0f, -90f, 90f), Is.EqualTo(0f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(90f, -90f, 90f), Is.EqualTo(1f));
            Assert.That(Dg5fGraspSpec.NormalizeJoint(900f, -90f, 90f), Is.EqualTo(1f));
        }

        [TestCase(0.05f, true, true, 0.10f, true)]
        [TestCase(0.049f, true, true, 0.099f, false)]
        [TestCase(0.051f, false, true, 0.099f, false)]
        [TestCase(0.051f, true, false, 0.099f, false)]
        [TestCase(0.051f, true, true, 0.101f, false)]
        public void FinalSuccessRequiresLiftContactsAndContainment(
            float lift,
            bool thumb,
            bool opposing,
            float distance,
            bool expected)
        {
            bool actual = Dg5fGraspSpec.FinalSuccess(lift, 0.05f, thumb, opposing, distance, 0.10f);
            Assert.That(actual, Is.EqualTo(expected));
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
