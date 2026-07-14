using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// Versioned, pure-data contract shared by the Agent, scene builder, and tests.
    /// Changing observation order or action semantics requires a new spec version.
    /// </summary>
    public static class Dg5fGraspSpec
    {
        public const string SpecVersion = "1.0.0";
        public const string BehaviorName = "DG5FGrasp";
        public const int ObservationSize = 43;
        public const int ActionSize = 7;
        public const int ArmJointCount = 6;
        public const int HandJointCount = 20;
        public const int FingerCount = 5;

        public static readonly string[] ArmLinks =
        {
            "shoulder_link", "upper_arm_link", "forearm_link",
            "wrist_1_link", "wrist_2_link", "wrist_3_link"
        };

        public static readonly float[] ArmSafeMinDeg =
        {
            -90f, -120f, 20f, -180f, -150f, -180f
        };

        public static readonly float[] ArmSafeMaxDeg =
        {
            90f, -20f, 140f, 0f, -30f, 180f
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

        public static float GripTargetDeg(int channel, float closure)
        {
            if (channel < 0 || channel >= HandJointCount)
                throw new System.ArgumentOutOfRangeException(nameof(channel));
            return Mathf.Lerp(0f, LeftFistDeg[channel], Mathf.Clamp01(closure));
        }

        public static float NormalizeJoint(float valueDeg, float lowerDeg, float upperDeg)
        {
            if (upperDeg <= lowerDeg) return 0f;
            return Mathf.Clamp((valueDeg - lowerDeg) / (upperDeg - lowerDeg) * 2f - 1f, -1f, 1f);
        }

        public static bool FinalSuccess(
            float liftMeters,
            float requiredLiftMeters,
            bool thumbContact,
            bool opposingContact,
            float graspDistance,
            float maximumGraspDistance)
        {
            return liftMeters >= requiredLiftMeters
                && thumbContact
                && opposingContact
                && graspDistance <= maximumGraspDistance;
        }
    }
}
