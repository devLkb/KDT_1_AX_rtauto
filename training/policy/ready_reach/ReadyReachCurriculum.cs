using Unity.MLAgents;
using UnityEngine;

namespace KDT.ReachTraining
{
    /// <summary>
    /// Runtime curriculum layer for the packaged 37-observation ReadyReach player.
    /// Stage 3 remains available for an explicit precision run, but the default
    /// trainer contract stops at stage 2 (3 cm).
    /// </summary>
    public static class ReadyReachCurriculum
    {
        public const string SpecVersion = "1.0.0";
        public const string ParameterName = "reach_stage";
        public const int FirstStage = 1;
        public const int FinalStage = 3;

        static int _stage = FirstStage;

        public static int CurrentStage => _stage;

        public static void RefreshStage()
        {
            SetStage(Academy.Instance.EnvironmentParameters.GetWithDefault(
                ParameterName,
                FirstStage));
        }

        public static void SetStage(float value)
        {
            _stage = IsFinite(value)
                ? Mathf.Clamp(Mathf.RoundToInt(value), FirstStage, FinalStage)
                : FirstStage;
        }

        public static float SuccessDistance =>
            _stage <= FirstStage ? 0.05f : _stage < FinalStage ? 0.03f : 0.01f;

        public static float MaximumPointSpeed =>
            _stage <= FirstStage ? 1000f : _stage < FinalStage ? 0.15f : 0.05f;

        public static float RequiredHoldSeconds =>
            _stage <= FirstStage ? 0.02f : _stage < FinalStage ? 0.10f : 0.25f;

        public static bool MeetsLockState(
            float centerDistance,
            float pointSpeed,
            float palmAlignment,
            float upperConeAlignment)
        {
            return Dg5fReachSpec.IsFinite(centerDistance)
                && Dg5fReachSpec.IsFinite(pointSpeed)
                && Dg5fReachSpec.IsFinite(palmAlignment)
                && Dg5fReachSpec.IsFinite(upperConeAlignment)
                && centerDistance >= 0f
                && pointSpeed >= 0f
                && centerDistance <= SuccessDistance
                && pointSpeed <= MaximumPointSpeed
                && palmAlignment >= Dg5fReachSpec.MinimumPalmAlignment
                && upperConeAlignment >= Dg5fReachSpec.MinimumUpperConeAlignment;
        }

        public static float LockHoldProgress(float holdSeconds)
        {
            if (!Dg5fReachSpec.IsFinite(holdSeconds)) return 0f;
            return Mathf.Clamp01(
                Mathf.Max(0f, holdSeconds) / RequiredHoldSeconds);
        }

        public static bool HasCompletedLockHold(float holdSeconds)
        {
            return Dg5fReachSpec.IsFinite(holdSeconds)
                && holdSeconds >= RequiredHoldSeconds;
        }

        static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
