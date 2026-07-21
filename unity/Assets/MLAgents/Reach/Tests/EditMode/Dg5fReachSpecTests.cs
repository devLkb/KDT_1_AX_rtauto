using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;

namespace KDT.ReachTraining.Tests
{
    public sealed class Dg5fReachSpecTests
    {
        [Test]
        public void PolicyContract_IsVersionedAndContiguous()
        {
            Assert.That(Dg5fReachSpec.SpecVersion, Is.EqualTo("1.1.0"));
            Assert.That(Dg5fReachSpec.BehaviorName, Is.EqualTo("DG5FGraspPointReach"));
            Assert.That(Dg5fReachSpec.ArmJointCount, Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.ActionSize, Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.ObservationSize, Is.EqualTo(26));
            Assert.That(Dg5fReachSpec.DecisionPeriod, Is.EqualTo(5));

            Assert.That(Dg5fReachSpec.ArmPositionObservationOffset, Is.EqualTo(0));
            Assert.That(Dg5fReachSpec.ArmVelocityObservationOffset, Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.ArmTargetObservationOffset, Is.EqualTo(12));
            Assert.That(Dg5fReachSpec.TargetOffsetObservationOffset, Is.EqualTo(18));
            Assert.That(Dg5fReachSpec.DistanceObservationIndex, Is.EqualTo(21));
            Assert.That(Dg5fReachSpec.GraspPointVelocityObservationOffset, Is.EqualTo(22));
            Assert.That(Dg5fReachSpec.HoldProgressObservationIndex, Is.EqualTo(25));
        }

        [Test]
        public void ArmContract_HasExactlySixNamedSafeRanges()
        {
            Assert.That(Dg5fReachSpec.ArmLinks.Length, Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.ArmLinks.Distinct().Count(), Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.ArmSafeMinDeg.Length, Is.EqualTo(6));
            Assert.That(Dg5fReachSpec.ArmSafeMaxDeg.Length, Is.EqualTo(6));
            for (int index = 0; index < Dg5fReachSpec.ArmJointCount; index++)
                Assert.That(
                    Dg5fReachSpec.ArmSafeMinDeg[index],
                    Is.LessThan(Dg5fReachSpec.ArmSafeMaxDeg[index]));
        }

        [Test]
        public void GraspPoint_UsesCalibratedSingleLogicalPoint()
        {
            Assert.That(
                Dg5fReachSpec.CalibratedGraspPointLocalPosition.x,
                Is.EqualTo(0.0170203224f).Within(1e-8f));
            Assert.That(
                Dg5fReachSpec.CalibratedGraspPointLocalPosition.y,
                Is.EqualTo(0.152462155f).Within(1e-8f));
            Assert.That(
                Dg5fReachSpec.CalibratedGraspPointLocalPosition.z,
                Is.EqualTo(0.0135399457f).Within(1e-8f));
        }

        [Test]
        public void Spawn_IsDeterministicForSameSeed()
        {
            var first = new System.Random(91234);
            var second = new System.Random(91234);
            Vector3 initial = new Vector3(0.1f, 0.3f, -0.2f);

            for (int sample = 0; sample < 100; sample++)
            {
                Assert.That(
                    Dg5fReachSpec.SpawnTargetLocalPosition(first, initial),
                    Is.EqualTo(
                        Dg5fReachSpec.SpawnTargetLocalPosition(second, initial)));
            }
        }

        [Test]
        public void Spawn_CoversFullAzimuthAndUniformRadiusRange()
        {
            var random = new System.Random(4312);
            // A high initial point prevents rejection from biasing this distribution test.
            Vector3 initial = new Vector3(0f, 2f, 0f);
            var quadrantCounts = new int[4];
            float radiusSum = 0f;
            const int samples = 10000;

            for (int sample = 0; sample < samples; sample++)
            {
                Vector3 point = Dg5fReachSpec.SpawnTargetLocalPosition(random, initial);
                float radius = new Vector2(point.x, point.z).magnitude;
                radiusSum += radius;
                int quadrant = point.x >= 0f
                    ? (point.z >= 0f ? 0 : 3)
                    : (point.z >= 0f ? 1 : 2);
                quadrantCounts[quadrant]++;

                Assert.That(
                    Dg5fReachSpec.IsValidSpawn(point, initial),
                    Is.True);
                Assert.That(point.y, Is.EqualTo(Dg5fReachSpec.TargetRadius).Within(1e-6f));
            }

            float expectedUniformRadiusMean =
                (Dg5fReachSpec.MinimumTargetRadius
                    + Dg5fReachSpec.MaximumTargetRadius) * 0.5f;
            Assert.That(radiusSum / samples, Is.EqualTo(expectedUniformRadiusMean).Within(0.01f));
            Assert.That(quadrantCounts, Has.All.GreaterThan(2200));
        }

        [Test]
        public void Spawn_AlwaysRespectsInitialCenterSeparation()
        {
            var random = new System.Random(199);
            Vector3 initial = new Vector3(
                Dg5fReachSpec.MinimumTargetRadius,
                Dg5fReachSpec.TargetRadius,
                0f);

            for (int sample = 0; sample < 2000; sample++)
            {
                Vector3 point =
                    Dg5fReachSpec.SpawnTargetLocalPosition(random, initial);
                Assert.That(
                    Vector3.Distance(point, initial),
                    Is.GreaterThanOrEqualTo(
                        Dg5fReachSpec.MinimumInitialCenterDistance - 1e-5f));
            }
        }

        [Test]
        public void Spawn_AppliesScenePredicateWithinSameBoundedLoop()
        {
            var random = new System.Random(71);
            int attempts = 0;
            Vector3 point = Dg5fReachSpec.SpawnTargetLocalPosition(
                random,
                new Vector3(0f, 2f, 0f),
                Dg5fReachSpec.TargetRadius,
                candidate => ++attempts == 4);

            Assert.That(attempts, Is.EqualTo(4));
            Assert.That(
                Dg5fReachSpec.IsValidSpawn(
                    point,
                    new Vector3(0f, 2f, 0f)),
                Is.True);
        }

        [Test]
        public void Spawn_StopsAfterTwoHundredFiftySixRejectedCandidates()
        {
            var random = new System.Random(72);
            int attempts = 0;

            Assert.Throws<InvalidOperationException>(() =>
                Dg5fReachSpec.SpawnTargetLocalPosition(
                    random,
                    new Vector3(0f, 2f, 0f),
                    Dg5fReachSpec.TargetRadius,
                    candidate =>
                    {
                        attempts++;
                        return false;
                    }));
            Assert.That(attempts, Is.EqualTo(Dg5fReachSpec.MaximumSpawnAttempts));
        }

        [Test]
        public void PanelBounds_KeepEntireTargetSphereInsideRotatedBox()
        {
            var panelObject = new GameObject("PanelBoundsTest");
            try
            {
                var panel = panelObject.AddComponent<BoxCollider>();
                panel.size = new Vector3(2f, 0.2f, 2f);
                panelObject.transform.rotation = Quaternion.Euler(0f, 37f, 0f);

                Vector3 inside = panelObject.transform.TransformPoint(
                    new Vector3(0.97f, 0.1f, 0f));
                Vector3 outside = panelObject.transform.TransformPoint(
                    new Vector3(0.99f, 0.1f, 0f));
                Assert.That(
                    Dg5fReachSpec.IsTargetSphereWithinPanel(
                        inside,
                        Dg5fReachSpec.TargetRadius,
                        panel),
                    Is.True);
                Assert.That(
                    Dg5fReachSpec.IsTargetSphereWithinPanel(
                        outside,
                        Dg5fReachSpec.TargetRadius,
                        panel),
                    Is.False);
            }
            finally
            {
                UnityEngine.Object.DestroyImmediate(panelObject);
            }
        }

        [Test]
        public void SpawnValidation_RejectsRadiusHeightAndSeparationViolations()
        {
            Vector3 initial = Vector3.zero;
            Assert.That(
                Dg5fReachSpec.IsValidSpawn(
                    new Vector3(0.19f, Dg5fReachSpec.TargetRadius, 0f),
                    initial),
                Is.False);
            Assert.That(
                Dg5fReachSpec.IsValidSpawn(
                    new Vector3(0.86f, Dg5fReachSpec.TargetRadius, 0f),
                    initial),
                Is.False);
            Assert.That(
                Dg5fReachSpec.IsValidSpawn(
                    new Vector3(0.20f, 0.03f, 0f),
                    initial),
                Is.False);
            Assert.That(
                Dg5fReachSpec.IsValidSpawn(
                    new Vector3(0.20f, Dg5fReachSpec.TargetRadius, 0f),
                    new Vector3(0.20f, Dg5fReachSpec.TargetRadius, 0f)),
                Is.False);
        }

        [Test]
        public void Actions_ClampInputDeltaAndJointRange()
        {
            Assert.That(
                Dg5fReachSpec.AccumulateJointTarget(10f, 2f, 99f, -20f, 20f),
                Is.EqualTo(14f).Within(1e-6f));
            Assert.That(
                Dg5fReachSpec.AccumulateJointTarget(19f, 1f, 4f, -20f, 20f),
                Is.EqualTo(20f).Within(1e-6f));
            Assert.That(
                Dg5fReachSpec.AccumulateJointTarget(-19f, -1f, 4f, -20f, 20f),
                Is.EqualTo(-20f).Within(1e-6f));
            Assert.That(
                Dg5fReachSpec.AccumulateJointTarget(5f, 1f, -2f, -20f, 20f),
                Is.EqualTo(5f).Within(1e-6f));
        }

        [Test]
        public void PointVelocity_IncludesAngularVelocityAtOffsetPoint()
        {
            Vector3 velocity = Dg5fReachSpec.PointVelocity(
                new Vector3(1f, 2f, 3f),
                new Vector3(0f, 0f, 2f),
                Vector3.zero,
                Vector3.right);

            Assert.That(velocity.x, Is.EqualTo(1f).Within(1e-6f));
            Assert.That(velocity.y, Is.EqualTo(4f).Within(1e-6f));
            Assert.That(velocity.z, Is.EqualTo(3f).Within(1e-6f));
        }

        [Test]
        public void PointVelocity_NonFiniteInputCannotLeakToObservations()
        {
            Vector3 velocity = Dg5fReachSpec.PointVelocity(
                new Vector3(float.NaN, 0f, 0f),
                Vector3.one,
                Vector3.zero,
                Vector3.right);
            Assert.That(velocity, Is.EqualTo(Vector3.zero));
        }

        [Test]
        public void Reward_ApproachThenRetreatCannotFarmProgress()
        {
            float approach = Dg5fReachSpec.DecisionReward(0.5f, 0.4f);
            float retreat = Dg5fReachSpec.DecisionReward(0.4f, 0.5f);

            Assert.That(
                approach + retreat,
                Is.EqualTo(2f * Dg5fReachSpec.DecisionTimePenalty).Within(1e-6f));
            Assert.That(
                Dg5fReachSpec.DecisionReward(0.5f, 0.5f),
                Is.EqualTo(Dg5fReachSpec.DecisionTimePenalty).Within(1e-6f));
        }

        [Test]
        public void Reward_TerminalValuesMatchContract()
        {
            Assert.That(
                Dg5fReachSpec.SuccessReward(0f, 0f),
                Is.EqualTo(6f).Within(1e-6f));
            Assert.That(
                Dg5fReachSpec.SuccessReward(
                    Dg5fReachSpec.EpisodeTimeoutSeconds,
                    Dg5fReachSpec.SuccessDistance),
                Is.EqualTo(2f).Within(1e-6f));
            Assert.That(
                Dg5fReachSpec.FailurePenalty("Timeout"),
                Is.EqualTo(-1f).Within(1e-6f));
            Assert.That(
                Dg5fReachSpec.FailurePenalty("WorkspaceExit"),
                Is.EqualTo(-2f).Within(1e-6f));
            Assert.That(
                Dg5fReachSpec.FailurePenalty("NonFinitePhysics"),
                Is.EqualTo(-2f).Within(1e-6f));
        }

        [Test]
        public void SuccessState_BoundariesAreInclusive()
        {
            Assert.That(
                Dg5fReachSpec.MeetsSuccessState(
                    Dg5fReachSpec.SuccessDistance,
                    Dg5fReachSpec.MaximumSuccessPointSpeed),
                Is.True);
            Assert.That(
                Dg5fReachSpec.MeetsSuccessState(
                    Dg5fReachSpec.SuccessDistance + 1e-5f,
                    Dg5fReachSpec.MaximumSuccessPointSpeed),
                Is.False);
            Assert.That(
                Dg5fReachSpec.MeetsSuccessState(
                    Dg5fReachSpec.SuccessDistance,
                    Dg5fReachSpec.MaximumSuccessPointSpeed + 1e-5f),
                Is.False);
        }

        [Test]
        public void SuccessHold_RequiresContinuousQuarterSecond()
        {
            float hold = Dg5fReachSpec.NextSuccessHoldSeconds(
                0.20f,
                Dg5fReachSpec.SuccessDistance,
                Dg5fReachSpec.MaximumSuccessPointSpeed,
                0.05f);
            Assert.That(hold, Is.EqualTo(0.25f).Within(1e-6f));
            Assert.That(Dg5fReachSpec.HasCompletedSuccessHold(hold), Is.True);
            Assert.That(Dg5fReachSpec.SuccessHoldProgress(hold), Is.EqualTo(1f));

            Assert.That(
                Dg5fReachSpec.NextSuccessHoldSeconds(
                    hold,
                    Dg5fReachSpec.SuccessDistance + 1e-4f,
                    0f,
                    0.02f),
                Is.Zero);
            Assert.That(
                Dg5fReachSpec.NextSuccessHoldSeconds(
                    hold,
                    0f,
                    Dg5fReachSpec.MaximumSuccessPointSpeed + 1e-4f,
                    0.02f),
                Is.Zero);
        }

        [Test]
        public void Curriculum_ProgressivelyTightensReachContract()
        {
            Assert.That(Dg5fReachSpec.SuccessDistanceForStage(1), Is.EqualTo(0.05f));
            Assert.That(Dg5fReachSpec.SuccessDistanceForStage(2), Is.EqualTo(0.03f));
            Assert.That(Dg5fReachSpec.SuccessDistanceForStage(3), Is.EqualTo(0.01f));
            Assert.That(Dg5fReachSpec.RequiredSuccessHoldSecondsForStage(1), Is.EqualTo(0.02f));
            Assert.That(Dg5fReachSpec.RequiredSuccessHoldSecondsForStage(2), Is.EqualTo(0.10f));
            Assert.That(Dg5fReachSpec.RequiredSuccessHoldSecondsForStage(3), Is.EqualTo(0.25f));
            Assert.That(Dg5fReachSpec.MinimumTargetRadiusForStage(1), Is.EqualTo(0.35f));
            Assert.That(Dg5fReachSpec.MaximumTargetRadiusForStage(1), Is.EqualTo(0.70f));
            Assert.That(Dg5fReachSpec.MinimumTargetRadiusForStage(3), Is.EqualTo(0.20f));
            Assert.That(Dg5fReachSpec.MaximumTargetRadiusForStage(3), Is.EqualTo(0.85f));
        }

        [Test]
        public void Curriculum_UsesTwoDegreesThenOneDegreeNearTarget()
        {
            Assert.That(Dg5fReachSpec.ArmDeltaDegForStage(1, 0.01f), Is.EqualTo(2f));
            Assert.That(Dg5fReachSpec.ArmDeltaDegForStage(2, 0.20f), Is.EqualTo(2f));
            Assert.That(Dg5fReachSpec.ArmDeltaDegForStage(2, 0.10f), Is.EqualTo(1f));
            Assert.That(Dg5fReachSpec.ArmDeltaDegForStage(3, 0.01f), Is.EqualTo(1f));
        }

        [Test]
        public void Timeout_IsExactlyTwentySimulationSeconds()
        {
            Assert.That(
                Dg5fReachSpec.ReachedEpisodeTimeout(
                    Dg5fReachSpec.EpisodeTimeoutSeconds - 1e-4f),
                Is.False);
            Assert.That(
                Dg5fReachSpec.ReachedEpisodeTimeout(
                    Dg5fReachSpec.EpisodeTimeoutSeconds),
                Is.True);
        }

        [Test]
        public void Workspace_BoundariesAreInclusive()
        {
            Assert.That(
                Dg5fReachSpec.IsWithinWorkspace(
                    new Vector3(
                        Dg5fReachSpec.WorkspaceRadius,
                        Dg5fReachSpec.WorkspaceMinimumY,
                        0f)),
                Is.True);
            Assert.That(
                Dg5fReachSpec.IsWithinWorkspace(
                    new Vector3(
                        0f,
                        Dg5fReachSpec.WorkspaceMaximumY,
                        Dg5fReachSpec.WorkspaceRadius)),
                Is.True);
            Assert.That(
                Dg5fReachSpec.IsWithinWorkspace(
                    new Vector3(Dg5fReachSpec.WorkspaceRadius + 1e-4f, 0f, 0f)),
                Is.False);
            Assert.That(
                Dg5fReachSpec.IsWithinWorkspace(
                    new Vector3(0f, Dg5fReachSpec.WorkspaceMaximumY + 1e-4f, 0f)),
                Is.False);
        }

        [Test]
        public void Agent_HasNoHandOrFingerPolicyState()
        {
            FieldInfo[] fields = typeof(Dg5fGraspPointReachAgent).GetFields(
                BindingFlags.Instance
                | BindingFlags.Public
                | BindingFlags.NonPublic
                | BindingFlags.DeclaredOnly);
            string[] forbiddenFields = fields
                .Select(field => field.Name)
                .Where(name =>
                    name.IndexOf("hand", StringComparison.OrdinalIgnoreCase) >= 0
                    || name.IndexOf("finger", StringComparison.OrdinalIgnoreCase) >= 0)
                .ToArray();

            Assert.That(forbiddenFields, Is.Empty);
            Assert.That(
                fields.Count(field => field.FieldType == typeof(ArticulationBody[])),
                Is.EqualTo(1),
                "The only articulation array must be the six arm joints.");
        }

        [Test]
        public void EvaluationContract_UsesExactStableCsvHeader()
        {
            Assert.That(
                ArmReachEvaluationSession.CsvHeader,
                Is.EqualTo(
                    "episode,seed,success,final_distance_meters,"
                    + "grasp_point_speed_mps,success_hold_seconds,elapsed_seconds,"
                    + "workspace_safe,finite_physics,termination_reason"));
            Assert.That(
                ArmReachEvaluationSession.EpisodesArgument,
                Is.EqualTo("--dg5f-eval-episodes"));
            Assert.That(
                ArmReachEvaluationSession.BaseSeedArgument,
                Is.EqualTo("--dg5f-eval-base-seed"));
            Assert.That(
                ArmReachEvaluationSession.CsvArgument,
                Is.EqualTo("--dg5f-eval-csv"));
        }

        [Test]
        public void EvaluationDistribution_CoversFiveHundredEpisodesExactlyOnce()
        {
            const int episodeCount = 500;
            const int areaCount = ArmReachEvaluationSession.AreaCount;
            var assigned = new HashSet<int>();

            for (int area = 0; area < areaCount; area++)
            {
                int perArea = ArmReachEvaluationSession.EpisodesForArea(
                    episodeCount,
                    area,
                    areaCount);
                Assert.That(perArea, Is.EqualTo(25));
                for (int local = 0; local < perArea; local++)
                {
                    assigned.Add(ArmReachEvaluationSession.EpisodeForArea(
                        area,
                        local,
                        areaCount));
                }
            }

            Assert.That(assigned.Count, Is.EqualTo(episodeCount));
            Assert.That(assigned.Min(), Is.Zero);
            Assert.That(assigned.Max(), Is.EqualTo(episodeCount - 1));
        }
    }
}
