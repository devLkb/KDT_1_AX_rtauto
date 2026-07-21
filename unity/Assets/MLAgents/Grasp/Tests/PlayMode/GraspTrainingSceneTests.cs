using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace KDT.GraspTraining.PlayModeTests
{
    public sealed class GraspTrainingSceneTests
    {
        [UnityTest]
        public IEnumerator TrainingSceneContainsTwentyIndependentPrefabAreas()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            var agents = Object.FindObjectsByType<Dg5fGraspAgent>();
            Assert.That(agents, Has.Length.EqualTo(20));
            Assert.That(agents.Select(agent => agent.transform.root).Distinct().Count(), Is.EqualTo(20));
            Assert.That(agents.Select(agent => agent.ball).Distinct().Count(), Is.EqualTo(20));
            Assert.That(agents.Select(agent => agent.pedestal).Distinct().Count(), Is.EqualTo(20));
            Assert.That(agents.Select(agent => agent.spawnSeed).Distinct().Count(), Is.EqualTo(20));

            var areaPositions = new HashSet<Vector3>();
            foreach (var agent in agents)
            {
                Transform area = agent.transform.root;
                Assert.That(area.name, Does.StartWith("DG5F_GraspTrainingArea_"));
                Assert.That(areaPositions.Add(area.position), Is.True, "Training areas must not overlap.");
                Assert.That(agent.ball.transform.IsChildOf(area), Is.True);
                Assert.That(agent.pedestal.IsChildOf(area), Is.True);
                Assert.That(agent.GetComponent<BehaviorParameters>().BehaviorName,
                    Is.EqualTo(Dg5fGraspSpec.BehaviorName));
                for (int sensorIndex = 0; sensorIndex < agent.contactSensors.Length; sensorIndex++)
                {
                    var sensor = agent.contactSensors[sensorIndex];
                    Assert.That(sensor.targetBall, Is.SameAs(agent.ball));
                    Assert.That(sensor.fingerIndex, Is.EqualTo(sensorIndex));
                }

                Assert.DoesNotThrow(agent.OnEpisodeBegin);
                Assert.That(agent.IsEpisodeActive, Is.True);
                Assert.That(agent.CurrentEpisodeSeconds, Is.Zero);
            }

            Assert.That(areaPositions.Select(position => position.x).Distinct().OrderBy(value => value),
                Is.EqualTo(new[] { 0f, 3f, 6f, 9f }));
            Assert.That(areaPositions.Select(position => position.y).Distinct().OrderBy(value => value),
                Is.EqualTo(new[] { 0f, 3f, 6f, 9f, 12f }));
            Assert.That(areaPositions.Select(position => position.z).Distinct(), Is.EqualTo(new[] { 0f }));

            Camera camera = Object.FindAnyObjectByType<Camera>();
            Assert.That(camera, Is.Not.Null);
            Assert.That(camera.transform.position, Is.EqualTo(new Vector3(4.5f, 6f, -18f)));
            Assert.That(Vector3.Angle(camera.transform.forward, Vector3.forward), Is.LessThan(1e-4f));
        }

        [UnityTest]
        public IEnumerator TrainingSceneLoadsWithV2ContractAndSingleDriveOwner()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            Assert.That(agent, Is.Not.Null);
            Assert.That(agent.ball, Is.Not.Null);
            Assert.That(agent.pedestal, Is.Not.Null);
            Assert.That(agent.pedestalCollider, Is.Not.Null);
            Assert.That(agent.pedestal.name, Is.EqualTo("GraspPanel"));
            Assert.That(agent.pedestalCollider, Is.TypeOf<BoxCollider>());
            Assert.That(agent.pedestalCollider.bounds.size.x,
                Is.EqualTo(Dg5fGraspSpec.PanelWidth).Within(2e-3f));
            Assert.That(agent.pedestalCollider.bounds.size.y,
                Is.EqualTo(Dg5fGraspSpec.PanelThickness).Within(2e-3f));
            Assert.That(agent.pedestalCollider.bounds.size.z,
                Is.EqualTo(Dg5fGraspSpec.PanelDepth).Within(2e-3f));
            Assert.That(agent.ball.mass, Is.EqualTo(0.05f).Within(1e-6f));
            Assert.That(agent.ball.useGravity, Is.True);
            Color ballColor = agent.ball.GetComponent<Renderer>().sharedMaterial.color;
            Assert.That(ballColor.r, Is.EqualTo(1f).Within(1e-4f));
            Assert.That(ballColor.g, Is.EqualTo(0f).Within(1e-4f));
            Assert.That(ballColor.b, Is.EqualTo(0f).Within(1e-4f));
            Assert.That(agent.contactSensors, Has.Length.EqualTo(Dg5fGraspSpec.FingerCount));
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("2.1.0"));
            Assert.That(Dg5fGraspSpec.BehaviorName, Is.EqualTo("DG5FGraspJoint"));
            Assert.That(agent.MaxStep, Is.Zero, "v2 measures timeout in simulation time.");

            var behavior = agent.GetComponent<BehaviorParameters>();
            Assert.That(behavior.BehaviorName, Is.EqualTo(Dg5fGraspSpec.BehaviorName));
            Assert.That(behavior.BrainParameters.VectorObservationSize,
                Is.EqualTo(Dg5fGraspSpec.ObservationSize));
            Assert.That(behavior.BrainParameters.ActionSpec.NumContinuousActions,
                Is.EqualTo(Dg5fGraspSpec.ActionSize));

            var requester = agent.GetComponent<Unity.MLAgents.DecisionRequester>();
            Assert.That(requester, Is.Not.Null);
            Assert.That(requester.DecisionPeriod, Is.EqualTo(5));
            Assert.That(requester.TakeActionsBetweenDecisions, Is.False);

            agent.RequestDecision();
            yield return null;
            Assert.That(agent.GetObservations(), Has.Count.EqualTo(Dg5fGraspSpec.ObservationSize));
            Assert.That(agent.GetObservations(), Is.All.Matches<float>(Dg5fGraspSpec.IsFinite));
            Assert.That(agent.GetObservations().Skip(108).Take(4),
                Is.EqualTo(new[] { 0f, 1f, 0f, 0f }));

            string[] competingDrivers =
            {
                "Dg5fReceiver", "Dg5fHandDriver", "Dg5fThumbIK", "Dg5fJointLogger",
                "HandSliderUI", "ArmTargetIK", "RobotInitialPoseSync"
            };
            foreach (var behaviour in agent.GetComponents<MonoBehaviour>())
            {
                foreach (string typeName in competingDrivers)
                    if (behaviour != null && behaviour.GetType().Name == typeName)
                        Assert.That(behaviour.enabled, Is.False, $"{typeName} must be disabled in training.");
            }
        }

        [UnityTest]
        public IEnumerator FiveCentimeterReachIsAMilestoneAndDoesNotEndV2Episode()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            Assert.That(agent, Is.Not.Null);
            agent.ball.isKinematic = true;
            agent.ball.useGravity = false;
            agent.ball.position = agent.graspPoint.position;
            agent.ball.linearVelocity = Vector3.zero;
            agent.ball.angularVelocity = Vector3.zero;
            Physics.SyncTransforms();

            yield return new WaitForFixedUpdate();

            Assert.That(agent.ReachSucceeded, Is.True);
            Assert.That(agent.FirstReachSeconds, Is.GreaterThanOrEqualTo(0f));
            Assert.That(agent.IsEpisodeActive, Is.True,
                "V2 must continue after the inherited 5 cm reach milestone.");
            Assert.That(agent.CurrentEpisodeSeconds,
                Is.LessThan(Dg5fGraspSpec.EpisodeTimeoutSeconds));
        }

        [UnityTest]
        public IEnumerator StageOneLocksArmTargetsWhileTwentyHandTargetsRemainControllable()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            agent.curriculumStageOverride = 1;
            agent.OnEpisodeBegin();
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            Assert.That(agent.CurrentCurriculumStage, Is.EqualTo(1));
            Assert.That(agent.CurrentEpisodeTimeoutSeconds,
                Is.EqualTo(Dg5fGraspSpec.StageOneEpisodeTimeoutSeconds));
            float[] armBefore = Enumerable.Range(0, Dg5fGraspSpec.ArmJointCount)
                .Select(agent.CurrentArmTargetDeg)
                .ToArray();
            float[] handBefore = Enumerable.Range(0, Dg5fGraspSpec.HandJointCount)
                .Select(agent.CurrentHandTargetDeg)
                .ToArray();
            var action = Enumerable.Repeat(1f, Dg5fGraspSpec.ActionSize).ToArray();

            agent.OnActionReceived(new ActionBuffers(action, new int[0]));
            if (!Enumerable.Range(0, Dg5fGraspSpec.HandJointCount)
                    .Any(hand => !Mathf.Approximately(
                        agent.CurrentHandTargetDeg(hand),
                        handBefore[hand])))
            {
                for (int hand = 0; hand < Dg5fGraspSpec.HandJointCount; hand++)
                    action[Dg5fGraspSpec.HandActionIndex(hand)] = -1f;
                agent.OnActionReceived(new ActionBuffers(action, new int[0]));
            }

            for (int arm = 0; arm < armBefore.Length; arm++)
                Assert.That(agent.CurrentArmTargetDeg(arm),
                    Is.EqualTo(armBefore[arm]).Within(1e-6f));
            for (int hand = 0; hand < handBefore.Length; hand++)
            {
                Assert.That(
                    Mathf.Abs(agent.CurrentHandTargetDeg(hand) - handBefore[hand]),
                    Is.LessThanOrEqualTo(1f + 1e-6f));
            }
            Assert.That(
                Enumerable.Range(0, Dg5fGraspSpec.HandJointCount)
                    .Any(hand => !Mathf.Approximately(
                        agent.CurrentHandTargetDeg(hand),
                        handBefore[hand])),
                Is.True);
        }

        [UnityTest]
        public IEnumerator StageOneEndsAtFiveSecondsWithOrdinaryTimeout()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            agent.curriculumStageOverride = 1;
            agent.OnEpisodeBegin();
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();
            KeepBallKinematicAtSafeLocation(agent);
            int completedBefore = agent.CompletedEpisodeCount;

            for (int step = 0; step < 300 && agent.CompletedEpisodeCount == completedBefore; step++)
            {
                KeepBallKinematicAtSafeLocation(agent);
                yield return new WaitForFixedUpdate();
            }

            Assert.That(agent.CompletedEpisodeCount, Is.EqualTo(completedBefore + 1));
            Assert.That(agent.LastEpisodeSucceeded, Is.False);
            Assert.That(agent.LastFailureReason, Is.EqualTo("Timeout"));
        }

        [UnityTest]
        public IEnumerator StageTwoContinuesAfterReachThenUsesFiveSecondPostReachTimeout()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            agent.curriculumStageOverride = 2;
            agent.OnEpisodeBegin();
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            agent.ball.isKinematic = true;
            agent.ball.useGravity = false;
            agent.ball.position = agent.graspPoint.position;
            agent.ball.linearVelocity = Vector3.zero;
            agent.ball.angularVelocity = Vector3.zero;
            Physics.SyncTransforms();
            yield return new WaitForFixedUpdate();

            Assert.That(agent.ReachSucceeded, Is.True);
            Assert.That(agent.IsEpisodeActive, Is.True);
            int completedBefore = agent.CompletedEpisodeCount;
            for (int step = 0; step < 300 && agent.CompletedEpisodeCount == completedBefore; step++)
            {
                KeepBallKinematicAtSafeLocation(agent);
                yield return new WaitForFixedUpdate();
            }

            Assert.That(agent.CompletedEpisodeCount, Is.EqualTo(completedBefore + 1));
            Assert.That(agent.LastEpisodeSucceeded, Is.False);
            Assert.That(agent.LastFailureReason, Is.EqualTo("PostReachTimeout"));
        }

        [UnityTest]
        public IEnumerator HalfSecondDualContactEndsWithSuccess()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            agent.curriculumStageOverride = 3;
            agent.OnEpisodeBegin();
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();
            KeepBallKinematicAtSafeLocation(agent);

            Collider ballCollider = agent.ball.GetComponent<Collider>();
            ContactSet(agent.contactSensors[0]).Add(ballCollider);
            ContactSet(agent.contactSensors[1]).Add(ballCollider);
            int completedBefore = agent.CompletedEpisodeCount;
            for (int step = 0; step < 30 && agent.CompletedEpisodeCount == completedBefore; step++)
                yield return new WaitForFixedUpdate();

            Assert.That(agent.CompletedEpisodeCount, Is.EqualTo(completedBefore + 1));
            Assert.That(agent.LastEpisodeSucceeded, Is.True);
            Assert.That(agent.LastFailureReason, Is.EqualTo("None"));
            Assert.That(agent.LastMaxContactHoldSeconds,
                Is.GreaterThanOrEqualTo(Dg5fGraspSpec.RequiredContactHoldSeconds));
        }

        [UnityTest]
        public IEnumerator RepeatedResetsAfterMotionRemainFiniteAndReleaseBallSafely()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            Assert.That(agent, Is.Not.Null);
            agent.useDeterministicSpawns = true;

            Collider ballCollider = agent.ball.GetComponent<Collider>();
            Collider[] robotColliders = agent.robotBase.GetComponentsInChildren<Collider>(true);
            Vector3 pedestalPosition = agent.pedestal.position;
            Quaternion pedestalRotation = agent.pedestal.rotation;
            Vector3 pedestalScale = agent.pedestal.localScale;

            for (int reset = 0; reset < 20; reset++)
            {
                var action = new float[Dg5fGraspSpec.ActionSize];
                action[0] = reset % 2 == 0 ? 1f : -1f;
                action[Dg5fGraspSpec.HandActionIndex(0)] = 1f;
                agent.OnActionReceived(new ActionBuffers(action, new int[0]));
                yield return new WaitForFixedUpdate();

                Assert.DoesNotThrow(agent.OnEpisodeBegin,
                    $"Reset {reset}: spawn must not depend on stale articulation colliders.");
                Assert.That(agent.CurrentEpisodeSeconds, Is.Zero);
                for (int joint = 0; joint < Dg5fGraspSpec.HandJointCount; joint++)
                {
                    Assert.That(agent.CurrentHandTargetDeg(joint),
                        Is.EqualTo(agent.CurrentHandDriveTargetDeg(joint)).Within(1e-5f));
                    Assert.That(agent.CurrentHandPositionDeg(joint),
                        Is.EqualTo(agent.CurrentHandTargetDeg(joint)).Within(1e-3f));
                    Assert.That(agent.CurrentHandVelocityRadPerSecond(joint),
                        Is.EqualTo(0f).Within(1e-6f));
                }
                Assert.That(agent.ball.isKinematic, Is.True,
                    $"Reset {reset}: ball stays kinematic until reset pose reaches physics.");
                Assert.That(agent.ball.useGravity, Is.False);

                yield return new WaitForFixedUpdate();
                yield return new WaitForFixedUpdate();

                Vector3 localBall = agent.CurrentBallLocalPosition;
                float radius = new Vector2(localBall.x, localBall.z).magnitude;
                Assert.That(radius, Is.InRange(
                    Dg5fGraspSpec.V1MinimumSpawnRadius - 1e-4f,
                    Dg5fGraspSpec.V1MaximumSpawnRadius + 1e-4f));
                Assert.That(agent.CurrentSupportTopHeight,
                    Is.EqualTo(Dg5fGraspSpec.SupportTopHeight).Within(1e-6f));
                Assert.That(agent.ball.isKinematic, Is.False);
                Assert.That(agent.ball.useGravity, Is.True);
                Assert.That(Dg5fGraspSpec.IsFinite(localBall), Is.True);
                Assert.That(Dg5fGraspSpec.IsFinite(agent.ball.linearVelocity), Is.True);
                Assert.That(Dg5fGraspSpec.IsFinite(agent.ball.angularVelocity), Is.True);
                Assert.That(agent.BestGraspDistance, Is.EqualTo(agent.CurrentGraspDistance).Within(1e-4f));
                Assert.That(agent.pedestal.position, Is.EqualTo(pedestalPosition));
                Assert.That(agent.pedestal.rotation, Is.EqualTo(pedestalRotation));
                Assert.That(agent.pedestal.localScale, Is.EqualTo(pedestalScale));
                Assert.That(agent.contactSensors.Select(sensor => sensor.IsTouching), Is.All.False);

                float ballRadius = ballCollider.bounds.extents.y;
                Assert.That(ballCollider.bounds.min.y,
                    Is.EqualTo(agent.pedestalCollider.bounds.max.y).Within(2e-3f));
                Assert.That(Mathf.Abs(localBall.x) + ballRadius,
                    Is.LessThanOrEqualTo(Dg5fGraspSpec.PanelWidth * 0.5f + 1e-4f));
                Assert.That(Mathf.Abs(localBall.z) + ballRadius,
                    Is.LessThanOrEqualTo(Dg5fGraspSpec.PanelDepth * 0.5f + 1e-4f));
                foreach (Collider robotCollider in robotColliders)
                {
                    if (robotCollider == null
                        || !robotCollider.enabled
                        || !robotCollider.gameObject.activeInHierarchy
                        || robotCollider.isTrigger)
                    {
                        continue;
                    }

                    bool overlaps = Physics.ComputePenetration(
                        ballCollider,
                        ballCollider.transform.position,
                        ballCollider.transform.rotation,
                        robotCollider,
                        robotCollider.transform.position,
                        robotCollider.transform.rotation,
                        out _,
                        out _);
                    Assert.That(overlaps, Is.False,
                        $"Reset {reset}: ball overlaps {robotCollider.name} after articulation reset.");
                }
            }
        }

        [UnityTest]
        public IEnumerator ShoulderPanActionCanReachEntireSafeRange()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            var action = new float[Dg5fGraspSpec.ActionSize];
            action[0] = 1f;
            for (int i = 0; i < 100; i++)
                agent.OnActionReceived(new ActionBuffers(action, new int[0]));
            Assert.That(agent.CurrentArmTargetDeg(0), Is.EqualTo(180f));

            action[0] = -1f;
            for (int i = 0; i < 200; i++)
                agent.OnActionReceived(new ActionBuffers(action, new int[0]));
            Assert.That(agent.CurrentArmTargetDeg(0), Is.EqualTo(-180f));
        }

        [UnityTest]
        public IEnumerator EveryHandActionChangesOnlyItsMappedJointTarget()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            Assert.That(agent, Is.Not.Null);
            for (int selected = 0; selected < Dg5fGraspSpec.HandJointCount; selected++)
            {
                float[] before = Enumerable.Range(0, Dg5fGraspSpec.HandJointCount)
                    .Select(agent.CurrentHandTargetDeg)
                    .ToArray();
                var action = new float[Dg5fGraspSpec.ActionSize];
                action[Dg5fGraspSpec.HandActionIndex(selected)] = 1f;
                agent.OnActionReceived(new ActionBuffers(action, new int[0]));
                if (Mathf.Approximately(before[selected], agent.CurrentHandTargetDeg(selected)))
                {
                    action[Dg5fGraspSpec.HandActionIndex(selected)] = -1f;
                    agent.OnActionReceived(new ActionBuffers(action, new int[0]));
                }

                Assert.That(agent.CurrentHandTargetDeg(selected),
                    Is.Not.EqualTo(before[selected]).Within(1e-6f),
                    $"hand action {Dg5fGraspSpec.HandActionIndex(selected)} did not move joint {selected}");
                for (int other = 0; other < Dg5fGraspSpec.HandJointCount; other++)
                {
                    if (other == selected) continue;
                    Assert.That(agent.CurrentHandTargetDeg(other),
                        Is.EqualTo(before[other]).Within(1e-6f),
                        $"hand action for {selected} changed joint {other}");
                }
            }
        }

        static void KeepBallKinematicAtSafeLocation(Dg5fGraspAgent agent)
        {
            agent.ball.isKinematic = true;
            agent.ball.useGravity = false;
            agent.ball.position = agent.robotBase.TransformPoint(new Vector3(0f, 0.4f, 0f));
            agent.ball.linearVelocity = Vector3.zero;
            agent.ball.angularVelocity = Vector3.zero;
            Physics.SyncTransforms();
        }

        static HashSet<Collider> ContactSet(GraspContactSensor sensor)
        {
            FieldInfo field = typeof(GraspContactSensor).GetField(
                "_contacts",
                BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(field, Is.Not.Null);
            return (HashSet<Collider>)field.GetValue(sensor);
        }
    }
}
