using System.Collections;
using System.Collections.Generic;
using System.Linq;
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

            var agents = Object.FindObjectsByType<Dg5fGraspAgent>(FindObjectsSortMode.None);
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
                foreach (var sensor in agent.contactSensors)
                    Assert.That(sensor.targetBall, Is.SameAs(agent.ball));
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
        public IEnumerator TrainingSceneLoadsWithV1ContractAndSingleDriveOwner()
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
            Assert.That(agent.graspPoint.parent, Is.EqualTo(agent.palm));
            Assert.That(Vector3.Distance(
                    agent.graspPoint.localPosition,
                    Dg5fGraspSpec.FullHandGraspPointLocalPosition),
                Is.LessThan(1e-6f));
            Assert.That(Vector3.Angle(agent.graspPoint.forward, agent.palm.forward),
                Is.LessThan(1e-3f));
            Assert.That(agent.CurrentPalmFacingAlignment,
                Is.EqualTo(Dg5fGraspSpec.PalmFacingAlignment(
                    agent.graspPoint.forward,
                    agent.ball.position - agent.palm.position)).Within(1e-6f));
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("1.3.0"));
            Assert.That(Dg5fGraspSpec.BehaviorName, Is.EqualTo("DG5FGrasp"));
            Assert.That(agent.MaxStep, Is.Zero, "v1 measures timeout in simulation time.");
            Assert.That(agent.enablePolicyClosure, Is.False);
            Assert.That(agent.endEpisodeOnReach, Is.True);
            GraspTeleoperationHandoff handoff =
                agent.GetComponent<GraspTeleoperationHandoff>();
            Assert.That(handoff, Is.Not.Null);
            Assert.That(handoff.agent, Is.SameAs(agent));
            Assert.That(handoff.teleoperationDrivers, Is.Not.Empty);
            Assert.That(handoff.teleoperationDrivers.Select(driver => driver.enabled),
                Is.All.False);

            Collider[] movingColliders = agent.robotBase
                .GetComponentsInChildren<Collider>(true)
                .Where(collider =>
                {
                    ArticulationBody body =
                        collider.GetComponentInParent<ArticulationBody>();
                    return collider.enabled && !collider.isTrigger
                        && body != null && !body.isRoot;
                })
                .ToArray();
            Assert.That(movingColliders, Is.Not.Empty);
            Assert.That(agent.safetySensors, Has.Length.EqualTo(movingColliders.Length));
            foreach (Collider collider in movingColliders)
            {
                GraspSurfaceContactSensor sensor =
                    collider.GetComponent<GraspSurfaceContactSensor>();
                Assert.That(sensor, Is.Not.Null, collider.name);
                Assert.That(sensor.agent, Is.SameAs(agent));
                Assert.That(sensor.unsafeSurface, Is.SameAs(agent.pedestalCollider));
            }

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

            string[] competingDrivers =
            {
                "Dg5fReceiver", "Dg5fHandDriver", "Dg5fFingerIK", "Dg5fThumbIK", "Dg5fJointLogger",
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
        public IEnumerator RepeatedResetsAfterMotionRemainFiniteAndReleaseBallSafely()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            Assert.That(agent, Is.Not.Null);
            agent.GetComponent<Unity.MLAgents.DecisionRequester>().enabled = false;
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
                action[6] = 1f;
                agent.OnActionReceived(new ActionBuffers(action, new int[0]));
                yield return new WaitForFixedUpdate();

                Assert.DoesNotThrow(agent.OnEpisodeBegin,
                    $"Reset {reset}: spawn must not depend on stale articulation colliders.");
                Assert.That(agent.CurrentClosure, Is.Zero);
                Assert.That(agent.CurrentEpisodeSeconds, Is.Zero);
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
                Assert.That(Dg5fGraspSpec.IsFinite(agent.BestGraspDistance), Is.True);
                Assert.That(agent.BestGraspDistance, Is.GreaterThanOrEqualTo(0f));
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
        public IEnumerator CompatibilityClosureActionIsIgnoredAndHandStaysOpen()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            Dg5fGraspAgent agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            agent.GetComponent<Unity.MLAgents.DecisionRequester>().enabled = false;
            ArticulationBody[] handJoints = agent.robotBase
                .GetComponentsInChildren<ArticulationBody>(true)
                .Where(body => body.name.Contains("_dg_")
                    && body.jointType == ArticulationJointType.RevoluteJoint)
                .ToArray();
            Assert.That(handJoints, Has.Length.EqualTo(Dg5fGraspSpec.HandJointCount));
            float[] openTargets = handJoints.Select(body => body.xDrive.target).ToArray();

            var action = new float[Dg5fGraspSpec.ActionSize];
            action[6] = 1f;
            for (int i = 0; i < 10; i++)
                agent.OnActionReceived(new ActionBuffers(action, new int[0]));
            yield return new WaitForFixedUpdate();

            Assert.That(agent.CurrentClosure, Is.Zero);
            Assert.That(handJoints.Select(body => body.xDrive.target),
                Is.EqualTo(openTargets).Within(1e-5f));
        }

        [UnityTest]
        public IEnumerator DeploymentReachLocksArmAndYieldsHandWriter()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            Dg5fGraspAgent agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            agent.GetComponent<Unity.MLAgents.DecisionRequester>().enabled = false;
            agent.endEpisodeOnReach = false;
            agent.ball.isKinematic = true;
            agent.ball.useGravity = false;
            agent.ball.position = agent.graspPoint.position;
            agent.ball.rotation = Quaternion.identity;
            Physics.SyncTransforms();
            yield return new WaitForFixedUpdate();

            Assert.That(agent.IsArmLocked, Is.True);
            Assert.That(agent.IsExternalHandControl, Is.True);
            Assert.That(agent.IsEpisodeActive, Is.False);
            Assert.That(agent.LastTerminationReason, Is.EqualTo("Success"));
            GraspTeleoperationHandoff handoff =
                agent.GetComponent<GraspTeleoperationHandoff>();
            yield return new WaitForFixedUpdate();
            Assert.That(handoff.IsTeleoperationActive, Is.True);
            Assert.That(handoff.teleoperationDrivers.Select(driver => driver.enabled),
                Is.All.True);

            ArticulationBody handJoint = agent.robotBase
                .GetComponentsInChildren<ArticulationBody>(true)
                .First(body => body.name.Contains("_dg_")
                    && body.jointType == ArticulationJointType.RevoluteJoint);
            ArticulationDrive externalTarget = handJoint.xDrive;
            externalTarget.target += 1f;
            handJoint.xDrive = externalTarget;
            float expectedHandTarget = handJoint.xDrive.target;
            float[] lockedArmTargets = Enumerable.Range(0, Dg5fGraspSpec.ArmJointCount)
                .Select(agent.CurrentArmTargetDeg)
                .ToArray();

            var action = Enumerable.Repeat(1f, Dg5fGraspSpec.ActionSize).ToArray();
            agent.OnActionReceived(new ActionBuffers(action, new int[0]));
            yield return new WaitForFixedUpdate();

            Assert.That(handJoint.xDrive.target,
                Is.EqualTo(expectedHandTarget).Within(1e-5f));
            Assert.That(Enumerable.Range(0, Dg5fGraspSpec.ArmJointCount)
                    .Select(agent.CurrentArmTargetDeg),
                Is.EqualTo(lockedArmTargets).Within(1e-5f));
        }

        [UnityTest]
        public IEnumerator ReportedPanelContactTerminatesAsUnsafeSurfaceContact()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();

            Dg5fGraspAgent agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            agent.GetComponent<Unity.MLAgents.DecisionRequester>().enabled = false;
            agent.NotifyUnsafeSurfaceContact(agent.pedestalCollider);
            yield return new WaitForFixedUpdate();

            Assert.That(agent.LastTerminationReason,
                Is.EqualTo("UnsafeSurfaceContact"));
        }
    }
}
