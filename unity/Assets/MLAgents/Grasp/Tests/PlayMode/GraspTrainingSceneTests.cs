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
            yield return new WaitForFixedUpdate();

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
        public IEnumerator TrainingSceneLoadsWithVersionThreeContractAndSingleDriveOwner()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
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
            Assert.That(agent.pedestalCollider.bounds.min.y,
                Is.EqualTo(agent.transform.root.position.y).Within(2e-3f));
            Assert.That(agent.pedestalCollider.bounds.max.y,
                Is.EqualTo(agent.robotBase.position.y).Within(2e-3f));
            Assert.That(agent.ball.mass, Is.EqualTo(0.05f).Within(1e-6f));
            Assert.That(agent.ball.useGravity, Is.True);
            Color ballColor = agent.ball.GetComponent<Renderer>().sharedMaterial.color;
            Assert.That(ballColor.r, Is.EqualTo(1f).Within(1e-4f));
            Assert.That(ballColor.g, Is.EqualTo(0f).Within(1e-4f));
            Assert.That(ballColor.b, Is.EqualTo(0f).Within(1e-4f));
            Assert.That(agent.contactSensors, Has.Length.EqualTo(Dg5fGraspSpec.FingerCount));
            Assert.That(agent.MaxStep, Is.Zero, "v3 has no fixed episode-length cutoff.");

            Vector3 center = agent.robotBase.TransformPoint(Vector3.zero);
            Ray centerRay = new Ray(center + Vector3.up, Vector3.down);
            Assert.That(agent.pedestalCollider.Raycast(centerRay, out RaycastHit panelHit, 2f), Is.True);
            Assert.That(panelHit.point.y,
                Is.EqualTo(agent.robotBase.position.y).Within(2e-3f));

            foreach (var collider in agent.palm.GetComponentsInChildren<Collider>(true))
                if (collider is MeshCollider handMeshCollider)
                    Assert.That(handMeshCollider.convex, Is.True,
                        $"{collider.name} must remain convex for ComputePenetration against the static panel.");

            var behavior = agent.GetComponent<BehaviorParameters>();
            Assert.That(behavior.BrainParameters.VectorObservationSize, Is.EqualTo(43));
            Assert.That(behavior.BrainParameters.ActionSpec.NumContinuousActions, Is.EqualTo(7));

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
        public IEnumerator DeterministicResetsCoverWorkspaceAndPlaceBallOnPedestal()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            Assert.That(agent, Is.Not.Null);
            agent.useDeterministicSpawns = true;

            var closeAction = new float[Dg5fGraspSpec.ActionSize];
            closeAction[6] = 1f;
            agent.OnActionReceived(new ActionBuffers(closeAction, new int[0]));
            Assert.That(agent.CurrentClosure, Is.GreaterThan(0f));
            agent.OnEpisodeBegin();
            Assert.That(agent.CurrentClosure, Is.Zero, "Episode reset must open the hand before writing drives.");

            bool[] quadrants = new bool[4];
            Collider ballCollider = agent.ball.GetComponent<Collider>();
            Collider[] robotColliders = agent.robotBase.GetComponentsInChildren<Collider>(true);
            Vector3 pedestalPosition = agent.pedestal.position;
            Quaternion pedestalRotation = agent.pedestal.rotation;
            Vector3 pedestalScale = agent.pedestal.localScale;
            for (int reset = 0; reset < 100; reset++)
            {
                agent.OnEpisodeBegin();
                Vector3 localBall = agent.CurrentBallLocalPosition;
                float radius = new Vector2(localBall.x, localBall.z).magnitude;
                Assert.That(radius, Is.InRange(
                    Dg5fGraspSpec.MinimumSpawnRadius - 1e-4f,
                    Dg5fGraspSpec.MaximumSpawnRadius + 1e-4f));
                Assert.That(agent.CurrentSupportTopHeight,
                    Is.EqualTo(Dg5fGraspSpec.SupportTopHeight).Within(1e-6f));
                Assert.That(localBall.magnitude, Is.LessThanOrEqualTo(
                    Dg5fGraspSpec.MaximumSpawnBallDistance + 1e-4f));
                Assert.That(ballCollider.bounds.min.y, Is.EqualTo(agent.pedestalCollider.bounds.max.y).Within(2e-3f));
                Assert.That(Dg5fGraspSpec.IsFinite(localBall), Is.True);
                Assert.That(agent.pedestal.position, Is.EqualTo(pedestalPosition));
                Assert.That(agent.pedestal.rotation, Is.EqualTo(pedestalRotation));
                Assert.That(agent.pedestal.localScale, Is.EqualTo(pedestalScale));

                float ballRadius = ballCollider.bounds.extents.y;
                Assert.That(Mathf.Abs(localBall.x) + ballRadius,
                    Is.LessThanOrEqualTo(Dg5fGraspSpec.PanelWidth * 0.5f + 1e-4f));
                Assert.That(Mathf.Abs(localBall.z) + ballRadius,
                    Is.LessThanOrEqualTo(Dg5fGraspSpec.PanelDepth * 0.5f + 1e-4f));
                foreach (var robotCollider in robotColliders)
                {
                    if (robotCollider == null
                        || !robotCollider.enabled
                        || !robotCollider.gameObject.activeInHierarchy
                        || robotCollider.isTrigger)
                    {
                        continue;
                    }

                    float centerToSurfaceDistance = Vector3.Distance(
                        agent.ball.position,
                        robotCollider.ClosestPoint(agent.ball.position));
                    float surfaceClearance = centerToSurfaceDistance - ballRadius;
                    Assert.That(
                        surfaceClearance,
                        Is.GreaterThanOrEqualTo(Dg5fGraspSpec.MinimumRobotSpawnClearance - 1e-4f),
                        $"Reset {reset}: ball too close to {robotCollider.name}.");
                }

                int quadrant = localBall.x >= 0f
                    ? (localBall.z >= 0f ? 0 : 3)
                    : (localBall.z >= 0f ? 1 : 2);
                quadrants[quadrant] = true;
            }
            Assert.That(quadrants, Is.All.True, "Fixed-seed resets must exercise all four azimuth quadrants.");

            foreach (var body in agent.GetComponentsInChildren<ArticulationBody>())
            {
                if (body.jointType != ArticulationJointType.RevoluteJoint || body.dofCount == 0) continue;
                Assert.That(float.IsNaN(body.jointPosition[0]), Is.False, body.name);
                Assert.That(Mathf.Abs(body.jointVelocity[0]), Is.LessThan(1e-6f), body.name);
            }
        }

        [UnityTest]
        public IEnumerator ShoulderPanActionCanReachEntireSafeRange()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;

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
    }
}
