using System.Collections;
using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace KDT.GraspTraining.PlayModeTests
{
    public sealed class GraspTrainingSceneTests
    {
        [Test]
        public void FinalLessonStateHandlesRegraspLiftHoldAndDropScenarios()
        {
            Dg5fCurriculumStage stage = Dg5fGraspSpec.GetCurriculumStage(4);
            var state = new Dg5fEpisodeState();
            state.Reset();

            // A premature contact break returns to Reach; reacquisition must earn
            // a fresh continuous 0.25 seconds before lifting begins.
            state.Advance(Input(true, 0f, 0f), stage, 0.20f);
            state.Advance(Input(false, 0f, 0f), stage, 0.01f);
            Assert.That(state.Phase, Is.EqualTo(Dg5fGraspPhase.Reach));
            state.Advance(Input(true, 0f, 0f), stage, 0.25f);
            Assert.That(state.Phase, Is.EqualTo(Dg5fGraspPhase.Lift));

            state.Advance(Input(true, 0.10f, 0.04f), stage, 0.02f);
            Assert.That(state.Phase, Is.EqualTo(Dg5fGraspPhase.Hold));
            Assert.That(state.MaximumLiftMeters, Is.GreaterThanOrEqualTo(0.10f));

            // Invalid hold time is discarded, then a complete 5 seconds succeeds.
            state.Advance(Input(true, 0.10f, 0.04f), stage, 2f);
            state.Advance(Input(true, 0.08f, 0.04f), stage, 0.02f);
            Assert.That(state.HoldSeconds, Is.Zero);
            state.Advance(Input(true, 0.10f, 0.04f), stage, 0.02f);
            Assert.That(state.IsTerminal, Is.False);
            Dg5fEpisodeStepResult success = state.Advance(Input(true, 0.10f, 0.04f), stage, 4.98f);
            Assert.That(success.Succeeded, Is.True);
            Assert.That(state.HoldSeconds, Is.EqualTo(5f).Within(1e-5f));

            state.Reset();
            state.Advance(Input(true, 0f, 0f), stage, 0.25f);
            state.Advance(Input(true, 0.10f, 0.04f), stage, 0.02f);
            Dg5fEpisodeStepResult dropped = state.Advance(Input(true, 0.02f, 0.04f), stage, 0.02f);
            Assert.That(dropped.FailureReason, Is.EqualTo(Dg5fFailureReason.Dropped));
        }

        static Dg5fEpisodeInput Input(bool contact, float liftMeters, float speed)
        {
            return new Dg5fEpisodeInput(contact, liftMeters, speed, true, false, true);
        }

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
        public IEnumerator TrainingSceneLoadsWithVersionFourContractAndSingleDriveOwner()
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
            Assert.That(Dg5fGraspSpec.SpecVersion, Is.EqualTo("4.0.0"));
            Assert.That(Dg5fGraspSpec.BehaviorName, Is.EqualTo("DG5FGraspV4"));
            Assert.That(agent.MaxStep, Is.Zero, "v4 enforces its 20-second timeout in simulation time.");

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
            Assert.That(behavior.BehaviorName, Is.EqualTo(Dg5fGraspSpec.BehaviorName));
            Assert.That(behavior.BrainParameters.VectorObservationSize,
                Is.EqualTo(Dg5fGraspSpec.ObservationSize));
            Assert.That(behavior.BrainParameters.ActionSpec.NumContinuousActions,
                Is.EqualTo(Dg5fGraspSpec.ActionSize));

            var requester = agent.GetComponent<DecisionRequester>();
            Assert.That(requester, Is.Not.Null);
            Assert.That(requester.DecisionPeriod, Is.EqualTo(5));
            Assert.That(requester.TakeActionsBetweenDecisions, Is.False);

            agent.RequestDecision();
            yield return null;
            Assert.That(agent.GetObservations(), Has.Count.EqualTo(Dg5fGraspSpec.ObservationSize));
            Assert.That(agent.GetObservations(), Is.All.Matches<float>(Dg5fGraspSpec.IsFinite));

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
            agent.OnEpisodeBegin();

            var closeAction = new float[Dg5fGraspSpec.ActionSize];
            float initialShoulderTarget = agent.CurrentArmTargetDeg(0);
            closeAction[0] = 1f;
            closeAction[6] = 1f;
            agent.OnActionReceived(new ActionBuffers(closeAction, new int[0]));
            Assert.That(agent.CurrentClosure, Is.GreaterThan(0f));
            Assert.That(agent.CurrentArmTargetDeg(0), Is.Not.EqualTo(initialShoulderTarget));
            agent.OnEpisodeBegin();
            Assert.That(agent.CurrentClosure, Is.Zero, "Episode reset must open the hand before writing drives.");
            Assert.That(agent.CurrentArmTargetDeg(0), Is.EqualTo(initialShoulderTarget).Within(1e-4f));

            Collider ballCollider = agent.ball.GetComponent<Collider>();
            Collider[] robotColliders = agent.robotBase.GetComponentsInChildren<Collider>(true);
            ArticulationBody[] revoluteBodies = agent.GetComponentsInChildren<ArticulationBody>()
                .Where(body => body.jointType == ArticulationJointType.RevoluteJoint && body.dofCount > 0)
                .ToArray();
            ArticulationBody[] armBodies = Dg5fGraspSpec.ArmLinks
                .Select(link => revoluteBodies.Single(body => body.name == link))
                .ToArray();
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
                Assert.That(agent.CurrentClosure, Is.Zero,
                    $"Reset {reset}: grip closure was not cleared.");
                Assert.That(agent.CurrentPhase, Is.EqualTo(Dg5fGraspPhase.Reach));
                Assert.That(agent.CurrentGraspSeconds, Is.Zero);
                Assert.That(agent.CurrentHoldSeconds, Is.Zero);
                Assert.That(agent.CurrentEpisodeSeconds, Is.Zero);
                Assert.That(agent.MaximumLiftMeters, Is.Zero);
                Assert.That(agent.MaximumHoldSeconds, Is.Zero);
                Assert.That(agent.NormalizedArmTravel, Is.Zero);
                Assert.That(localBall.magnitude, Is.LessThanOrEqualTo(
                    Dg5fGraspSpec.MaximumSpawnBallDistance + 1e-4f));
                Assert.That(ballCollider.bounds.min.y, Is.EqualTo(agent.pedestalCollider.bounds.max.y).Within(2e-3f));
                Assert.That(Dg5fGraspSpec.IsFinite(localBall), Is.True);
                Assert.That(agent.ball.isKinematic, Is.False, $"Reset {reset}: ball must be dynamic.");
                Assert.That(agent.ball.useGravity, Is.True, $"Reset {reset}: gravity must be restored.");
                Assert.That(Dg5fGraspSpec.IsFinite(agent.ball.linearVelocity), Is.True);
                Assert.That(Dg5fGraspSpec.IsFinite(agent.ball.angularVelocity), Is.True);
                Assert.That(agent.ball.linearVelocity.sqrMagnitude,
                    Is.LessThan(1e-10f), $"Reset {reset}: linear velocity was not cleared.");
                Assert.That(agent.ball.angularVelocity.sqrMagnitude,
                    Is.LessThan(1e-10f), $"Reset {reset}: angular velocity was not cleared.");
                Assert.That(IsFinite(agent.ball.rotation), Is.True,
                    $"Reset {reset}: ball rotation must remain finite.");
                Assert.That(agent.pedestal.position, Is.EqualTo(pedestalPosition));
                Assert.That(agent.pedestal.rotation, Is.EqualTo(pedestalRotation));
                Assert.That(agent.pedestal.localScale, Is.EqualTo(pedestalScale));
                Assert.That(agent.contactSensors.Select(sensor => sensor.IsTouching), Is.All.False,
                    $"Reset {reset}: cached contacts must be cleared.");

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

                foreach (var body in revoluteBodies)
                {
                    float positionDeg = body.jointPosition[0] * Mathf.Rad2Deg;
                    float velocity = body.jointVelocity[0];
                    Assert.That(float.IsNaN(positionDeg) || float.IsInfinity(positionDeg), Is.False,
                        $"Reset {reset}: {body.name} position must be finite.");
                    Assert.That(float.IsNaN(velocity) || float.IsInfinity(velocity), Is.False,
                        $"Reset {reset}: {body.name} velocity must be finite.");
                    Assert.That(velocity, Is.EqualTo(0f).Within(1e-6f),
                        $"Reset {reset}: {body.name} velocity was not cleared.");
                    Assert.That(body.xDrive.target, Is.EqualTo(positionDeg).Within(1e-3f),
                        $"Reset {reset}: {body.name} target and position diverged.");
                }

                for (int arm = 0; arm < Dg5fGraspSpec.ArmJointCount; arm++)
                {
                    Assert.That(agent.CurrentArmTargetDeg(arm),
                        Is.EqualTo(armBodies[arm].xDrive.target).Within(1e-4f),
                        $"Reset {reset}: arm target cache {arm} is out of sync with its drive.");
                }
            }
        }

        static bool IsFinite(Quaternion value)
        {
            return Dg5fGraspSpec.IsFinite(value.x)
                && Dg5fGraspSpec.IsFinite(value.y)
                && Dg5fGraspSpec.IsFinite(value.z)
                && Dg5fGraspSpec.IsFinite(value.w);
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
