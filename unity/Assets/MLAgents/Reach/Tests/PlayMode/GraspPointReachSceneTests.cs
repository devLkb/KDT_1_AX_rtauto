using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using NUnit.Framework;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

namespace KDT.ReachTraining.PlayModeTests
{
    public sealed class GraspPointReachSceneTests
    {
        [UnityTest]
        public IEnumerator SceneContainsTwentyIndependentReachAreas()
        {
            yield return LoadTrainingScene();

            Dg5fGraspPointReachAgent[] agents =
                Object.FindObjectsByType<Dg5fGraspPointReachAgent>();
            Assert.That(agents, Has.Length.EqualTo(20));
            Assert.That(
                agents.Select(agent => agent.transform.root).Distinct().Count(),
                Is.EqualTo(20));
            Assert.That(
                agents.Select(agent => agent.target).Distinct().Count(),
                Is.EqualTo(20));
            Assert.That(
                agents.Select(agent => agent.spawnSeed).Distinct().Count(),
                Is.EqualTo(20));

            foreach (Dg5fGraspPointReachAgent agent in agents)
            {
                Assert.That(
                    agent.transform.root.name,
                    Does.StartWith("DG5F_GraspPointReachArea_"));
                Assert.That(
                    agent.GetComponent<BehaviorParameters>().BehaviorName,
                    Is.EqualTo(Dg5fReachSpec.BehaviorName));
                Assert.That(
                    agent.transform.root
                        .GetComponentsInChildren<BehaviorParameters>(true),
                    Has.Length.EqualTo(1));
                Assert.That(agent.target.IsChildOf(agent.transform.root), Is.True);
                Assert.That(agent.robotBase.localPosition, Is.EqualTo(Vector3.zero));
            }
        }

        [UnityTest]
        public IEnumerator PolicyUsesOneGraspPointAndSixArmActionsOnly()
        {
            yield return LoadTrainingScene();

            Dg5fGraspPointReachAgent agent =
                Object.FindAnyObjectByType<Dg5fGraspPointReachAgent>();
            Assert.That(agent, Is.Not.Null);
            Assert.That(agent.graspPoint.name, Is.EqualTo("GraspPoint"));
            Assert.That(agent.graspPoint.parent.name, Is.EqualTo("ll_dg_palm"));
            Assert.That(
                Vector3.Distance(
                    agent.graspPoint.localPosition,
                    Dg5fReachSpec.CalibratedGraspPointLocalPosition),
                Is.LessThan(1e-7f));

            BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
            Assert.That(
                behavior.BrainParameters.VectorObservationSize,
                Is.EqualTo(Dg5fReachSpec.ObservationSize));
            Assert.That(
                behavior.BrainParameters.ActionSpec.NumContinuousActions,
                Is.EqualTo(Dg5fReachSpec.ActionSize));
            Assert.That(
                behavior.BrainParameters.ActionSpec.NumDiscreteActions,
                Is.Zero);

            var requester = agent.GetComponent<Unity.MLAgents.DecisionRequester>();
            Assert.That(requester.DecisionPeriod, Is.EqualTo(5));
            Assert.That(requester.TakeActionsBetweenDecisions, Is.False);

            var sensor = new VectorSensor(Dg5fReachSpec.ObservationSize);
            agent.CollectObservations(sensor);
            float[] observations = ((IEnumerable<float>)typeof(VectorSensor)
                    .GetMethod(
                        "GetObservations",
                        BindingFlags.Instance | BindingFlags.NonPublic)
                    ?.Invoke(sensor, null))
                .ToArray();
            Assert.That(
                observations,
                Has.Length.EqualTo(Dg5fReachSpec.ObservationSize));
            Assert.That(observations, Is.All.Matches<float>(Dg5fReachSpec.IsFinite));

            ArticulationBody[] handBodies =
                agent.graspPoint.parent.GetComponentsInChildren<ArticulationBody>(true);
            Assert.That(handBodies.Length, Is.GreaterThanOrEqualTo(20));
            Assert.That(handBodies, Has.All.Matches<ArticulationBody>(body => !body.enabled));
            Collider[] handColliders =
                agent.graspPoint.parent.GetComponentsInChildren<Collider>(true);
            Assert.That(handColliders, Has.All.Matches<Collider>(collider => !collider.enabled));
            Renderer[] handRenderers =
                agent.graspPoint.parent.GetComponentsInChildren<Renderer>(true);
            Assert.That(handRenderers.Length, Is.GreaterThan(0));
            Assert.That(handRenderers, Has.All.Matches<Renderer>(renderer => renderer.enabled));

            float[] handTargets = handBodies.Select(body => body.xDrive.target).ToArray();
            float[] armTargets = Enumerable.Range(0, Dg5fReachSpec.ActionSize)
                .Select(agent.CurrentArmTargetDeg)
                .ToArray();
            agent.OnActionReceived(
                new ActionBuffers(
                    Enumerable.Repeat(1f, Dg5fReachSpec.ActionSize).ToArray(),
                    System.Array.Empty<int>()));

            for (int index = 0; index < armTargets.Length; index++)
            {
                float delta = agent.CurrentArmTargetDeg(index) - armTargets[index];
                Assert.That(
                    delta,
                    Is.InRange(0f, Dg5fReachSpec.MaximumArmDeltaDegPerDecision));
            }
            Assert.That(
                handBodies.Select(body => body.xDrive.target),
                Is.EqualTo(handTargets),
                "The reach policy must not write any hand drive.");
        }

        [UnityTest]
        public IEnumerator TargetIsStaticTriggerOnTheExpandedPanelPlane()
        {
            yield return LoadTrainingScene();

            foreach (Dg5fGraspPointReachAgent agent in
                     Object.FindObjectsByType<Dg5fGraspPointReachAgent>())
            {
                SphereCollider targetCollider =
                    agent.target.GetComponent<SphereCollider>();
                Assert.That(targetCollider, Is.Not.Null);
                Assert.That(targetCollider.isTrigger, Is.True);
                Assert.That(agent.target.GetComponent<Rigidbody>(), Is.Null);
                Assert.That(
                    targetCollider.bounds.extents.x,
                    Is.EqualTo(Dg5fReachSpec.TargetRadius).Within(1e-4f));

                BoxCollider panel = agent.panelCollider as BoxCollider;
                Assert.That(panel, Is.Not.Null);
                Assert.That(
                    panel.bounds.max.y,
                    Is.EqualTo(agent.transform.root.position.y).Within(1e-4f));

                Vector3 targetLocal = agent.CurrentTargetLocalPosition;
                float radius = new Vector2(targetLocal.x, targetLocal.z).magnitude;
                Assert.That(
                    radius,
                    Is.InRange(
                        Dg5fReachSpec.MinimumTargetRadius - 1e-5f,
                        Dg5fReachSpec.MaximumTargetRadius + 1e-5f));
                Assert.That(
                    targetLocal.y,
                    Is.EqualTo(Dg5fReachSpec.TargetRadius).Within(1e-5f));
                Assert.That(
                    Dg5fReachSpec.IsTargetSphereWithinPanel(
                        agent.target.position,
                        Dg5fReachSpec.TargetRadius,
                        panel),
                    Is.True);
                Assert.That(
                    Vector3.Distance(
                        agent.CurrentTargetLocalPosition,
                        agent.CurrentGraspPointLocalPosition),
                    Is.GreaterThanOrEqualTo(
                        Dg5fReachSpec.MinimumInitialCenterDistance - 1e-5f));

                foreach (Collider robotCollider in agent.robotColliders)
                {
                    Vector3 closest = robotCollider.ClosestPoint(
                        agent.target.position);
                    Assert.That(
                        Vector3.Distance(closest, agent.target.position),
                        Is.GreaterThanOrEqualTo(
                            Dg5fReachSpec.TargetRadius - 1e-5f),
                        $"Target overlaps {robotCollider.name}.");
                }
            }
        }

        static IEnumerator LoadTrainingScene()
        {
            SceneManager.LoadScene("DG5F_GraspPointReachTraining");
            yield return null;
            yield return new WaitForFixedUpdate();
            yield return new WaitForFixedUpdate();
        }
    }
}
