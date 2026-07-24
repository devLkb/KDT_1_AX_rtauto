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
        public IEnumerator SceneContainsTwentyIndependentGraspReadyAreas()
        {
            yield return LoadTrainingScene();
            Dg5fGraspPointReachAgent[] agents =
                Object.FindObjectsByType<Dg5fGraspPointReachAgent>();
            Assert.That(agents, Has.Length.EqualTo(20));
            Assert.That(agents.Select(agent => agent.transform.root).Distinct().Count(),
                Is.EqualTo(20));
            Assert.That(agents.Select(agent => agent.target).Distinct().Count(),
                Is.EqualTo(20));
            Assert.That(agents.Select(agent => agent.spawnSeed).Distinct().Count(),
                Is.EqualTo(20));
            Assert.That(agents, Has.All.Matches<Dg5fGraspPointReachAgent>(agent =>
                agent.endEpisodeOnLock));
        }

        [UnityTest]
        public IEnumerator PolicyUsesThirtySevenObservationsAndSixArmActions()
        {
            yield return LoadTrainingScene();
            Dg5fGraspPointReachAgent agent =
                Object.FindAnyObjectByType<Dg5fGraspPointReachAgent>();
            BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
            Assert.That(behavior.BehaviorName, Is.EqualTo(Dg5fReachSpec.BehaviorName));
            Assert.That(behavior.BrainParameters.VectorObservationSize,
                Is.EqualTo(Dg5fReachSpec.ObservationSize));
            Assert.That(behavior.BrainParameters.ActionSpec.NumContinuousActions,
                Is.EqualTo(Dg5fReachSpec.ActionSize));
            Assert.That(behavior.BrainParameters.ActionSpec.NumDiscreteActions, Is.Zero);

            var requester = agent.GetComponent<Unity.MLAgents.DecisionRequester>();
            Assert.That(requester.DecisionPeriod,
                Is.EqualTo(Dg5fReachSpec.DecisionPeriod));
            Assert.That(requester.TakeActionsBetweenDecisions, Is.False);

            var sensor = new VectorSensor(Dg5fReachSpec.ObservationSize);
            agent.CollectObservations(sensor);
            float[] observations = ((IEnumerable<float>)typeof(VectorSensor)
                    .GetMethod(
                        "GetObservations",
                        BindingFlags.Instance | BindingFlags.NonPublic)
                    ?.Invoke(sensor, null))
                .ToArray();
            Assert.That(observations, Has.Length.EqualTo(37));
            Assert.That(observations,
                Is.All.Matches<float>(Dg5fReachSpec.IsFinite));
        }

        [UnityTest]
        public IEnumerator HandPhysicsIsEnabledAndArmActionsCannotWriteHandTargets()
        {
            yield return LoadTrainingScene();
            Dg5fGraspPointReachAgent agent =
                Object.FindAnyObjectByType<Dg5fGraspPointReachAgent>();
            ArticulationBody[] handBodies = agent.palm
                .GetComponentsInChildren<ArticulationBody>(true)
                .Where(body => body.name.Contains("_dg_") && body.dofCount > 0)
                .ToArray();
            Assert.That(handBodies, Has.Length.EqualTo(20));
            Assert.That(handBodies,
                Has.All.Matches<ArticulationBody>(body => body.enabled));
            Collider[] handColliders = agent.palm
                .GetComponentsInChildren<Collider>(true);
            Assert.That(handColliders, Is.Not.Empty);
            Assert.That(handColliders,
                Has.All.Matches<Collider>(collider =>
                    collider.enabled && !collider.isTrigger));

            float[] before = handBodies.Select(body => body.xDrive.target).ToArray();
            agent.OnActionReceived(new ActionBuffers(
                Enumerable.Repeat(1f, Dg5fReachSpec.ActionSize).ToArray(),
                System.Array.Empty<int>()));
            yield return new WaitForFixedUpdate();
            Assert.That(handBodies.Select(body => body.xDrive.target),
                Is.EqualTo(before));
        }

        [UnityTest]
        public IEnumerator EveryMovingRobotColliderHasPanelSafetySensor()
        {
            yield return LoadTrainingScene();
            Dg5fGraspPointReachAgent agent =
                Object.FindAnyObjectByType<Dg5fGraspPointReachAgent>();
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
            foreach (Collider collider in movingColliders)
            {
                ReachSurfaceContactSensor sensor =
                    collider.GetComponent<ReachSurfaceContactSensor>();
                Assert.That(sensor, Is.Not.Null, collider.name);
                Assert.That(sensor.agent, Is.SameAs(agent));
                Assert.That(sensor.unsafeSurface, Is.SameAs(agent.panelCollider));
            }
            Assert.That(agent.safetySensors.Length,
                Is.EqualTo(movingColliders.Length));
        }

        [UnityTest]
        public IEnumerator TargetIsSolidKinematicBallOnPanel()
        {
            yield return LoadTrainingScene();
            foreach (Dg5fGraspPointReachAgent agent in
                     Object.FindObjectsByType<Dg5fGraspPointReachAgent>())
            {
                SphereCollider targetCollider =
                    agent.target.GetComponent<SphereCollider>();
                Assert.That(targetCollider, Is.Not.Null);
                Assert.That(targetCollider.isTrigger, Is.False);
                Assert.That(targetCollider.bounds.extents.x,
                    Is.EqualTo(Dg5fReachSpec.TargetRadius).Within(1e-4f));
                Assert.That(agent.targetBody, Is.Not.Null);
                Assert.That(agent.targetBody.isKinematic, Is.True);
                Assert.That(agent.targetBody.useGravity, Is.False);
                Assert.That(agent.CurrentTargetLocalPosition.y,
                    Is.EqualTo(Dg5fReachSpec.TargetRadius).Within(1e-5f));
                Assert.That(Dg5fReachSpec.IsTargetSphereWithinPanel(
                    agent.target.position,
                    Dg5fReachSpec.TargetRadius,
                    agent.panelCollider), Is.True);
                Assert.That(agent.CurrentPreGraspPoint,
                    Is.EqualTo(agent.target.position
                        + Vector3.up * Dg5fReachSpec.PreGraspHeight));
            }
        }

        [UnityTest]
        public IEnumerator GraspPointAndOpenHandTargetsAreStableAcrossReset()
        {
            yield return LoadTrainingScene();
            Dg5fGraspPointReachAgent agent =
                Object.FindAnyObjectByType<Dg5fGraspPointReachAgent>();
            Assert.That(agent.graspPoint.parent, Is.EqualTo(agent.palm));
            Assert.That(agent.graspPoint.localPosition,
                Is.EqualTo(Dg5fReachSpec.CalibratedGraspPointLocalPosition));
            float[] first = Enumerable.Range(0, Dg5fReachSpec.HandJointCount)
                .Select(agent.OpenHandTargetDeg)
                .ToArray();
            agent.EndEpisode();
            yield return null;
            yield return new WaitForFixedUpdate();
            Assert.That(Enumerable.Range(0, Dg5fReachSpec.HandJointCount)
                    .Select(agent.OpenHandTargetDeg),
                Is.EqualTo(first));
            Assert.That(agent.CurrentPhase, Is.EqualTo(ReachPhase.Transit));
            Assert.That(agent.IsArmLocked, Is.False);
        }

        [UnityTest]
        public IEnumerator DeploymentLockHoldsArmUntilExplicitRelease()
        {
            yield return LoadTrainingScene();
            Dg5fGraspPointReachAgent agent =
                Object.FindAnyObjectByType<Dg5fGraspPointReachAgent>();
            agent.endEpisodeOnLock = false;
            float[] lockedTargets = Enumerable.Range(0, Dg5fReachSpec.ArmJointCount)
                .Select(agent.CurrentArmTargetDeg)
                .ToArray();

            typeof(Dg5fGraspPointReachAgent)
                .GetMethod("LockArm", BindingFlags.Instance | BindingFlags.NonPublic)
                ?.Invoke(agent, null);
            Assert.That(agent.IsArmLocked, Is.True);

            agent.OnActionReceived(new ActionBuffers(
                Enumerable.Repeat(1f, Dg5fReachSpec.ActionSize).ToArray(),
                System.Array.Empty<int>()));
            yield return new WaitForFixedUpdate();
            Assert.That(Enumerable.Range(0, Dg5fReachSpec.ArmJointCount)
                    .Select(agent.CurrentArmTargetDeg),
                Is.EqualTo(lockedTargets));

            agent.ReleaseArmLock();
            Assert.That(agent.IsArmLocked, Is.False);
            Assert.That(agent.CurrentPhase, Is.EqualTo(ReachPhase.Transit));
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
