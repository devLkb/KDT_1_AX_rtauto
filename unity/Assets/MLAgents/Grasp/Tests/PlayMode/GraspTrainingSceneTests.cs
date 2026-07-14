using System.Collections;
using NUnit.Framework;
using Unity.MLAgents.Actuators;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace KDT.GraspTraining.PlayModeTests
{
    public sealed class GraspTrainingSceneTests
    {
        [UnityTest]
        public IEnumerator TrainingSceneLoadsWithPhysicalBallAndSingleDriveOwner()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;
            yield return new WaitForFixedUpdate();

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            Assert.That(agent, Is.Not.Null);
            Assert.That(agent.ball, Is.Not.Null);
            Assert.That(agent.ball.mass, Is.EqualTo(0.05f).Within(1e-6f));
            Assert.That(agent.ball.useGravity, Is.True);
            Assert.That(agent.contactSensors, Has.Length.EqualTo(Dg5fGraspSpec.FingerCount));
            Assert.That(agent.MaxStep, Is.EqualTo(750), "750 physics steps must allow 15 seconds at 50 Hz.");

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
        public IEnumerator OneHundredResetsKeepJointsFiniteAndBallInsideSpawnRegion()
        {
            SceneManager.LoadScene("DG5F_GraspTraining");
            yield return null;

            var agent = Object.FindAnyObjectByType<Dg5fGraspAgent>();
            Assert.That(agent, Is.Not.Null);

            var closeAction = new float[Dg5fGraspSpec.ActionSize];
            closeAction[6] = 1f;
            agent.OnActionReceived(new ActionBuffers(closeAction, new int[0]));
            Assert.That(agent.CurrentClosure, Is.GreaterThan(0f));
            agent.OnEpisodeBegin();
            Assert.That(agent.CurrentClosure, Is.Zero, "Episode reset must open the hand before writing drives.");

            for (int reset = 0; reset < 100; reset++)
            {
                agent.OnEpisodeBegin();
                yield return new WaitForFixedUpdate();
                Vector3 offset = agent.ball.position - agent.spawnCenter.position;
                Assert.That(Mathf.Abs(offset.x), Is.LessThanOrEqualTo(0.061f));
                Assert.That(Mathf.Abs(offset.z), Is.LessThanOrEqualTo(0.061f));
                Assert.That(float.IsNaN(agent.ball.position.x), Is.False);
            }

            foreach (var body in agent.GetComponentsInChildren<ArticulationBody>())
            {
                if (body.jointType != ArticulationJointType.RevoluteJoint || body.dofCount == 0) continue;
                Assert.That(float.IsNaN(body.jointPosition[0]), Is.False, body.name);
                Assert.That(Mathf.Abs(body.jointVelocity[0]), Is.LessThan(0.01f), body.name);
            }
        }
    }
}
