using System.Collections;
using System.Collections.Generic;
using System.Linq;
using KDT.GraspLiftTraining;
using NUnit.Framework;
using Unity.MLAgents;
using Unity.MLAgents.Policies;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace KDT.GraspLiftTraining.PlayModeTests
{
    /// <summary>
    /// Scene contract plus the physics feasibility probe. The feasibility probe is
    /// the important one: no amount of PPO can learn a grasp the simulation cannot
    /// physically sustain, so "can a closed DG5F hold this block through a lift"
    /// must be answered directly rather than inferred from training curves.
    /// </summary>
    public sealed class GraspLiftSceneTests
    {
        const string SceneName = "DG5F_GraspLiftTraining";

        static IEnumerator LoadScene()
        {
            SceneManager.LoadScene(SceneName);
            yield return null;
            // Let the agents resolve, reset, and release their blocks (the release
            // happens two fixed steps after OnEpisodeBegin).
            for (int i = 0; i < 8; i++) yield return new WaitForFixedUpdate();
        }

        [UnityTest]
        public IEnumerator SceneHasTwentyIndependentTrainingAreas()
        {
            yield return LoadScene();

            var agents = Object.FindObjectsByType<Dg5fGraspLiftAgent>(FindObjectsSortMode.None);
            Assert.That(agents, Has.Length.EqualTo(20));
            Assert.That(agents.Select(a => a.transform.root).Distinct().Count(), Is.EqualTo(20));
            Assert.That(agents.Select(a => a.graspObject).Distinct().Count(), Is.EqualTo(20));
            Assert.That(agents.Select(a => a.pedestal).Distinct().Count(), Is.EqualTo(20));
            Assert.That(agents.Select(a => a.spawnSeed).Distinct().Count(), Is.EqualTo(20));

            foreach (var agent in agents)
            {
                Transform area = agent.transform.root;
                Assert.That(agent.graspObject.transform.IsChildOf(area), Is.True,
                    "each area must own its own block");
                Assert.That(agent.pedestal.IsChildOf(area), Is.True);
                Assert.That(agent.GetComponent<BehaviorParameters>().BehaviorName,
                    Is.EqualTo(Dg5fGraspLiftSpec.BehaviorName));
            }
        }

        [UnityTest]
        public IEnumerator BehaviorParametersMatchTheSpecShape()
        {
            yield return LoadScene();

            var agent = Object.FindAnyObjectByType<Dg5fGraspLiftAgent>();
            var behavior = agent.GetComponent<BehaviorParameters>();
            Assert.That(behavior.BrainParameters.VectorObservationSize,
                Is.EqualTo(Dg5fGraspLiftSpec.ObservationSize));
            Assert.That(behavior.BrainParameters.NumStackedVectorObservations, Is.EqualTo(1));
            Assert.That(behavior.BrainParameters.ActionSpec.NumContinuousActions,
                Is.EqualTo(Dg5fGraspLiftSpec.ActionSize));
            Assert.That(behavior.BrainParameters.ActionSpec.NumDiscreteActions, Is.EqualTo(0));
            Assert.That(agent.GetComponent<DecisionRequester>().DecisionPeriod, Is.EqualTo(5));
            Assert.That(agent.MaxStep, Is.EqualTo(0),
                "episode length is measured in simulation seconds, not agent steps");
        }

        [UnityTest]
        public IEnumerator BlockUsesTheDocumentedPhysicsSetup()
        {
            yield return LoadScene();

            var agent = Object.FindAnyObjectByType<Dg5fGraspLiftAgent>();
            Rigidbody block = agent.graspObject;
            Assert.That(block.mass, Is.EqualTo(Dg5fGraspLiftSpec.CurrentBlockMass).Within(1e-6f));
            Assert.That(block.useGravity, Is.True);
            Assert.That(block.isKinematic, Is.False, "the block must be released after reset");
            Assert.That(block.collisionDetectionMode,
                Is.EqualTo(CollisionDetectionMode.ContinuousDynamic),
                "fast-closing fingers tunnel through a discrete-detection block");

            Collider collider = block.GetComponent<Collider>();
            Assert.That(collider, Is.TypeOf<BoxCollider>());
            // Measure the box itself, not its world AABB: the block spawns with a
            // random yaw, so the axis-aligned bounds are wider than the box.
            var box = (BoxCollider)collider;
            Vector3 size = Vector3.Scale(box.size, block.transform.lossyScale);
            Assert.That(size.x, Is.EqualTo(Dg5fGraspLiftSpec.CurrentBlockWidth).Within(2e-3f));
            Assert.That(size.y, Is.EqualTo(Dg5fGraspLiftSpec.BlockHeight).Within(2e-3f));
            Assert.That(size.z, Is.EqualTo(Dg5fGraspLiftSpec.CurrentBlockWidth).Within(2e-3f));
            Assert.That(collider.material.staticFriction, Is.GreaterThan(1f),
                "the block needs high friction for a friction grasp to hold");
        }

        [UnityTest]
        public IEnumerator BlockSpawnsUprightAndRestingOnThePanel()
        {
            yield return LoadScene();

            foreach (var agent in Object.FindObjectsByType<Dg5fGraspLiftAgent>(
                         FindObjectsSortMode.None))
            {
                Vector3 local = agent.CurrentObjectLocalPosition;
                Assert.That(
                    Dg5fGraspLiftSpec.IsValidSpawn(
                        local, Dg5fGraspLiftSpec.CurrentBlockWidth, Dg5fGraspLiftSpec.BlockHeight),
                    Is.True,
                    $"invalid spawn {local}");
                float tilt = Vector3.Angle(agent.graspObject.transform.up, Vector3.up);
                Assert.That(tilt, Is.LessThan(2f), "the block must start upright");
            }
        }

        [UnityTest]
        public IEnumerator ContactSensorsCoverEveryFingertipAndThePalm()
        {
            yield return LoadScene();

            var agent = Object.FindAnyObjectByType<Dg5fGraspLiftAgent>();
            Assert.That(agent.contactSensors, Is.Not.Empty);
            for (int index = 0; index < Dg5fGraspLiftSpec.ContactPointCount; index++)
            {
                var forIndex = agent.contactSensors
                    .Where(sensor => sensor != null && sensor.contactIndex == index)
                    .ToArray();
                Assert.That(forIndex, Is.Not.Empty, $"contact point {index} is uninstrumented");
                Assert.That(forIndex.All(sensor => sensor.targetObject == agent.graspObject),
                    Is.True);
                // The regression this guards: the legacy GraspContactSensor sat only on
                // the link GameObject, but the URDF importer parents colliders under a
                // "Collisions" child, so OnCollision* never reached it.
                Assert.That(
                    forIndex.Any(sensor => sensor.GetComponent<Collider>() != null),
                    Is.True,
                    $"contact point {index} has no sensor on an actual collider");
            }
        }

        [UnityTest]
        public IEnumerator PanelSafetySensorsSkipTheHand()
        {
            yield return LoadScene();

            var agent = Object.FindAnyObjectByType<Dg5fGraspLiftAgent>();
            Assert.That(agent.safetySensors, Is.Not.Empty);
            foreach (var sensor in agent.safetySensors)
            {
                for (Transform t = sensor.transform; t != null; t = t.parent)
                {
                    Assert.That(t.name.Contains("_dg_"), Is.False,
                        "fingers must be free to work at the table surface");
                }
            }
        }
    }
}
