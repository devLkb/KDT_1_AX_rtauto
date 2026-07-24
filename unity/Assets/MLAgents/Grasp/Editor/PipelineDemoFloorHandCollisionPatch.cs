using System;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace KDT.GraspTraining.Editor
{
    /// <summary>
    /// Temporary teleop-testing convenience: during manual hand teleoperation the palm
    /// often brushes the pedestal/floor before the ball is actually grasped, which (a)
    /// trips GraspSurfaceContactSensor's unsafe-contact episode reset before the grab
    /// even happens, and (b) physically pins/drags the hand along the floor since the
    /// two colliders push each other apart. This puts the ball, pedestal/floor, and hand
    /// on dedicated layers and sets the physics Layer Collision Matrix so:
    ///   floor &lt;-&gt; ball   : collide (unchanged real physics)
    ///   hand  &lt;-&gt; floor  : do NOT collide
    ///   hand  &lt;-&gt; ball   : collide
    /// Trade-off (expected): GraspSurfaceContactSensor relies on OnCollisionEnter/Stay
    /// between the hand and the pedestal collider, so with hand<->floor collision
    /// disabled it will never fire for hand/floor contact anymore. That's the intended
    /// effect here (temporary demo/teleop use), not training data collection.
    /// </summary>
    public static class PipelineDemoFloorHandCollisionPatch
    {
        const string FloorLayerName = "GraspFloor";
        const string BallLayerName = "GraspBall";
        const string HandLayerName = "GraspHand";
        const string HandRootName = "ll_dg_palm";

        [MenuItem("Tools/ML-Agents/Fix Hand-Floor Collision In Pipeline Demo Scene")]
        public static void Run()
        {
            int floorLayer = EnsureLayer(FloorLayerName);
            int ballLayer = EnsureLayer(BallLayerName);
            int handLayer = EnsureLayer(HandLayerName);

            Scene scene = SceneManager.GetSceneByPath(PipelineDemoSceneBuilder.DemoScenePath);
            bool openedHere = false;
            if (!scene.IsValid() || !scene.isLoaded)
            {
                scene = EditorSceneManager.OpenScene(
                    PipelineDemoSceneBuilder.DemoScenePath,
                    OpenSceneMode.Additive);
                openedHere = true;
            }

            Dg5fGraspAgent agent = scene.GetRootGameObjects()
                .Select(go => go.GetComponentInChildren<Dg5fGraspAgent>(true))
                .FirstOrDefault(a => a != null);
            if (agent == null)
                throw new InvalidOperationException(
                    $"No {nameof(Dg5fGraspAgent)} found in {PipelineDemoSceneBuilder.DemoScenePath}.");
            if (agent.ball == null || agent.pedestal == null)
                throw new InvalidOperationException(
                    "[PipelineDemoFloorHandCollisionPatch] agent.ball / agent.pedestal is not set.");

            SetLayerRecursively(agent.ball.transform, ballLayer);
            SetLayerRecursively(agent.pedestal, floorLayer);

            Transform handRoot = agent.robotBase
                .GetComponentsInChildren<Transform>(true)
                .FirstOrDefault(t => t.name == HandRootName);
            if (handRoot == null)
                throw new InvalidOperationException($"Missing transform: {HandRootName}");
            SetLayerRecursively(handRoot, handLayer);

            Physics.IgnoreLayerCollision(floorLayer, ballLayer, false);
            Physics.IgnoreLayerCollision(handLayer, floorLayer, true);
            Physics.IgnoreLayerCollision(handLayer, ballLayer, false);

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            if (openedHere) EditorSceneManager.CloseScene(scene, true);

            Debug.Log(
                "[PipelineDemoFloorHandCollisionPatch] "
                + $"floor='{FloorLayerName}'({floorLayer}) ball='{BallLayerName}'({ballLayer}) "
                + $"hand='{HandLayerName}'({handLayer}); floor<->ball collide, "
                + "hand<->floor ignored, hand<->ball collide.");
        }

        static void SetLayerRecursively(Transform root, int layer)
        {
            foreach (Transform t in root.GetComponentsInChildren<Transform>(true))
            {
                Undo.RecordObject(t.gameObject, "Set physics layer");
                t.gameObject.layer = layer;
            }
        }

        static int EnsureLayer(string name)
        {
            for (int i = 8; i <= 31; i++)
                if (LayerMask.LayerToName(i) == name) return i;

            var tagManagerAssets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset");
            if (tagManagerAssets.Length == 0)
                throw new InvalidOperationException("Could not load ProjectSettings/TagManager.asset.");

            var tagManager = new SerializedObject(tagManagerAssets[0]);
            SerializedProperty layers = tagManager.FindProperty("layers");
            for (int i = 8; i <= 31; i++)
            {
                SerializedProperty layerSp = layers.GetArrayElementAtIndex(i);
                if (string.IsNullOrEmpty(layerSp.stringValue))
                {
                    layerSp.stringValue = name;
                    tagManager.ApplyModifiedProperties();
                    return i;
                }
            }
            throw new InvalidOperationException("No free layer slots (8-31) available.");
        }
    }
}
