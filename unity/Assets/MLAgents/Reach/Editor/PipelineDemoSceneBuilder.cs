using System;
using System.Linq;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace KDT.ReachTraining.Editor
{
    /// <summary>
    /// Builds a single-robot live demo scene from the already-wired
    /// DG5F_GraspPointReachArea_00 training-area instance: keeps its
    /// CameraTargetReceiver + trained Reach model, and re-enables the hand
    /// teleop path (Dg5fReceiver/Dg5fHandDriver) that ArmReachTrainingSceneBuilder
    /// disables for training. ArmTargetIK/HandSliderUI/Dg5fFingerIK/Dg5fThumbIK/
    /// RobotInitialPoseSync are left disabled — HandSliderUI in particular applies
    /// its armJoints every FixedUpdate and would fight the RL agent's direct
    /// xDrive writes on the same joints.
    /// </summary>
    public static class PipelineDemoSceneBuilder
    {
        public const string SourceScenePath =
            "Assets/MLAgents/Reach/DG5F_GraspPointReachTraining.unity";
        public const string SourceAreaName = "DG5F_GraspPointReachArea_00";
        public const string DemoScenePath = "Assets/Scenes/Pipeline_Demo.unity";
        const string HandRootName = "ll_dg_palm";

        static readonly string[] HandTeleopTypeNames =
        {
            "Dg5fReceiver",
            "Dg5fHandDriver",
            "Dg5fJointLogger"
        };

        // Must stay disabled: HandSliderUI re-applies its armJoints every
        // FixedUpdate and would fight the RL agent's direct arm xDrive writes.
        // ArmTargetIK/Dg5fFingerIK/Dg5fThumbIK are the pre-RL/pre-Dg5fHandDriver
        // manual-IK control path and would fight Dg5fHandDriver on the same
        // finger joints. Some of these were found already enabled on individual
        // finger sub-objects in the checked-in training area, so this is forced
        // rather than assumed.
        static readonly string[] ForceDisabledTypeNames =
        {
            "ArmTargetIK",
            "HandSliderUI",
            "Dg5fFingerIK",
            "Dg5fThumbIK",
            "RobotInitialPoseSync"
        };

        [MenuItem("Tools/ML-Agents/Build Pipeline Demo Scene")]
        public static void Build()
        {
            Scene sourceScene = EditorSceneManager.OpenScene(
                SourceScenePath,
                OpenSceneMode.Additive);

            GameObject sourceArea = sourceScene.GetRootGameObjects()
                .FirstOrDefault(go => go.name == SourceAreaName);
            if (sourceArea == null)
            {
                EditorSceneManager.CloseScene(sourceScene, true);
                throw new InvalidOperationException(
                    $"'{SourceAreaName}' not found in {SourceScenePath}.");
            }

            GameObject areaCopy = UnityEngine.Object.Instantiate(sourceArea);
            areaCopy.name = SourceAreaName;

            // Create the demo scene additively (NOT Single) so sourceScene stays
            // loaded until after the copy has been moved out of it — closing
            // sourceScene first (or via NewSceneMode.Single, which tears down
            // every loaded scene) destroys areaCopy along with it.
            Scene demoScene = EditorSceneManager.NewScene(
                NewSceneSetup.DefaultGameObjects,
                NewSceneMode.Additive);
            demoScene.name = "Pipeline_Demo";
            SceneManager.MoveGameObjectToScene(areaCopy, demoScene);

            EditorSceneManager.CloseScene(sourceScene, true);
            EditorSceneManager.SetActiveScene(demoScene);

            ReEnableHandTeleop(areaCopy);
            SetInferenceOnly(areaCopy);

            EnsureFolder("Assets/Scenes");
            EditorSceneManager.SaveScene(demoScene, DemoScenePath);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Selection.activeGameObject = areaCopy;
            Debug.Log(
                $"[PipelineDemoSceneBuilder] Built {DemoScenePath} from "
                + $"{SourceAreaName} (hand teleop re-enabled, arm stays RL-controlled).");
        }

        static void ReEnableHandTeleop(GameObject area)
        {
            foreach (MonoBehaviour behaviour in
                     area.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour == null) continue;
                string typeName = behaviour.GetType().Name;
                if (Array.IndexOf(HandTeleopTypeNames, typeName) >= 0)
                    behaviour.enabled = true;
                else if (Array.IndexOf(ForceDisabledTypeNames, typeName) >= 0)
                    behaviour.enabled = false;
            }

            Transform handRoot = area.GetComponentsInChildren<Transform>(true)
                .FirstOrDefault(t => t.name == HandRootName);
            if (handRoot == null)
                throw new InvalidOperationException(
                    $"Missing transform: {HandRootName}");

            foreach (ArticulationBody body in
                     handRoot.GetComponentsInChildren<ArticulationBody>(true))
            {
                body.enabled = true;
            }
            foreach (Collider collider in
                     handRoot.GetComponentsInChildren<Collider>(true))
            {
                collider.enabled = true;
            }
        }

        static void SetInferenceOnly(GameObject area)
        {
            BehaviorParameters behavior =
                area.GetComponentInChildren<BehaviorParameters>(true);
            if (behavior == null) return;

            // Only force InferenceOnly (which requires a Model) once a matching
            // trained model is actually assigned. With no Model, Default falls
            // back to Heuristic (idle) instead of throwing at Play time.
            behavior.BehaviorType = behavior.Model != null
                ? BehaviorType.InferenceOnly
                : BehaviorType.Default;
        }

        static void EnsureFolder(string path)
        {
            if (AssetDatabase.IsValidFolder(path)) return;
            string parent = System.IO.Path.GetDirectoryName(path)?.Replace('\\', '/');
            string leaf = System.IO.Path.GetFileName(path);
            if (!string.IsNullOrEmpty(parent) && !AssetDatabase.IsValidFolder(parent))
                EnsureFolder(parent);
            AssetDatabase.CreateFolder(parent, leaf);
        }
    }
}
