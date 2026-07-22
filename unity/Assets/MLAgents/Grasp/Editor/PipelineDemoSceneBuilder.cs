using System;
using System.Linq;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace KDT.GraspTraining.Editor
{
    /// <summary>
    /// Builds the single-robot live pipeline scene from the already-trained
    /// DG5FGrasp behavior (57 obs / 7 action, arm + grip closure): duplicates
    /// DG5F_GraspTrainingArea_00 out of the 20-parallel training scene, re-enables
    /// the hand teleop path (Dg5fReceiver/Dg5fHandDriver) that GraspTrainingSceneBuilder
    /// disables for training, and adds a CameraTargetReceiver that feeds the ball's
    /// one-shot spawn position (Dg5fGraspAgent.cameraReceiver) instead of continuously
    /// overwriting a Transform — GraspBall is a real simulated Rigidbody, not a
    /// kinematic marker, so it must stay physics-driven between episode resets.
    /// Dg5fGraspAgent.driveHandJoints is set to false so the trained policy only
    /// actuates the 6 arm joints; the 20 finger joints are left to Dg5fHandDriver.
    /// ArmTargetIK/HandSliderUI/Dg5fFingerIK/Dg5fFingerIKMode/Dg5fIKVectorDebug/
    /// Dg5fThumbIK/RobotInitialPoseSync are force-disabled — some finger-level
    /// instances were found already enabled on the checked-in training area, which
    /// would fight Dg5fHandDriver once hand physics is re-enabled.
    /// </summary>
    public static class PipelineDemoSceneBuilder
    {
        public const string SourceScenePath =
            "Assets/MLAgents/Grasp/DG5F_GraspTraining.unity";
        public const string SourceAreaName = "DG5F_GraspTrainingArea_00";
        public const string DemoScenePath = "Assets/Scenes/Pipeline_Demo.unity";
        const string HandRootName = "ll_dg_palm";
        const int CameraReceiverPort = 5007;

        static readonly string[] HandTeleopTypeNames =
        {
            "Dg5fReceiver",
            "Dg5fHandDriver",
            "Dg5fJointLogger"
        };

        // Must stay disabled: HandSliderUI/ArmTargetIK would fight the RL agent's
        // direct arm xDrive writes, and the finger-level IK scripts would fight
        // Dg5fHandDriver on the same finger joints once hand physics is re-enabled.
        static readonly string[] ForceDisabledTypeNames =
        {
            "ArmTargetIK",
            "HandSliderUI",
            "Dg5fFingerIK",
            "Dg5fFingerIKMode",
            "Dg5fIKVectorDebug",
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

            // Additive (not Single) so sourceScene stays loaded until the copy has
            // been moved out of it — closing it first destroys areaCopy along with it.
            Scene demoScene = EditorSceneManager.NewScene(
                NewSceneSetup.DefaultGameObjects,
                NewSceneMode.Additive);
            demoScene.name = "Pipeline_Demo";
            SceneManager.MoveGameObjectToScene(areaCopy, demoScene);

            EditorSceneManager.CloseScene(sourceScene, true);
            EditorSceneManager.SetActiveScene(demoScene);

            // Now that demoScene exists alongside whatever else is loaded, it's
            // always safe to close a stale scene at the same path (never the
            // last loaded scene at this point).
            CloseExistingDemoSceneIfLoaded(demoScene);

            ReEnableHandTeleop(areaCopy);
            CameraTargetReceiver cameraReceiver = ConfigureCameraReceiver(areaCopy);
            ConfigureAgent(areaCopy, cameraReceiver);
            SetInferenceOnly(areaCopy);

            EnsureFolder("Assets/Scenes");
            if (!EditorSceneManager.SaveScene(demoScene, DemoScenePath))
                throw new InvalidOperationException($"Failed to save {DemoScenePath}.");
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Selection.activeGameObject = areaCopy;
            Debug.Log(
                $"[PipelineDemoSceneBuilder] Built {DemoScenePath} from "
                + $"{SourceAreaName} (Grasp-based; hand teleop re-enabled, "
                + "arm+grip stays RL with driveHandJoints=false, ball spawn "
                + "wired to CameraTargetReceiver).");
        }

        static void CloseExistingDemoSceneIfLoaded(Scene except)
        {
            for (int i = SceneManager.sceneCount - 1; i >= 0; i--)
            {
                Scene s = SceneManager.GetSceneAt(i);
                if (s != except && s.path == DemoScenePath)
                    EditorSceneManager.CloseScene(s, true);
            }
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

        static CameraTargetReceiver ConfigureCameraReceiver(GameObject area)
        {
            Dg5fGraspAgent agent = area.GetComponentInChildren<Dg5fGraspAgent>(true);
            if (agent == null)
                throw new InvalidOperationException(
                    $"Missing {nameof(Dg5fGraspAgent)} on {area.name}.");

            CameraTargetReceiver receiver =
                agent.gameObject.GetComponent<CameraTargetReceiver>();
            if (receiver == null)
                receiver = agent.gameObject.AddComponent<CameraTargetReceiver>();

            receiver.port = CameraReceiverPort;
            receiver.robotBase = agent.robotBase;
            receiver.target = null;
            receiver.continuousApply = false;
            receiver.inputIsCameraSpace = false;
            receiver.clampToWorkspace = true;
            receiver.minRadius = Dg5fGraspSpec.V1MinimumSpawnRadius;
            receiver.maxRadius = Dg5fGraspSpec.V1MaximumSpawnRadius;
            receiver.logToConsole = true;
            return receiver;
        }

        static void ConfigureAgent(GameObject area, CameraTargetReceiver cameraReceiver)
        {
            Dg5fGraspAgent agent = area.GetComponentInChildren<Dg5fGraspAgent>(true);
            agent.cameraReceiver = cameraReceiver;
            agent.driveHandJoints = false;
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
