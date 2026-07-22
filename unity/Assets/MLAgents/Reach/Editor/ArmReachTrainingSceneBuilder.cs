using System;
using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace KDT.ReachTraining.Editor
{
    public static class ArmReachTrainingSceneBuilder
    {
        public const string SourceRobotPath =
            "Assets/Robots/Prefabs/ur5e_dg5f_left.prefab";
        public const string TrainingRoot = "Assets/MLAgents/Reach";
        public const string TrainingPrefabPath = TrainingRoot + "/TrainingArea.prefab";
        public const string TrainingScenePath =
            TrainingRoot + "/DG5F_GraspPointReachTraining.unity";
        public const string TargetMaterialPath = TrainingRoot + "/ReachTarget.mat";
        public const string PanelMaterialPath = TrainingRoot + "/ReachPanel.mat";

        public const int TrainingAreaCount = 20;
        public const int TrainingAreaColumns = 4;
        public const float TrainingAreaSpacing = 3f;

        static readonly HashSet<string> CompetingDriverTypes = new HashSet<string>
        {
            "Dg5fReceiver",
            "Dg5fHandDriver",
            "Dg5fFingerIK",
            "Dg5fThumbIK",
            "Dg5fJointLogger",
            "HandSliderUI",
            "ArmTargetIK",
            "RobotInitialPoseSync"
        };

        [MenuItem("Tools/ML-Agents/Build DG5F GraspPoint Reach Scene")]
        public static void Build()
        {
            EnsureFolder(TrainingRoot);
            GameObject sourceRobot =
                AssetDatabase.LoadAssetAtPath<GameObject>(SourceRobotPath);
            if (sourceRobot == null)
                throw new InvalidOperationException(
                    $"Missing robot prefab: {SourceRobotPath}");

            Scene scene = EditorSceneManager.NewScene(
                NewSceneSetup.DefaultGameObjects,
                NewSceneMode.Single);
            scene.name = "DG5F_GraspPointReachTraining";

            var area = new GameObject("DG5F_GraspPointReachArea");
            var robot = (GameObject)PrefabUtility.InstantiatePrefab(
                sourceRobot,
                area.transform);
            robot.name = "UR5e_DG5F_ArmAgent";
            robot.transform.SetLocalPositionAndRotation(
                Vector3.zero,
                Quaternion.identity);

            DisableCompetingDrivers(robot);
            ConfigureArmPhysics(robot);

            GameObject panel = CreatePanel(area.transform);
            Transform target = CreateTarget(area.transform);
            Transform handRoot = FindTransform(robot, "ll_dg_palm");
            ConfigureOpenHandPhysics(handRoot);
            Transform graspPoint = CreateGraspPoint(handRoot);

            var agent = robot.GetComponent<Dg5fGraspPointReachAgent>();
            if (agent == null)
                agent = robot.AddComponent<Dg5fGraspPointReachAgent>();
            agent.robotBase = robot.transform;
            agent.graspPoint = graspPoint;
            agent.graspPointBody = FindArticulationBody(robot, "tool0");
            agent.target = target;
            agent.targetBody = target.GetComponent<Rigidbody>();
            agent.palm = handRoot;
            agent.panelCollider = panel.GetComponent<BoxCollider>();
            agent.targetCenterLocalY = Dg5fReachSpec.TargetRadius;
            agent.endEpisodeOnLock = true;
            agent.MaxStep = 0;
            agent.safetySensors = ConfigureSafetySensors(
                robot,
                agent.panelCollider,
                agent);

            var behavior = robot.GetComponent<BehaviorParameters>();
            if (behavior == null) behavior = robot.AddComponent<BehaviorParameters>();
            behavior.BehaviorName = Dg5fReachSpec.BehaviorName;
            behavior.BehaviorType = BehaviorType.Default;
            behavior.BrainParameters.VectorObservationSize =
                Dg5fReachSpec.ObservationSize;
            behavior.BrainParameters.NumStackedVectorObservations = 1;
            behavior.BrainParameters.ActionSpec =
                ActionSpec.MakeContinuous(Dg5fReachSpec.ActionSize);
            behavior.BrainParameters.VectorActionDescriptions =
                ArmActionDescriptions();

            var requester = robot.GetComponent<DecisionRequester>();
            if (requester == null) requester = robot.AddComponent<DecisionRequester>();
            requester.DecisionPeriod = Dg5fReachSpec.DecisionPeriod;
            requester.TakeActionsBetweenDecisions = false;

            PrefabUtility.SaveAsPrefabAssetAndConnect(
                area,
                TrainingPrefabPath,
                InteractionMode.AutomatedAction);
            PopulateTrainingAreas(area);
            ConfigureCamera(LayoutCenter());
            Selection.activeGameObject = area;

            EditorSceneManager.SaveScene(scene, TrainingScenePath);
            ReplaceTrainingSceneInBuildSettings();
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Debug.Log(
                $"[ArmReachTrainingSceneBuilder] Built {TrainingPrefabPath} "
                + $"and {TrainingScenePath}");
        }

        static void PopulateTrainingAreas(GameObject firstArea)
        {
            GameObject trainingPrefab =
                AssetDatabase.LoadAssetAtPath<GameObject>(TrainingPrefabPath);
            if (trainingPrefab == null)
                throw new InvalidOperationException(
                    $"Missing generated training prefab: {TrainingPrefabPath}");

            ConfigureTrainingAreaInstance(firstArea, 0);
            for (int index = 1; index < TrainingAreaCount; index++)
            {
                var instance =
                    (GameObject)PrefabUtility.InstantiatePrefab(trainingPrefab);
                ConfigureTrainingAreaInstance(instance, index);
            }
        }

        static void ConfigureTrainingAreaInstance(GameObject area, int index)
        {
            int row = index / TrainingAreaColumns;
            int column = index % TrainingAreaColumns;
            area.name = $"DG5F_GraspPointReachArea_{index:00}";
            area.transform.SetPositionAndRotation(
                new Vector3(
                    column * TrainingAreaSpacing,
                    row * TrainingAreaSpacing,
                    0f),
                Quaternion.identity);

            Dg5fGraspPointReachAgent agent =
                area.GetComponentInChildren<Dg5fGraspPointReachAgent>(true);
            if (agent == null)
                throw new InvalidOperationException(
                    $"Training area {index} has no "
                    + $"{nameof(Dg5fGraspPointReachAgent)}.");
            agent.spawnSeed = 12345 + index;
        }

        static string[] ArmActionDescriptions()
        {
            return new[]
            {
                "shoulder_pan_delta",
                "shoulder_lift_delta",
                "elbow_delta",
                "wrist_1_delta",
                "wrist_2_delta",
                "wrist_3_delta"
            };
        }

        static Vector3 LayoutCenter()
        {
            int rows = Mathf.CeilToInt(
                (float)TrainingAreaCount / TrainingAreaColumns);
            return new Vector3(
                (TrainingAreaColumns - 1) * TrainingAreaSpacing * 0.5f,
                (rows - 1) * TrainingAreaSpacing * 0.5f,
                0f);
        }

        static void DisableCompetingDrivers(GameObject robot)
        {
            foreach (MonoBehaviour behaviour in
                     robot.GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour != null
                    && CompetingDriverTypes.Contains(behaviour.GetType().Name))
                {
                    behaviour.enabled = false;
                }
            }
        }

        static void ConfigureArmPhysics(GameObject robot)
        {
            foreach (ArticulationBody body in
                     robot.GetComponentsInChildren<ArticulationBody>(true))
            {
                body.useGravity = false;
                if (body.jointType != ArticulationJointType.RevoluteJoint
                    || body.name.Contains("_dg_"))
                {
                    continue;
                }

                ArticulationDrive drive = body.xDrive;
                drive.forceLimit = body.name.StartsWith(
                    "wrist_",
                    StringComparison.Ordinal)
                    ? 28f
                    : 150f;
                body.xDrive = drive;
            }
        }

        static void ConfigureOpenHandPhysics(Transform handRoot)
        {
            foreach (ArticulationBody body in
                     handRoot.GetComponentsInChildren<ArticulationBody>(true))
            {
                body.enabled = true;
                body.useGravity = false;
            }

            foreach (Collider collider in
                     handRoot.GetComponentsInChildren<Collider>(true))
            {
                collider.enabled = true;
                collider.isTrigger = false;
            }

            foreach (Renderer renderer in
                     handRoot.GetComponentsInChildren<Renderer>(true))
            {
                renderer.enabled = true;
            }
        }

        static ReachSurfaceContactSensor[] ConfigureSafetySensors(
            GameObject robot,
            Collider panel,
            Dg5fGraspPointReachAgent agent)
        {
            var sensors = new List<ReachSurfaceContactSensor>();
            foreach (Collider collider in
                     robot.GetComponentsInChildren<Collider>(true))
            {
                if (collider == null || !collider.enabled || collider.isTrigger)
                    continue;
                ArticulationBody body =
                    collider.GetComponentInParent<ArticulationBody>();
                if (body == null || body.isRoot) continue;

                ReachSurfaceContactSensor sensor =
                    collider.GetComponent<ReachSurfaceContactSensor>();
                if (sensor == null)
                    sensor = collider.gameObject.AddComponent<ReachSurfaceContactSensor>();
                sensor.agent = agent;
                sensor.unsafeSurface = panel;
                sensors.Add(sensor);
            }
            if (sensors.Count == 0)
                throw new InvalidOperationException(
                    "No moving robot colliders were available for panel safety.");
            return sensors.ToArray();
        }

        static GameObject CreatePanel(Transform parent)
        {
            var panel = GameObject.CreatePrimitive(PrimitiveType.Cube);
            panel.name = "ReachPanel";
            panel.transform.SetParent(parent, false);
            panel.transform.SetLocalPositionAndRotation(
                Vector3.down * Dg5fReachSpec.PanelThickness * 0.5f,
                Quaternion.identity);
            panel.transform.localScale = new Vector3(
                Dg5fReachSpec.PanelWidth,
                Dg5fReachSpec.PanelThickness,
                Dg5fReachSpec.PanelDepth);
            panel.GetComponent<Renderer>().sharedMaterial =
                GetOrCreateMaterial(
                    PanelMaterialPath,
                    "ReachPanel",
                    new Color(0.18f, 0.22f, 0.27f));
            return panel;
        }

        static Transform CreateTarget(Transform parent)
        {
            var target = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            target.name = "ReachTarget";
            target.transform.SetParent(parent, false);
            target.transform.localPosition = new Vector3(
                0.35f,
                Dg5fReachSpec.TargetRadius,
                0.25f);
            target.transform.localScale =
                Vector3.one * (Dg5fReachSpec.TargetRadius * 2f);
            SphereCollider collider = target.GetComponent<SphereCollider>();
            collider.isTrigger = false;
            var body = target.AddComponent<Rigidbody>();
            body.isKinematic = true;
            body.useGravity = false;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
            target.GetComponent<Renderer>().sharedMaterial =
                GetOrCreateMaterial(TargetMaterialPath, "ReachTarget", Color.red);
            return target.transform;
        }

        static Transform CreateGraspPoint(Transform palm)
        {
            Transform existing = palm.Find("GraspPoint");
            if (existing != null)
                UnityEngine.Object.DestroyImmediate(existing.gameObject);

            var graspPoint = new GameObject("GraspPoint").transform;
            graspPoint.SetParent(palm, false);
            graspPoint.localPosition =
                Dg5fReachSpec.CalibratedGraspPointLocalPosition;
            graspPoint.localRotation = Quaternion.identity;
            return graspPoint;
        }

        static Material GetOrCreateMaterial(
            string path,
            string name,
            Color color)
        {
            Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material != null) return material;

            Shader shader =
                Shader.Find("Universal Render Pipeline/Lit")
                ?? Shader.Find("Standard");
            material = new Material(shader) { name = name, color = color };
            AssetDatabase.CreateAsset(material, path);
            return material;
        }

        static Transform FindTransform(GameObject root, string name)
        {
            Transform found = root.GetComponentsInChildren<Transform>(true)
                .FirstOrDefault(item => item.name == name);
            if (found == null)
                throw new InvalidOperationException($"Missing transform: {name}");
            return found;
        }

        static ArticulationBody FindArticulationBody(
            GameObject root,
            string name)
        {
            ArticulationBody found =
                root.GetComponentsInChildren<ArticulationBody>(true)
                    .FirstOrDefault(item => item.name == name);
            if (found == null)
                throw new InvalidOperationException(
                    $"Missing articulation body: {name}");
            return found;
        }

        static void ConfigureCamera(Vector3 focus)
        {
            Camera camera = UnityEngine.Object.FindAnyObjectByType<Camera>();
            if (camera == null) return;
            camera.transform.position = focus + Vector3.back * 18f;
            camera.transform.LookAt(focus);
        }

        static void ReplaceTrainingSceneInBuildSettings()
        {
            EditorBuildSettingsScene[] scenes = EditorBuildSettings.scenes
                .Where(item =>
                    !item.path.EndsWith(
                        "/DG5F_GraspTraining.unity",
                        StringComparison.Ordinal)
                    && !string.Equals(
                        item.path,
                        TrainingScenePath,
                        StringComparison.Ordinal))
                .Concat(new[]
                {
                    new EditorBuildSettingsScene(TrainingScenePath, true)
                })
                .ToArray();
            EditorBuildSettings.scenes = scenes;
        }

        static void EnsureFolder(string path)
        {
            string[] parts = path.Split('/');
            string current = parts[0];
            for (int index = 1; index < parts.Length; index++)
            {
                string next = current + "/" + parts[index];
                if (!AssetDatabase.IsValidFolder(next))
                    AssetDatabase.CreateFolder(current, parts[index]);
                current = next;
            }
        }
    }
}
