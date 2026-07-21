using System;
using System.Collections.Generic;
using System.Linq;
using KDT.GraspTraining;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace KDT.GraspTraining.Editor
{
    public static class GraspTrainingSceneBuilder
    {
        const string SourceRobotPath = "Assets/Robots/Prefabs/ur5e_dg5f_left.prefab";
        const string TrainingRoot = "Assets/MLAgents/Grasp";
        const string TrainingPrefabPath = TrainingRoot + "/TrainingArea.prefab";
        const string TrainingScenePath = TrainingRoot + "/DG5F_GraspTraining.unity";
        const string BallMaterialPath = TrainingRoot + "/GraspBall.mat";
        const string PhysicsMaterialPath = TrainingRoot + "/GraspSurface.physicMaterial";
        const int TrainingAreaCount = 20;
        const int TrainingAreaColumns = 4;
        const float TrainingAreaSpacing = 3f;

        static readonly HashSet<string> CompetingDriverTypes = new HashSet<string>
        {
            "Dg5fReceiver",
            "Dg5fHandDriver",
            "Dg5fThumbIK",
            "Dg5fJointLogger",
            "HandSliderUI",
            "ArmTargetIK",
            "RobotInitialPoseSync"
        };

        [MenuItem("Tools/ML-Agents/Build DG5F Grasp Training Scene")]
        public static void Build()
        {
            EnsureFolder(TrainingRoot);
            var sourceRobot = AssetDatabase.LoadAssetAtPath<GameObject>(SourceRobotPath);
            if (sourceRobot == null)
                throw new InvalidOperationException($"Missing robot prefab: {SourceRobotPath}");

            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            scene.name = "DG5F_GraspTraining";

            var area = new GameObject("DG5F_GraspTrainingArea");
            var robot = (GameObject)PrefabUtility.InstantiatePrefab(sourceRobot, area.transform);
            robot.name = "UR5e_DG5F_Agent";
            robot.transform.SetLocalPositionAndRotation(
                Vector3.up * Dg5fGraspSpec.PanelThickness,
                Quaternion.identity);
            DisableCompetingDrivers(robot);
            ConfigureJointDrives(robot);

            PhysicsMaterial surface = GetOrCreatePhysicsMaterial();
            GameObject pedestal = CreatePedestal(area.transform, surface);
            Rigidbody ball = CreateBall(area.transform, surface);

            Transform palm = FindTransform(robot, "ll_dg_palm");
            var tips = new Transform[Dg5fGraspSpec.FingerCount];
            for (int finger = 0; finger < tips.Length; finger++)
                tips[finger] = FindTransform(robot, $"ll_dg_{finger + 1}_tip");
            Transform graspPoint = CreateGraspPoint(palm);

            var sensors = new GraspContactSensor[Dg5fGraspSpec.FingerCount];
            for (int finger = 0; finger < sensors.Length; finger++)
            {
                sensors[finger] = tips[finger].GetComponent<GraspContactSensor>();
                if (sensors[finger] == null) sensors[finger] = tips[finger].gameObject.AddComponent<GraspContactSensor>();
                sensors[finger].fingerIndex = finger;
                sensors[finger].targetBall = ball;
            }

            var agent = robot.GetComponent<Dg5fGraspAgent>();
            if (agent == null) agent = robot.AddComponent<Dg5fGraspAgent>();
            agent.ball = ball;
            agent.pedestal = pedestal.transform;
            agent.pedestalCollider = pedestal.GetComponent<Collider>();
            agent.robotBase = robot.transform;
            agent.palm = palm;
            agent.graspPoint = graspPoint;
            agent.fingerTips = tips;
            agent.contactSensors = sensors;
            agent.MaxStep = 0;

            var behavior = robot.GetComponent<BehaviorParameters>();
            if (behavior == null) behavior = robot.AddComponent<BehaviorParameters>();
            behavior.BehaviorName = Dg5fGraspSpec.BehaviorName;
            behavior.BehaviorType = BehaviorType.Default;
            behavior.BrainParameters.VectorObservationSize = Dg5fGraspSpec.ObservationSize;
            behavior.BrainParameters.NumStackedVectorObservations = 1;
            behavior.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(Dg5fGraspSpec.ActionSize);
            behavior.BrainParameters.VectorActionDescriptions = new[]
            {
                "shoulder_pan_delta", "shoulder_lift_delta", "elbow_delta",
                "wrist_1_delta", "wrist_2_delta", "wrist_3_delta", "grip_delta"
            };

            var requester = robot.GetComponent<DecisionRequester>();
            if (requester == null) requester = robot.AddComponent<DecisionRequester>();
            requester.DecisionPeriod = 5;
            requester.TakeActionsBetweenDecisions = false;

            PrefabUtility.SaveAsPrefabAssetAndConnect(area, TrainingPrefabPath, InteractionMode.AutomatedAction);
            PopulateTrainingAreas(area);
            ConfigureCamera(LayoutCenter());
            Selection.activeGameObject = area;

            EditorSceneManager.SaveScene(scene, TrainingScenePath);
            AddSceneToBuildSettings(TrainingScenePath);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Debug.Log($"[GraspTrainingSceneBuilder] Built {TrainingPrefabPath} and {TrainingScenePath}");
        }

        static void PopulateTrainingAreas(GameObject firstArea)
        {
            GameObject trainingPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(TrainingPrefabPath);
            if (trainingPrefab == null)
                throw new InvalidOperationException($"Missing generated training prefab: {TrainingPrefabPath}");

            ConfigureTrainingAreaInstance(firstArea, 0);
            for (int index = 1; index < TrainingAreaCount; index++)
            {
                var instance = (GameObject)PrefabUtility.InstantiatePrefab(trainingPrefab);
                ConfigureTrainingAreaInstance(instance, index);
            }
        }

        static void ConfigureTrainingAreaInstance(GameObject area, int index)
        {
            int row = index / TrainingAreaColumns;
            int column = index % TrainingAreaColumns;
            area.name = $"DG5F_GraspTrainingArea_{index:00}";
            area.transform.SetPositionAndRotation(
                new Vector3(column * TrainingAreaSpacing, row * TrainingAreaSpacing, 0f),
                Quaternion.identity);

            var agent = area.GetComponentInChildren<Dg5fGraspAgent>(true);
            if (agent == null)
                throw new InvalidOperationException($"Training area {index} has no {nameof(Dg5fGraspAgent)}.");
            agent.spawnSeed = 12345 + index;
        }

        static Vector3 LayoutCenter()
        {
            int rows = Mathf.CeilToInt((float)TrainingAreaCount / TrainingAreaColumns);
            return new Vector3(
                (TrainingAreaColumns - 1) * TrainingAreaSpacing * 0.5f,
                (rows - 1) * TrainingAreaSpacing * 0.5f,
                0f);
        }

        static void DisableCompetingDrivers(GameObject robot)
        {
            foreach (var behaviour in robot.GetComponents<MonoBehaviour>())
                if (behaviour != null && CompetingDriverTypes.Contains(behaviour.GetType().Name))
                    behaviour.enabled = false;
        }

        static void ConfigureJointDrives(GameObject robot)
        {
            foreach (var body in robot.GetComponentsInChildren<ArticulationBody>(true))
            {
                if (body.jointType != ArticulationJointType.RevoluteJoint) continue;
                var drive = body.xDrive;
                bool hand = body.name.Contains("_dg_");
                if (hand)
                {
                    drive.stiffness = 1000f;
                    drive.damping = 100f;
                    drive.forceLimit = 7.5f;
                }
                else
                {
                    drive.forceLimit = body.name.StartsWith("wrist_", StringComparison.Ordinal) ? 28f : 150f;
                }
                body.xDrive = drive;
                body.useGravity = false;
            }
        }

        static GameObject CreatePedestal(Transform parent, PhysicsMaterial material)
        {
            var pedestal = GameObject.CreatePrimitive(PrimitiveType.Cube);
            pedestal.name = "GraspPanel";
            pedestal.transform.SetParent(parent, false);
            pedestal.transform.SetLocalPositionAndRotation(
                Vector3.up * Dg5fGraspSpec.PanelThickness * 0.5f,
                Quaternion.identity);
            pedestal.transform.localScale = new Vector3(
                Dg5fGraspSpec.PanelWidth,
                Dg5fGraspSpec.PanelThickness,
                Dg5fGraspSpec.PanelDepth);

            var collider = pedestal.GetComponent<BoxCollider>();
            collider.material = material;
            return pedestal;
        }

        static Rigidbody CreateBall(Transform parent, PhysicsMaterial material)
        {
            var ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            ball.name = "GraspBall";
            ball.transform.SetParent(parent, false);
            ball.transform.localPosition = new Vector3(
                0.35f,
                Dg5fGraspSpec.PanelThickness + 0.02f,
                0.25f);
            ball.transform.localScale = Vector3.one * 0.04f;
            ball.GetComponent<Collider>().material = material;
            ball.GetComponent<Renderer>().sharedMaterial = GetOrCreateBallMaterial();

            var body = ball.AddComponent<Rigidbody>();
            body.mass = 0.05f;
            body.useGravity = true;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            body.interpolation = RigidbodyInterpolation.None;
            return body;
        }

        static Transform CreateGraspPoint(Transform palm)
        {
            Transform existing = palm.Find("GraspPoint");
            if (existing != null) UnityEngine.Object.DestroyImmediate(existing.gameObject);

            var grasp = new GameObject("GraspPoint").transform;
            grasp.SetParent(palm, false);
            grasp.localPosition = Dg5fGraspSpec.FullHandGraspPointLocalPosition;
            grasp.localRotation = Quaternion.identity;
            return grasp;
        }

        static Transform FindTransform(GameObject root, string name)
        {
            Transform found = root.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == name);
            if (found == null) throw new InvalidOperationException($"Missing transform: {name}");
            return found;
        }

        static PhysicsMaterial GetOrCreatePhysicsMaterial()
        {
            var material = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(PhysicsMaterialPath);
            if (material != null) return material;
            material = new PhysicsMaterial("GraspSurface")
            {
                dynamicFriction = 0.8f,
                staticFriction = 0.8f,
                bounciness = 0f,
                frictionCombine = PhysicsMaterialCombine.Maximum,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };
            AssetDatabase.CreateAsset(material, PhysicsMaterialPath);
            return material;
        }

        static Material GetOrCreateBallMaterial()
        {
            var material = AssetDatabase.LoadAssetAtPath<Material>(BallMaterialPath);
            if (material != null) return material;
            Shader shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            material = new Material(shader) { name = "GraspBall", color = Color.red };
            AssetDatabase.CreateAsset(material, BallMaterialPath);
            return material;
        }

        static void ConfigureCamera(Vector3 focus)
        {
            Camera camera = UnityEngine.Object.FindAnyObjectByType<Camera>();
            if (camera == null) return;
            camera.transform.position = focus + Vector3.back * 18f;
            camera.transform.LookAt(focus);
        }

        static void AddSceneToBuildSettings(string scenePath)
        {
            var scenes = EditorBuildSettings.scenes.ToList();
            if (scenes.All(item => item.path != scenePath))
                scenes.Add(new EditorBuildSettingsScene(scenePath, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }

        static void EnsureFolder(string path)
        {
            string[] parts = path.Split('/');
            string current = parts[0];
            for (int i = 1; i < parts.Length; i++)
            {
                string next = current + "/" + parts[i];
                if (!AssetDatabase.IsValidFolder(next)) AssetDatabase.CreateFolder(current, parts[i]);
                current = next;
            }
        }
    }
}
