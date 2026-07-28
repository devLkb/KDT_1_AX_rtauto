using System;
using System.Collections.Generic;
using System.Linq;
using KDT.GraspLiftTraining;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace KDT.GraspLiftTraining.Editor
{
    /// <summary>
    /// Regenerates the DG5F grasp + lift training scene from the robot prefab.
    /// The scene is a build artefact: never hand-edit it, re-run this menu item.
    /// </summary>
    public static class GraspLiftTrainingSceneBuilder
    {
        const string SourceRobotPath = "Assets/Robots/Prefabs/ur5e_dg5f_left.prefab";
        const string TrainingRoot = "Assets/MLAgents/GraspLift";
        const string TrainingPrefabPath = TrainingRoot + "/GraspLiftTrainingArea.prefab";
        const string TrainingScenePath = TrainingRoot + "/DG5F_GraspLiftTraining.unity";
        const string BlockMaterialPath = TrainingRoot + "/GraspLiftBlock.mat";
        const string PanelPhysicsMaterialPath = TrainingRoot + "/GraspLiftPanel.physicMaterial";
        const string BlockPhysicsMaterialPath = TrainingRoot + "/GraspLiftBlock.physicMaterial";
        const int TrainingAreaCount = 20;
        const int TrainingAreaColumns = 4;
        const float TrainingAreaSpacing = 3f;

        static readonly HashSet<string> CompetingDriverTypes = new HashSet<string>
        {
            "Dg5fReceiver",
            "Dg5fHandDriver",
            "Dg5fFingerIK",
            "Dg5fThumbIK",
            "Dg5fJointLogger",
            "HandSliderUI",
            "ArmTargetIK",
            "RobotInitialPoseSync",
            "Dg5fGraspAgent",
            "GraspTeleoperationHandoff"
        };

        [MenuItem("Tools/ML-Agents/Build DG5F Grasp Lift Training Scene")]
        public static void Build()
        {
            EnsureFolder(TrainingRoot);
            var sourceRobot = AssetDatabase.LoadAssetAtPath<GameObject>(SourceRobotPath);
            if (sourceRobot == null)
                throw new InvalidOperationException($"Missing robot prefab: {SourceRobotPath}");

            Scene scene = EditorSceneManager.NewScene(
                NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            scene.name = "DG5F_GraspLiftTraining";

            var area = new GameObject("DG5F_GraspLiftTrainingArea");
            var robot = (GameObject)PrefabUtility.InstantiatePrefab(sourceRobot, area.transform);
            robot.name = "UR5e_DG5F_GraspLiftAgent";
            robot.transform.SetLocalPositionAndRotation(
                Vector3.up * Dg5fGraspLiftSpec.PanelThickness,
                Quaternion.identity);
            DisableCompetingDrivers(robot);
            ConfigureJointDrives(robot);

            GameObject pedestal = CreatePanel(area.transform, GetOrCreatePanelPhysicsMaterial());
            Rigidbody block = CreateBlock(area.transform, GetOrCreateBlockPhysicsMaterial());

            Transform palm = FindTransform(robot, "ll_dg_palm");
            var tips = new Transform[Dg5fGraspLiftSpec.FingerCount];
            for (int finger = 0; finger < tips.Length; finger++)
                tips[finger] = FindTransform(robot, $"ll_dg_{finger + 1}_tip");
            Transform graspPoint = CreateGraspPoint(palm);

            var agent = robot.GetComponent<Dg5fGraspLiftAgent>();
            if (agent == null) agent = robot.AddComponent<Dg5fGraspLiftAgent>();
            agent.graspObject = block;
            agent.pedestal = pedestal.transform;
            agent.pedestalCollider = pedestal.GetComponent<Collider>();
            agent.robotBase = robot.transform;
            agent.palm = palm;
            agent.graspPoint = graspPoint;
            agent.fingerTips = tips;
            agent.contactSensors = ConfigureObjectContactSensors(palm, tips, block);
            Collider panelCollider = pedestal.GetComponent<Collider>();
            agent.safetySensors = ConfigureSafetySensors(
                robot, panelCollider, agent);
            agent.handSurfaceSensors = ConfigureHandSurfaceSensors(robot, panelCollider);
            agent.MaxStep = 0;

            var behavior = robot.GetComponent<BehaviorParameters>();
            if (behavior == null) behavior = robot.AddComponent<BehaviorParameters>();
            behavior.BehaviorName = Dg5fGraspLiftSpec.BehaviorName;
            behavior.BehaviorType = BehaviorType.Default;
            // Sampling during inference is visible as arm tremble; regenerated
            // scenes must retain deterministic action selection.
            behavior.DeterministicInference = true;
            behavior.BrainParameters.VectorObservationSize = Dg5fGraspLiftSpec.ObservationSize;
            behavior.BrainParameters.NumStackedVectorObservations = 1;
            behavior.BrainParameters.ActionSpec =
                ActionSpec.MakeContinuous(Dg5fGraspLiftSpec.ActionSize);
            behavior.BrainParameters.VectorActionDescriptions = new[]
            {
                "shoulder_pan_delta", "shoulder_lift_delta", "elbow_delta",
                "wrist_1_delta", "wrist_2_delta", "wrist_3_delta",
                "hand_closure_delta"
            };

            var requester = robot.GetComponent<DecisionRequester>();
            if (requester == null) requester = robot.AddComponent<DecisionRequester>();
            requester.DecisionPeriod = 5;
            requester.TakeActionsBetweenDecisions = false;

            PrefabUtility.SaveAsPrefabAssetAndConnect(
                area, TrainingPrefabPath, InteractionMode.AutomatedAction);
            PopulateTrainingAreas(area);
            ConfigureCamera(LayoutCenter());
            Selection.activeGameObject = area;

            EditorSceneManager.SaveScene(scene, TrainingScenePath);
            AddSceneToBuildSettings(TrainingScenePath);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Debug.Log(
                $"[GraspLiftTrainingSceneBuilder] Built {TrainingPrefabPath} and {TrainingScenePath}");
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
                var instance = (GameObject)PrefabUtility.InstantiatePrefab(trainingPrefab);
                ConfigureTrainingAreaInstance(instance, index);
            }
        }

        static void ConfigureTrainingAreaInstance(GameObject area, int index)
        {
            int row = index / TrainingAreaColumns;
            int column = index % TrainingAreaColumns;
            area.name = $"DG5F_GraspLiftTrainingArea_{index:00}";
            area.transform.SetPositionAndRotation(
                new Vector3(column * TrainingAreaSpacing, row * TrainingAreaSpacing, 0f),
                Quaternion.identity);

            var agent = area.GetComponentInChildren<Dg5fGraspLiftAgent>(true);
            if (agent == null)
                throw new InvalidOperationException(
                    $"Training area {index} has no {nameof(Dg5fGraspLiftAgent)}.");
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
                    // Grasping needs real grip force. The reach task ran the fingers at
                    // 7.5 N because they only had to hold an open pose; holding a 0.12 kg
                    // block through friction needs a much stiffer, stronger drive.
                    drive.stiffness = 1500f;
                    drive.damping = 120f;
                    drive.forceLimit = 20f;
                }
                else
                {
                    drive.forceLimit =
                        body.name.StartsWith("wrist_", StringComparison.Ordinal) ? 28f : 150f;
                }
                body.xDrive = drive;
                body.useGravity = false;
            }
        }

        /// Instruments the palm and the five fingertips. The URDF importer parents
        /// every collider under a "Collisions" child, and Unity delivers
        /// OnCollision* to the collider's GameObject, so a sensor on the link alone
        /// would never fire. Both are instrumented and share a contact index.
        static GraspLiftObjectContactSensor[] ConfigureObjectContactSensors(
            Transform palm,
            Transform[] tips,
            Rigidbody block)
        {
            var sensors = new List<GraspLiftObjectContactSensor>();
            for (int finger = 0; finger < tips.Length; finger++)
                AddContactSensors(tips[finger], finger, block, sensors);
            AddContactSensors(palm, Dg5fGraspLiftSpec.PalmContactIndex, block, sensors);

            for (int index = 0; index < Dg5fGraspLiftSpec.ContactPointCount; index++)
            {
                if (!sensors.Any(sensor => sensor.contactIndex == index))
                    throw new InvalidOperationException(
                        $"Contact point {index} has no collider to instrument.");
            }
            return sensors.ToArray();
        }

        static void AddContactSensors(
            Transform link,
            int contactIndex,
            Rigidbody block,
            List<GraspLiftObjectContactSensor> sensors)
        {
            if (link == null)
                throw new InvalidOperationException(
                    $"Contact point {contactIndex} has no link transform.");

            var targets = new List<GameObject> { link.gameObject };
            Transform collisions = link.Find("Collisions");
            if (collisions != null)
            {
                foreach (Collider collider in collisions.GetComponentsInChildren<Collider>(true))
                {
                    if (collider == null || collider.isTrigger) continue;
                    targets.Add(collider.gameObject);
                }
            }
            if (targets.Count < 2)
                throw new InvalidOperationException(
                    $"Contact point {contactIndex} ({link.name}) has no collision geometry.");

            foreach (GameObject target in targets)
            {
                var sensor = target.GetComponent<GraspLiftObjectContactSensor>();
                if (sensor == null)
                    sensor = target.AddComponent<GraspLiftObjectContactSensor>();
                sensor.contactIndex = contactIndex;
                sensor.targetObject = block;
                sensors.Add(sensor);
            }
        }

        /// Panel safety covers the ARM links only. The hand has to operate right at
        /// the table surface to grasp the block, so instrumenting finger colliders
        /// (as the reach task does) would abort every grasp attempt.
        static GraspLiftSurfaceContactSensor[] ConfigureSafetySensors(
            GameObject robot,
            Collider panel,
            Dg5fGraspLiftAgent agent)
        {
            var sensors = new List<GraspLiftSurfaceContactSensor>();
            foreach (Collider collider in robot.GetComponentsInChildren<Collider>(true))
            {
                if (collider == null || !collider.enabled || collider.isTrigger) continue;
                ArticulationBody body = collider.GetComponentInParent<ArticulationBody>();
                if (body == null || body.isRoot) continue;
                // Skip everything from the hand mount downward.
                if (IsHandCollider(collider.transform)) continue;

                var sensor = collider.GetComponent<GraspLiftSurfaceContactSensor>();
                if (sensor == null)
                    sensor = collider.gameObject.AddComponent<GraspLiftSurfaceContactSensor>();
                sensor.agent = agent;
                sensor.unsafeSurface = panel;
                sensors.Add(sensor);
            }
            if (sensors.Count == 0)
                throw new InvalidOperationException(
                    "No moving arm colliders were available for panel safety.");
            return sensors.ToArray();
        }

        static bool IsHandCollider(Transform colliderTransform)
        {
            for (Transform current = colliderTransform; current != null; current = current.parent)
            {
                if (current.name.Contains("_dg_") || current.name == "ll_dg_mount")
                    return true;
            }
            return false;
        }

        /// Instruments exactly the hand colliders excluded from terminal panel
        /// safety. Link and collider GameObjects are both included because imported
        /// URDF collision callbacks are delivered to the collider child.
        static GraspLiftHandSurfaceSensor[] ConfigureHandSurfaceSensors(
            GameObject robot,
            Collider panel)
        {
            var targets = new HashSet<GameObject>();
            foreach (Collider collider in robot.GetComponentsInChildren<Collider>(true))
            {
                if (collider == null || !collider.enabled || collider.isTrigger) continue;
                ArticulationBody body = collider.GetComponentInParent<ArticulationBody>();
                if (body == null || body.isRoot || !IsHandCollider(collider.transform)) continue;

                targets.Add(body.gameObject);
                targets.Add(collider.gameObject);
            }

            if (targets.Count == 0)
                throw new InvalidOperationException(
                    "No hand colliders were available for panel contact reporting.");

            var sensors = new List<GraspLiftHandSurfaceSensor>(targets.Count);
            foreach (GameObject target in targets)
            {
                var sensor = target.GetComponent<GraspLiftHandSurfaceSensor>();
                if (sensor == null)
                    sensor = target.AddComponent<GraspLiftHandSurfaceSensor>();
                sensor.surface = panel;
                sensors.Add(sensor);
            }
            return sensors.ToArray();
        }

        static GameObject CreatePanel(Transform parent, PhysicsMaterial material)
        {
            var panel = GameObject.CreatePrimitive(PrimitiveType.Cube);
            panel.name = "GraspPanel";
            panel.transform.SetParent(parent, false);
            panel.transform.SetLocalPositionAndRotation(
                Vector3.up * Dg5fGraspLiftSpec.PanelThickness * 0.5f,
                Quaternion.identity);
            panel.transform.localScale = new Vector3(
                Dg5fGraspLiftSpec.PanelWidth,
                Dg5fGraspLiftSpec.PanelThickness,
                Dg5fGraspLiftSpec.PanelDepth);
            panel.GetComponent<BoxCollider>().material = material;
            return panel;
        }

        static Rigidbody CreateBlock(Transform parent, PhysicsMaterial material)
        {
            var block = GameObject.CreatePrimitive(PrimitiveType.Cube);
            block.name = "GraspBlock";
            block.transform.SetParent(parent, false);
            block.transform.localPosition = new Vector3(
                0.40f,
                Dg5fGraspLiftSpec.PanelThickness + Dg5fGraspLiftSpec.BlockHalfHeight,
                0.25f);
            // Default size only; Dg5fGraspLiftAgent re-applies the `block_width`
            // lesson (scale and mass) at every episode reset.
            block.transform.localScale = new Vector3(
                Dg5fGraspLiftSpec.BlockWidth,
                Dg5fGraspLiftSpec.BlockHeight,
                Dg5fGraspLiftSpec.BlockWidth);
            block.GetComponent<Collider>().material = material;
            block.GetComponent<Renderer>().sharedMaterial = GetOrCreateBlockMaterial();

            var body = block.AddComponent<Rigidbody>();
            body.mass = Dg5fGraspLiftSpec.CurrentBlockMass;
            body.centerOfMass = Dg5fGraspLiftSpec.CurrentBlockCenterOfMassLocal;
            body.useGravity = true;
            // The fingers close fast relative to the physics step; discrete detection
            // lets them tunnel through a 5 cm block.
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
            grasp.localPosition = Dg5fGraspLiftSpec.FullHandGraspPointLocalPosition;
            grasp.localRotation = Quaternion.identity;
            return grasp;
        }

        static Transform FindTransform(GameObject root, string name)
        {
            Transform found = root.GetComponentsInChildren<Transform>(true)
                .FirstOrDefault(item => item.name == name);
            if (found == null) throw new InvalidOperationException($"Missing transform: {name}");
            return found;
        }

        static PhysicsMaterial GetOrCreatePanelPhysicsMaterial()
        {
            var material = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(PanelPhysicsMaterialPath);
            if (material != null) return material;
            material = new PhysicsMaterial("GraspLiftPanel")
            {
                dynamicFriction = 0.8f,
                staticFriction = 0.8f,
                bounciness = 0f,
                frictionCombine = PhysicsMaterialCombine.Average,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };
            AssetDatabase.CreateAsset(material, PanelPhysicsMaterialPath);
            return material;
        }

        static PhysicsMaterial GetOrCreateBlockPhysicsMaterial()
        {
            var material = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(BlockPhysicsMaterialPath);
            if (material != null) return material;
            // High friction on the block only (Maximum combine) so a good grasp holds
            // through friction, mirroring the reference environment's static_friction 2.0
            // object material. The panel stays at 0.8 so the block still slides if it is
            // shoved instead of grasped.
            material = new PhysicsMaterial("GraspLiftBlock")
            {
                dynamicFriction = 1.2f,
                staticFriction = 1.5f,
                bounciness = 0f,
                frictionCombine = PhysicsMaterialCombine.Maximum,
                bounceCombine = PhysicsMaterialCombine.Minimum
            };
            AssetDatabase.CreateAsset(material, BlockPhysicsMaterialPath);
            return material;
        }

        static Material GetOrCreateBlockMaterial()
        {
            var material = AssetDatabase.LoadAssetAtPath<Material>(BlockMaterialPath);
            if (material != null) return material;
            Shader shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            material = new Material(shader) { name = "GraspLiftBlock", color = Color.red };
            AssetDatabase.CreateAsset(material, BlockMaterialPath);
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
