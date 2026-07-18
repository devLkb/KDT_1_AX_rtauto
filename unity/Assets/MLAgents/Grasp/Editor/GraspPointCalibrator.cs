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
    /// Calibrates the fixed palm-local GraspPoint from an isolated 35% pre-grasp
    /// prefab preview. The training scene is never opened or saved.
    /// </summary>
    public static class GraspPointCalibrator
    {
        public const string TrainingPrefabPath =
            "Assets/MLAgents/Grasp/TrainingArea.prefab";

        [MenuItem("Tools/ML-Agents/Calibrate DG5F Stable GraspPoint")]
        public static void CalibrateTrainingPrefab()
        {
            GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(TrainingPrefabPath);
            if (prefab == null)
                throw new InvalidOperationException($"Missing training prefab: {TrainingPrefabPath}");

            Vector3 localPosition = CalculatePreGraspLocalPosition(prefab);
            if (Vector3.Distance(
                    localPosition,
                    Dg5fGraspSpec.CalibratedGraspPointLocalPosition) > 0.001f)
            {
                throw new InvalidOperationException(
                    "Calculated GraspPoint differs from the reviewed calibration by more than 1 mm. "
                    + "Review the robot geometry before updating the 3.0 contract.");
            }
            GameObject contents = PrefabUtility.LoadPrefabContents(TrainingPrefabPath);
            try
            {
                Dg5fGraspAgent agent = contents.GetComponentInChildren<Dg5fGraspAgent>(true);
                if (agent == null) throw new InvalidOperationException("Training prefab has no grasp agent.");
                Transform palm = FindTransform(agent.gameObject, "ll_dg_palm");
                Transform graspPoint = palm.Find("GraspPoint");
                if (graspPoint == null)
                    throw new InvalidOperationException("Training prefab has no palm/GraspPoint.");

                graspPoint.localPosition = localPosition;
                graspPoint.localRotation = Quaternion.identity;
                graspPoint.localScale = Vector3.one;

                foreach (BehaviorParameters stray in contents.GetComponents<BehaviorParameters>())
                    UnityEngine.Object.DestroyImmediate(stray);

                BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
                if (behavior == null)
                    throw new InvalidOperationException("The grasp Agent requires BehaviorParameters.");
                behavior.BehaviorName = Dg5fGraspSpec.BehaviorName;
                behavior.BrainParameters.VectorObservationSize = Dg5fGraspSpec.ObservationSize;
                behavior.BrainParameters.ActionSpec =
                    Unity.MLAgents.Actuators.ActionSpec.MakeContinuous(Dg5fGraspSpec.ActionSize);

                PrefabUtility.SaveAsPrefabAsset(contents, TrainingPrefabPath);
            }
            finally
            {
                PrefabUtility.UnloadPrefabContents(contents);
            }

            AssetDatabase.SaveAssets();
            Debug.Log(
                $"[GraspPointCalibrator] GraspPoint.localPosition={localPosition:R}; "
                + "five fingertips equally weighted at PreGrasp35Deg.");
        }

        public static Vector3 CalculatePreGraspLocalPosition(GameObject prefab)
        {
            if (prefab == null) throw new ArgumentNullException(nameof(prefab));

            Scene preview = EditorSceneManager.NewPreviewScene();
            try
            {
                var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab, preview);
                if (instance == null)
                    throw new InvalidOperationException("Failed to instantiate training prefab preview.");

                Dg5fGraspAgent agent = instance.GetComponentInChildren<Dg5fGraspAgent>(true);
                if (agent == null) throw new InvalidOperationException("Training prefab has no grasp agent.");
                agent.enabled = false;
                var requester = agent.GetComponent<Unity.MLAgents.DecisionRequester>();
                if (requester != null) requester.enabled = false;

                ArticulationBody[] bodies =
                    agent.GetComponentsInChildren<ArticulationBody>(true);
                for (int finger = 1; finger <= Dg5fGraspSpec.FingerCount; finger++)
                {
                    for (int joint = 1; joint <= 4; joint++)
                    {
                        int channel = (finger - 1) * 4 + joint - 1;
                        ArticulationBody body = bodies.FirstOrDefault(
                            candidate => candidate.name.EndsWith(
                                $"_dg_{finger}_{joint}",
                                StringComparison.Ordinal));
                        if (body == null)
                            throw new InvalidOperationException(
                                $"Missing preview hand joint {finger}_{joint}.");
                        SetJointPosition(body, Dg5fGraspSpec.PreGrasp35Deg[channel]);
                    }
                }

                Physics.SyncTransforms();

                Transform palm = FindTransform(agent.gameObject, "ll_dg_palm");
                var tips = new Transform[Dg5fGraspSpec.FingerCount];
                for (int finger = 0; finger < tips.Length; finger++)
                    tips[finger] = FindTransform(agent.gameObject, $"ll_dg_{finger + 1}_tip");
                return Dg5fGraspSpec.PalmLocalGraspPoint(palm, tips);
            }
            finally
            {
                EditorSceneManager.ClosePreviewScene(preview);
            }
        }

        static void SetJointPosition(ArticulationBody body, float targetDeg)
        {
            ArticulationDrive drive = body.xDrive;
            float clamped = Dg5fGraspSpec.ClampJointTarget(
                targetDeg,
                drive.lowerLimit,
                drive.upperLimit);
            drive.target = clamped;
            body.xDrive = drive;
            body.jointPosition = new ArticulationReducedSpace(clamped * Mathf.Deg2Rad);
            body.jointVelocity = new ArticulationReducedSpace(0f);
        }

        static Transform FindTransform(GameObject root, string name)
        {
            Transform found = root.GetComponentsInChildren<Transform>(true)
                .FirstOrDefault(candidate => candidate.name == name);
            if (found == null) throw new InvalidOperationException($"Missing transform: {name}");
            return found;
        }
    }
}
