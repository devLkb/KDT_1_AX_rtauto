using System;
using System.IO;
using System.Security.Cryptography;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEngine;

namespace KDT.GraspTraining.Editor
{
    public static class StableModelPromotion
    {
        const string PrefabPath = "Assets/MLAgents/Grasp/TrainingArea.prefab";
        const string ModelPath = "Assets/MLAgents/Grasp/Models/DG5FStableGrasp.onnx";
        const string ApprovalPath =
            "Assets/MLAgents/Grasp/Models/DG5FStableGrasp.approved.json";

        [Serializable]
        sealed class Approval
        {
            public string specVersion;
            public string behaviorName;
            public int observations;
            public int actions;
            public int episodes;
            public float successRate;
            public float reachRate;
            public string sha256;
        }

        [MenuItem("Tools/ML-Agents/Assign Approved DG5F Stable Model")]
        public static void AssignApprovedModel()
        {
            if (!File.Exists(ApprovalPath) || !File.Exists(ModelPath))
                throw new InvalidOperationException(
                    "Approved model and manifest must be staged before assignment.");

            Approval approval = JsonUtility.FromJson<Approval>(
                File.ReadAllText(ApprovalPath));
            if (approval == null
                || approval.specVersion != Dg5fGraspSpec.SpecVersion
                || approval.behaviorName != Dg5fGraspSpec.BehaviorName
                || approval.observations != Dg5fGraspSpec.ObservationSize
                || approval.actions != Dg5fGraspSpec.ActionSize
                || approval.episodes != 200
                || approval.successRate < 0.8f
                || approval.reachRate < 0.8f)
            {
                throw new InvalidOperationException(
                    "Stable model approval manifest does not meet the 3.0 acceptance gate.");
            }
            if (!string.Equals(Sha256(ModelPath), approval.sha256, StringComparison.OrdinalIgnoreCase))
                throw new InvalidOperationException("Stable model SHA-256 differs from approval manifest.");

            AssetDatabase.ImportAsset(ModelPath, ImportAssetOptions.ForceSynchronousImport);
            UnityEngine.Object model = AssetDatabase.LoadMainAssetAtPath(ModelPath);
            if (model == null) throw new InvalidOperationException("Unity could not import the approved ONNX.");

            GameObject contents = PrefabUtility.LoadPrefabContents(PrefabPath);
            try
            {
                Dg5fGraspAgent agent = contents.GetComponentInChildren<Dg5fGraspAgent>(true);
                BehaviorParameters behavior =
                    agent == null ? null : agent.GetComponent<BehaviorParameters>();
                if (behavior == null)
                    throw new InvalidOperationException("Training prefab has no Agent BehaviorParameters.");
                var serialized = new SerializedObject(behavior);
                SerializedProperty modelProperty = serialized.FindProperty("m_Model");
                if (modelProperty == null)
                    throw new InvalidOperationException("BehaviorParameters m_Model was not found.");
                modelProperty.objectReferenceValue = model;
                serialized.ApplyModifiedPropertiesWithoutUndo();
                PrefabUtility.SaveAsPrefabAsset(contents, PrefabPath);
            }
            finally
            {
                PrefabUtility.UnloadPrefabContents(contents);
            }
            AssetDatabase.SaveAssets();
            Debug.Log($"[StableModelPromotion] Assigned approved model {approval.sha256}.");
        }

        static string Sha256(string path)
        {
            using (SHA256 algorithm = SHA256.Create())
            using (FileStream stream = File.OpenRead(path))
                return BitConverter.ToString(algorithm.ComputeHash(stream))
                    .Replace("-", string.Empty)
                    .ToLowerInvariant();
        }
    }
}
