using System;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace KDT.GraspTraining.Editor
{
    /// <summary>
    /// GraspSurfaceContactSensor is required by Dg5fGraspAgent.ValidateConfiguration()
    /// but PipelineDemoSceneBuilder predates that requirement, so Pipeline_Demo.unity
    /// never got the sensors GraspTrainingSceneBuilder.ConfigureSafetySensors attaches
    /// to every training area. This adds them to the already-built demo scene in place
    /// (same logic: one sensor per moving, non-root, non-trigger collider, wired to the
    /// pedestal collider) without touching anything else already configured in the scene
    /// (model, camera receiver, driveHandJoints, etc.).
    /// </summary>
    public static class PipelineDemoSafetySensorPatch
    {
        [MenuItem("Tools/ML-Agents/Add Safety Sensors To Pipeline Demo Scene")]
        public static void Run()
        {
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

            if (agent.pedestalCollider == null)
                throw new InvalidOperationException(
                    "[PipelineDemoSafetySensorPatch] agent.pedestalCollider is not set — "
                    + "cannot wire GraspSurfaceContactSensor.unsafeSurface.");

            var sensors = new List<GraspSurfaceContactSensor>();
            foreach (Collider collider in
                     agent.robotBase.GetComponentsInChildren<Collider>(true))
            {
                if (collider == null || !collider.enabled || collider.isTrigger)
                    continue;
                ArticulationBody body = collider.GetComponentInParent<ArticulationBody>();
                if (body == null || body.isRoot) continue;

                GraspSurfaceContactSensor sensor =
                    collider.GetComponent<GraspSurfaceContactSensor>();
                if (sensor == null)
                    sensor = Undo.AddComponent<GraspSurfaceContactSensor>(collider.gameObject);
                sensor.agent = agent;
                sensor.unsafeSurface = agent.pedestalCollider;
                sensors.Add(sensor);
            }

            if (sensors.Count == 0)
                throw new InvalidOperationException(
                    "No moving robot colliders were available for panel safety.");

            Undo.RecordObject(agent, "Wire safety sensors");
            agent.safetySensors = sensors.ToArray();
            EditorUtility.SetDirty(agent);

            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            if (openedHere) EditorSceneManager.CloseScene(scene, true);

            Debug.Log(
                $"[PipelineDemoSafetySensorPatch] Wired {sensors.Count} "
                + $"GraspSurfaceContactSensor(s) in {PipelineDemoSceneBuilder.DemoScenePath}.");
        }
    }
}
