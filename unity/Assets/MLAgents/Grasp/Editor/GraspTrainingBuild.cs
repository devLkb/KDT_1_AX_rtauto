using System;
using System.IO;
using KDT.MLAgents.Editor;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace KDT.GraspTraining.Editor
{
    public static class GraspTrainingBuild
    {
        const string TrainingScene = "Assets/MLAgents/Grasp/DG5F_GraspTraining.unity";

        [MenuItem("Tools/ML-Agents/Build Linux Headless Training Environment")]
        public static void BuildLinuxHeadless()
        {
            GraspTrainingSceneBuilder.Build();

            BuildEnvironment environment = BuildEnvironment.Load();
            string outputDirectory = environment.GetPath(
                "DG5F_GRASP_BUILD_OUTPUT",
                "DG5F_BUILD_OUTPUT");
            string playerName = environment.GetFileName(
                "DG5F_GRASP_PLAYER_NAME");
            string dataDirectoryName =
                Path.GetFileNameWithoutExtension(playerName) + "_Data";
            Directory.CreateDirectory(outputDirectory);

            var options = new BuildPlayerOptions
            {
                scenes = new[] { TrainingScene },
                locationPathName = Path.Combine(outputDirectory, playerName),
                target = BuildTarget.StandaloneLinux64,
                // Build a regular Linux player so the optional Dedicated Server module
                // is not required. The launcher uses Xvfb on display-less hosts.
                subtarget = (int)StandaloneBuildSubtarget.Player,
                options = BuildOptions.None
            };

            BuildReport report = BuildPipeline.BuildPlayer(options);
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException(
                    $"Headless build failed: {report.summary.result}, errors={report.summary.totalErrors}");

            LinuxPlayerPostProcess.Apply(
                environment, outputDirectory, dataDirectoryName);

            Debug.Log($"[GraspTrainingBuild] Built {options.locationPathName}");
        }
    }
}
