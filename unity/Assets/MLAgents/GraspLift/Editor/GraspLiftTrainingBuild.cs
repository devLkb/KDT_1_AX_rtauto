using System;
using System.IO;
using KDT.MLAgents.Editor;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace KDT.GraspLiftTraining.Editor
{
    public static class GraspLiftTrainingBuild
    {
        const string TrainingScene = "Assets/MLAgents/GraspLift/DG5F_GraspLiftTraining.unity";

        [MenuItem("Tools/ML-Agents/Build DG5F Grasp Lift Linux Player")]
        public static void BuildLinuxPlayer()
        {
            GraspLiftTrainingSceneBuilder.Build();

            BuildEnvironment environment = BuildEnvironment.Load();
            string outputDirectory = environment.GetPath("DG5F_GRASPLIFT_BUILD_OUTPUT");
            string playerName = environment.GetFileName("DG5F_GRASPLIFT_PLAYER_NAME");
            string dataDirectoryName =
                Path.GetFileNameWithoutExtension(playerName) + "_Data";
            Directory.CreateDirectory(outputDirectory);

            var options = new BuildPlayerOptions
            {
                scenes = new[] { TrainingScene },
                locationPathName = Path.Combine(outputDirectory, playerName),
                target = BuildTarget.StandaloneLinux64,
                // A regular Linux player, so the optional Dedicated Server module is
                // not required. The launcher uses Xvfb on display-less hosts.
                subtarget = (int)StandaloneBuildSubtarget.Player,
                options = BuildOptions.None
            };

            BuildReport report = BuildPipeline.BuildPlayer(options);
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException(
                    $"Grasp lift build failed: {report.summary.result}, "
                    + $"errors={report.summary.totalErrors}");

            LinuxPlayerPostProcess.Apply(environment, outputDirectory, dataDirectoryName);

            Debug.Log($"[GraspLiftTrainingBuild] Built {options.locationPathName}");
        }
    }
}
