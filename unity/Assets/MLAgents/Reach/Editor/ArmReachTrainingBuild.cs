using System;
using System.IO;
using KDT.MLAgents.Editor;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace KDT.ReachTraining.Editor
{
    public static class ArmReachTrainingBuild
    {
        [MenuItem("Tools/ML-Agents/Build DG5F Grasp Ready Reach Linux Player")]
        public static void BuildLinux()
        {
            ArmReachTrainingSceneBuilder.Build();

            BuildEnvironment environment = BuildEnvironment.Load();
            string directory = environment.GetPath(
                "DG5F_REACH_BUILD_OUTPUT");
            string playerName = environment.GetFileName(
                "DG5F_REACH_PLAYER_NAME");
            string dataDirectoryName =
                Path.GetFileNameWithoutExtension(playerName) + "_Data";
            Directory.CreateDirectory(directory);
            string output = Path.Combine(directory, playerName);

            var options = new BuildPlayerOptions
            {
                scenes = new[]
                {
                    ArmReachTrainingSceneBuilder.TrainingScenePath
                },
                locationPathName = output,
                target = BuildTarget.StandaloneLinux64,
                subtarget = (int)StandaloneBuildSubtarget.Player,
                options = BuildOptions.None
            };
            BuildReport report = BuildPipeline.BuildPlayer(options);
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException(
                    "DG5F grasp-ready reach Linux build failed: "
                    + report.summary.result);

            LinuxPlayerPostProcess.Apply(
                environment, directory, dataDirectoryName);
            Debug.Log($"[ArmReachTrainingBuild] Built {output}");
        }
    }
}
