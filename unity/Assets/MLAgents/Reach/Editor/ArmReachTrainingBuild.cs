using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace KDT.ReachTraining.Editor
{
    public static class ArmReachTrainingBuild
    {
        const string BuildDirectoryName = "DG5FGraspReadyReach";
        const string PlayerName = "DG5FGraspReadyReach.x86_64";

        [MenuItem("Tools/ML-Agents/Build DG5F Grasp Ready Reach Linux Player")]
        public static void BuildLinux()
        {
            ArmReachTrainingSceneBuilder.Build();

            string projectRoot =
                Directory.GetParent(Application.dataPath)?.FullName
                ?? throw new InvalidOperationException(
                    "Cannot resolve Unity project root.");
            string repositoryRoot =
                Directory.GetParent(projectRoot)?.FullName
                ?? throw new InvalidOperationException(
                    "Cannot resolve repository root.");
            string directory = Path.Combine(
                repositoryRoot,
                "training",
                "builds",
                BuildDirectoryName);
            Directory.CreateDirectory(directory);
            string output = Path.Combine(directory, PlayerName);

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

            RemoveUnusedRuntimeAssimp(directory);
            InstallLinuxLibDlProbeShim(directory);
            Directory.CreateDirectory(Path.Combine(
                directory,
                "DG5FGraspReadyReach_Data",
                "ML-Agents",
                "Timers"));
            Debug.Log($"[ArmReachTrainingBuild] Built {output}");
        }

        static void RemoveUnusedRuntimeAssimp(string outputDirectory)
        {
            string plugin = Path.Combine(
                outputDirectory,
                "DG5FGraspReadyReach_Data",
                "Plugins",
                "libassimp.so");
            if (File.Exists(plugin)) File.Delete(plugin);
        }

        static void InstallLinuxLibDlProbeShim(string outputDirectory)
        {
            string[] candidates =
            {
                "/lib/x86_64-linux-gnu/libdl.so.2",
                "/usr/lib/x86_64-linux-gnu/libdl.so.2"
            };
            string source = Array.Find(candidates, File.Exists);
            if (source == null)
                throw new InvalidOperationException(
                    "Linux libdl.so.2 is required for the URDF importer probe.");
            string plugins = Path.Combine(
                outputDirectory,
                "DG5FGraspReadyReach_Data",
                "Plugins");
            Directory.CreateDirectory(plugins);
            File.Copy(source, Path.Combine(plugins, "libdl.so"), true);
        }
    }
}
