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

            RemoveUnusedRuntimeAssimp(directory, dataDirectoryName);
            InstallLinuxLibDlProbeShim(directory, dataDirectoryName);
            Directory.CreateDirectory(Path.Combine(
                directory,
                dataDirectoryName,
                "ML-Agents",
                "Timers"));
            Debug.Log($"[ArmReachTrainingBuild] Built {output}");
        }

        static void RemoveUnusedRuntimeAssimp(
            string outputDirectory,
            string dataDirectoryName)
        {
            string plugin = Path.Combine(
                outputDirectory,
                dataDirectoryName,
                "Plugins",
                "libassimp.so");
            if (File.Exists(plugin)) File.Delete(plugin);
        }

        static void InstallLinuxLibDlProbeShim(
            string outputDirectory,
            string dataDirectoryName)
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
                dataDirectoryName,
                "Plugins");
            Directory.CreateDirectory(plugins);
            File.Copy(source, Path.Combine(plugins, "libdl.so"), true);
        }
    }
}
