using System;
using System.IO;
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

            string projectRoot = Directory.GetParent(Application.dataPath)?.FullName
                ?? throw new InvalidOperationException("Cannot resolve Unity project root.");
            string repositoryRoot = Directory.GetParent(projectRoot)?.FullName
                ?? throw new InvalidOperationException("Cannot resolve repository root.");
            string outputDirectory = Environment.GetEnvironmentVariable("DG5F_BUILD_OUTPUT");
            if (string.IsNullOrWhiteSpace(outputDirectory))
                outputDirectory = Path.Combine(repositoryRoot, "training", "builds", "DG5FGrasp");
            else
                outputDirectory = Path.GetFullPath(outputDirectory);
            Directory.CreateDirectory(outputDirectory);

            var options = new BuildPlayerOptions
            {
                scenes = new[] { TrainingScene },
                locationPathName = Path.Combine(outputDirectory, "DG5FGrasp.x86_64"),
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

            RemoveUnusedRuntimeAssimp(outputDirectory);
            InstallLinuxLibDlProbeShim(outputDirectory);
            Directory.CreateDirectory(Path.Combine(
                outputDirectory,
                "DG5FGrasp_Data",
                "ML-Agents",
                "Timers"));

            Debug.Log($"[GraspTrainingBuild] Built {options.locationPathName}");
        }

        static void RemoveUnusedRuntimeAssimp(string outputDirectory)
        {
            string plugin = Path.Combine(
                outputDirectory,
                "DG5FGrasp_Data",
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
                "DG5FGrasp_Data",
                "Plugins");
            Directory.CreateDirectory(plugins);
            File.Copy(source, Path.Combine(plugins, "libdl.so"), true);
        }
    }
}
