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
            string projectRoot = Directory.GetParent(Application.dataPath)?.FullName
                ?? throw new InvalidOperationException("Cannot resolve Unity project root.");
            string repositoryRoot = Directory.GetParent(projectRoot)?.FullName
                ?? throw new InvalidOperationException("Cannot resolve repository root.");
            string outputDirectory = Path.Combine(repositoryRoot, "training", "builds", "DG5FGrasp");
            Directory.CreateDirectory(outputDirectory);

            var options = new BuildPlayerOptions
            {
                scenes = new[] { TrainingScene },
                locationPathName = Path.Combine(outputDirectory, "DG5FGrasp.x86_64"),
                target = BuildTarget.StandaloneLinux64,
                // Build a regular Linux player so the optional Dedicated Server module
                // is not required. ML-Agents launches it headlessly with -nographics.
                subtarget = (int)StandaloneBuildSubtarget.Player,
                options = BuildOptions.None
            };

            BuildReport report = BuildPipeline.BuildPlayer(options);
            if (report.summary.result != BuildResult.Succeeded)
                throw new InvalidOperationException(
                    $"Headless build failed: {report.summary.result}, errors={report.summary.totalErrors}");

            Debug.Log($"[GraspTrainingBuild] Built {options.locationPathName}");
        }
    }
}
