using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace KDT.MLAgents.Editor
{
    /// <summary>
    /// Post-processing shared by the Linux headless training builds. Kept OS
    /// agnostic so the players can be produced from Windows and macOS as well as
    /// Linux: the libdl probe shim is resolved from a vendored copy first and only
    /// falls back to the host's system libraries when it is available.
    /// </summary>
    public static class LinuxPlayerPostProcess
    {
        // Vendored fallback used on hosts (Windows/macOS) that do not ship a glibc
        // libdl.so.2. Relative to the repository root; overridable via .env.
        const string LibDlSourceKey = "DG5F_LINUX_LIBDL_SOURCE";

        static readonly string[] SystemLibDlCandidates =
        {
            "/lib/x86_64-linux-gnu/libdl.so.2",
            "/usr/lib/x86_64-linux-gnu/libdl.so.2"
        };

        public static void Apply(
            BuildEnvironment environment,
            string outputDirectory,
            string dataDirectoryName)
        {
            RemoveUnusedRuntimeAssimp(outputDirectory, dataDirectoryName);
            InstallLibDlProbeShim(
                environment, outputDirectory, dataDirectoryName);
            Directory.CreateDirectory(Path.Combine(
                outputDirectory,
                dataDirectoryName,
                "ML-Agents",
                "Timers"));
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

        static void InstallLibDlProbeShim(
            BuildEnvironment environment,
            string outputDirectory,
            string dataDirectoryName)
        {
            var candidates = new List<string>();
            if (environment.TryGetPath(out string vendored, LibDlSourceKey))
                candidates.Add(vendored);
            candidates.AddRange(SystemLibDlCandidates);

            string source = candidates.Find(
                path => !string.IsNullOrEmpty(path) && File.Exists(path));
            if (source == null)
                throw new InvalidOperationException(
                    "Linux libdl.so.2 is required for the URDF importer probe. "
                    + $"Provide it via {LibDlSourceKey} (a vendored copy) or "
                    + "install it at /lib/x86_64-linux-gnu/libdl.so.2.");

            string plugins = Path.Combine(
                outputDirectory,
                dataDirectoryName,
                "Plugins");
            Directory.CreateDirectory(plugins);
            File.Copy(source, Path.Combine(plugins, "libdl.so"), true);
        }
    }
}
