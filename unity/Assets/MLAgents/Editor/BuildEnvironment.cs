using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace KDT.MLAgents.Editor
{
    public sealed class BuildEnvironment
    {
        readonly IReadOnlyDictionary<string, string> defaultValues;
        readonly IReadOnlyDictionary<string, string> localValues;

        BuildEnvironment(
            string repositoryRoot,
            IReadOnlyDictionary<string, string> defaultValues,
            IReadOnlyDictionary<string, string> localValues)
        {
            RepositoryRoot = repositoryRoot;
            this.defaultValues = defaultValues;
            this.localValues = localValues;
        }

        public string RepositoryRoot { get; }

        public static BuildEnvironment Load()
        {
            string projectRoot =
                Directory.GetParent(Application.dataPath)?.FullName
                ?? throw new InvalidOperationException(
                    "Cannot resolve Unity project root.");
            string repositoryRoot =
                Directory.GetParent(projectRoot)?.FullName
                ?? throw new InvalidOperationException(
                    "Cannot resolve repository root.");

            return new BuildEnvironment(
                repositoryRoot,
                ReadDotEnv(Path.Combine(repositoryRoot, ".env.example")),
                ReadDotEnv(Path.Combine(repositoryRoot, ".env")));
        }

        public string GetPath(params string[] keys)
        {
            string configuredPath = GetValue(keys);
            return Path.GetFullPath(
                Path.IsPathRooted(configuredPath)
                    ? configuredPath
                    : Path.Combine(RepositoryRoot, configuredPath));
        }

        public string GetFileName(params string[] keys)
        {
            string fileName = GetValue(keys);
            if (!string.Equals(
                    fileName,
                    Path.GetFileName(fileName),
                    StringComparison.Ordinal))
            {
                throw new InvalidOperationException(
                    $"{keys[0]} must be a file name, not a path: {fileName}");
            }

            return fileName;
        }

        string GetValue(params string[] keys)
        {
            foreach (string key in keys)
            {
                string value = Environment.GetEnvironmentVariable(key);
                if (!string.IsNullOrWhiteSpace(value))
                    return value.Trim();
            }

            foreach (string key in keys)
            {
                if (localValues.TryGetValue(key, out string value)
                    && !string.IsNullOrWhiteSpace(value))
                {
                    return value;
                }
            }

            foreach (string key in keys)
            {
                if (defaultValues.TryGetValue(key, out string value)
                    && !string.IsNullOrWhiteSpace(value))
                {
                    return value;
                }
            }

            throw new InvalidOperationException(
                $"Missing build setting: {string.Join(" or ", keys)}. "
                + "Restore .env.example or define the environment variable.");
        }

        static IReadOnlyDictionary<string, string> ReadDotEnv(string path)
        {
            var values = new Dictionary<string, string>(
                StringComparer.Ordinal);
            if (!File.Exists(path))
                return values;

            foreach (string sourceLine in File.ReadAllLines(path))
            {
                string line = sourceLine.Trim();
                if (line.Length == 0 || line.StartsWith("#"))
                    continue;
                if (line.StartsWith("export ", StringComparison.Ordinal))
                    line = line.Substring("export ".Length).TrimStart();

                int separator = line.IndexOf('=');
                if (separator <= 0)
                    continue;

                string key = line.Substring(0, separator).Trim();
                string value = line.Substring(separator + 1).Trim();
                if (value.Length >= 2
                    && ((value[0] == '"' && value[value.Length - 1] == '"')
                        || (value[0] == '\'' && value[value.Length - 1] == '\'')))
                {
                    value = value.Substring(1, value.Length - 2);
                }

                if (key.Length > 0)
                    values[key] = value;
            }

            return values;
        }
    }
}
