using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace KDT.ReachTraining
{
    /// <summary>
    /// Assigns a deterministic, non-overlapping sequence of evaluation seeds across
    /// the 20 parallel reach areas and writes the evaluator's stable CSV contract.
    /// </summary>
    public static class ArmReachEvaluationSession
    {
        public const int AreaCount = 20;
        public const string EpisodesArgument = "--dg5f-eval-episodes";
        public const string BaseSeedArgument = "--dg5f-eval-base-seed";
        public const string CsvArgument = "--dg5f-eval-csv";
        public const string CsvHeader =
            "episode,seed,success,final_distance_meters,grasp_point_speed_mps,"
            + "success_hold_seconds,elapsed_seconds,workspace_safe,finite_physics,"
            + "termination_reason";

        static readonly object Gate = new object();
        static readonly Dictionary<Dg5fGraspPointReachAgent, int> AreaByAgent =
            new Dictionary<Dg5fGraspPointReachAgent, int>();
        static readonly HashSet<int> StartedEpisodes = new HashSet<int>();
        static readonly HashSet<int> RecordedEpisodes = new HashSet<int>();

        static bool _configured;
        static bool _enabled;
        static int _episodeCount;
        static int _baseSeed;
        static string _csvPath;

        public static bool IsEnabled
        {
            get
            {
                EnsureConfigured();
                return _enabled;
            }
        }

        public static void Register(Dg5fGraspPointReachAgent agent)
        {
            if (agent == null) throw new ArgumentNullException(nameof(agent));
            EnsureConfigured();
            if (!_enabled) return;

            int area = ParseAreaIndex(agent.transform.root.name);
            lock (Gate)
            {
                if (AreaByAgent.ContainsKey(agent)) return;
                if (AreaByAgent.ContainsValue(area))
                    throw new InvalidOperationException(
                        $"[ArmReachEvaluation] Duplicate evaluation area {area}.");
                AreaByAgent.Add(agent, area);
            }
        }

        public static void Unregister(Dg5fGraspPointReachAgent agent)
        {
            if (agent == null || !_configured || !_enabled) return;
            lock (Gate)
                AreaByAgent.Remove(agent);
        }

        public static bool TryBeginEpisode(
            Dg5fGraspPointReachAgent agent,
            out int episode,
            out int seed)
        {
            if (agent == null) throw new ArgumentNullException(nameof(agent));
            EnsureConfigured();
            episode = -1;
            seed = 0;
            if (!_enabled) return false;

            lock (Gate)
            {
                if (!AreaByAgent.TryGetValue(agent, out int area))
                    throw new InvalidOperationException(
                        "[ArmReachEvaluation] Agent was not registered.");

                for (int localEpisode = 0; ; localEpisode++)
                {
                    int candidate = EpisodeForArea(area, localEpisode, AreaCount);
                    if (candidate >= _episodeCount) return false;
                    if (!StartedEpisodes.Add(candidate)) continue;
                    episode = candidate;
                    seed = checked(_baseSeed + candidate);
                    return true;
                }
            }
        }

        public static void RecordEpisode(
            Dg5fGraspPointReachAgent agent,
            int episode,
            bool success,
            float finalDistanceMeters,
            float graspPointSpeedMetersPerSecond,
            float successHoldSeconds,
            float elapsedSeconds,
            bool workspaceSafe,
            bool finitePhysics,
            string terminationReason)
        {
            EnsureConfigured();
            if (!_enabled) return;

            lock (Gate)
            {
                if (!AreaByAgent.ContainsKey(agent))
                    throw new InvalidOperationException(
                        "[ArmReachEvaluation] Unknown evaluation agent.");
                if (episode < 0 || episode >= _episodeCount)
                    throw new ArgumentOutOfRangeException(nameof(episode));
                if (!StartedEpisodes.Contains(episode))
                    throw new InvalidOperationException(
                        $"[ArmReachEvaluation] Episode {episode} was not assigned.");
                if (!RecordedEpisodes.Add(episode))
                    throw new InvalidOperationException(
                        $"[ArmReachEvaluation] Episode {episode} was recorded twice.");

                int seed = checked(_baseSeed + episode);
                string row = string.Join(
                    ",",
                    episode.ToString(CultureInfo.InvariantCulture),
                    seed.ToString(CultureInfo.InvariantCulture),
                    success ? "1" : "0",
                    FormatFloat(finalDistanceMeters),
                    FormatFloat(graspPointSpeedMetersPerSecond),
                    FormatFloat(successHoldSeconds),
                    FormatFloat(elapsedSeconds),
                    workspaceSafe ? "1" : "0",
                    finitePhysics ? "1" : "0",
                    EscapeCsv(terminationReason));
                File.AppendAllText(
                    _csvPath,
                    row + Environment.NewLine,
                    new UTF8Encoding(false));

                if (RecordedEpisodes.Count == _episodeCount)
                {
                    Debug.Log(
                        $"[ArmReachEvaluation] Completed {_episodeCount} episodes: {_csvPath}");
                }
            }
        }

        public static int EpisodeForArea(int area, int localEpisode, int areaCount)
        {
            if (areaCount <= 0) throw new ArgumentOutOfRangeException(nameof(areaCount));
            if (area < 0 || area >= areaCount) throw new ArgumentOutOfRangeException(nameof(area));
            if (localEpisode < 0) throw new ArgumentOutOfRangeException(nameof(localEpisode));
            return checked(localEpisode * areaCount + area);
        }

        public static int EpisodesForArea(int episodeCount, int area, int areaCount)
        {
            if (episodeCount < 0) throw new ArgumentOutOfRangeException(nameof(episodeCount));
            if (areaCount <= 0) throw new ArgumentOutOfRangeException(nameof(areaCount));
            if (area < 0 || area >= areaCount) throw new ArgumentOutOfRangeException(nameof(area));
            if (area >= episodeCount) return 0;
            return (episodeCount - 1 - area) / areaCount + 1;
        }

        static void EnsureConfigured()
        {
            if (_configured) return;
            lock (Gate)
            {
                if (_configured) return;
                Configure(Environment.GetCommandLineArgs());
                _configured = true;
            }
        }

        static void Configure(string[] args)
        {
            string episodesText = FindArgument(args, EpisodesArgument);
            string baseSeedText = FindArgument(args, BaseSeedArgument);
            string csvText = FindArgument(args, CsvArgument);
            int supplied = (episodesText != null ? 1 : 0)
                + (baseSeedText != null ? 1 : 0)
                + (csvText != null ? 1 : 0);

            if (supplied == 0)
            {
                _enabled = false;
                return;
            }
            if (supplied != 3)
                throw new InvalidOperationException(
                    $"[ArmReachEvaluation] {EpisodesArgument}, {BaseSeedArgument}, and "
                    + $"{CsvArgument} must be supplied together.");
            if (!int.TryParse(
                    episodesText,
                    NumberStyles.Integer,
                    CultureInfo.InvariantCulture,
                    out _episodeCount)
                || _episodeCount < AreaCount)
            {
                throw new InvalidOperationException(
                    $"[ArmReachEvaluation] Episode count must be at least {AreaCount}.");
            }
            if (!int.TryParse(
                    baseSeedText,
                    NumberStyles.Integer,
                    CultureInfo.InvariantCulture,
                    out _baseSeed)
                || _baseSeed < 0
                || (long)_baseSeed + _episodeCount - 1 > int.MaxValue)
            {
                throw new InvalidOperationException(
                    $"[ArmReachEvaluation] Invalid base seed: {baseSeedText}");
            }

            _csvPath = Path.GetFullPath(csvText);
            string directory = Path.GetDirectoryName(_csvPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(
                _csvPath,
                CsvHeader + Environment.NewLine,
                new UTF8Encoding(false));
            _enabled = true;
            Debug.Log(
                $"[ArmReachEvaluation] episodes={_episodeCount}, "
                + $"baseSeed={_baseSeed}, csv={_csvPath}");
        }

        static string FindArgument(string[] args, string name)
        {
            for (int index = 0; index < args.Length; index++)
            {
                if (args[index] == name)
                {
                    if (index + 1 >= args.Length)
                        throw new InvalidOperationException(
                            $"[ArmReachEvaluation] Missing value after {name}.");
                    return args[index + 1];
                }

                string prefix = name + "=";
                if (args[index].StartsWith(prefix, StringComparison.Ordinal))
                    return args[index].Substring(prefix.Length);
            }
            return null;
        }

        static int ParseAreaIndex(string rootName)
        {
            int separator = rootName.LastIndexOf('_');
            if (separator < 0
                || !int.TryParse(
                    rootName.Substring(separator + 1),
                    NumberStyles.None,
                    CultureInfo.InvariantCulture,
                    out int area)
                || area < 0
                || area >= AreaCount)
            {
                throw new InvalidOperationException(
                    $"[ArmReachEvaluation] Cannot parse area 0..{AreaCount - 1} "
                    + $"from root name '{rootName}'.");
            }
            return area;
        }

        static string FormatFloat(float value)
        {
            return value.ToString("R", CultureInfo.InvariantCulture);
        }

        static string EscapeCsv(string value)
        {
            value = value ?? string.Empty;
            if (!value.Contains(",") && !value.Contains("\"") && !value.Contains("\n"))
                return value;
            return "\"" + value.Replace("\"", "\"\"") + "\"";
        }
    }
}
