using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// Coordinates deterministic built-player evaluation across the 20 areas.
    /// Evaluation is enabled only when all three --dg5f-eval-* arguments exist.
    /// </summary>
    public static class Dg5fEvaluationSession
    {
        public const int AreaCount = 20;
        public const string EpisodesArgument = "--dg5f-eval-episodes";
        public const string BaseSeedArgument = "--dg5f-eval-base-seed";
        public const string CsvArgument = "--dg5f-eval-csv";

        const string CsvHeader =
            "episode_id,seed,area,success,failure_reason,completion_seconds,"
            + "reach_success,first_reach_seconds,final_distance_meters,"
            + "best_distance_meters,max_contact_hold_seconds";

        static readonly object Gate = new object();
        static readonly Dictionary<Dg5fGraspAgent, int> AreaByAgent =
            new Dictionary<Dg5fGraspAgent, int>();
        static readonly HashSet<int> StartedEpisodeIds = new HashSet<int>();
        static readonly HashSet<int> RecordedEpisodeIds = new HashSet<int>();

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

        public static void Register(Dg5fGraspAgent agent)
        {
            if (agent == null) throw new ArgumentNullException(nameof(agent));
            EnsureConfigured();
            if (!_enabled) return;

            int area = ParseAreaIndex(agent.transform.root.name);
            lock (Gate)
            {
                if (AreaByAgent.ContainsValue(area))
                    throw new InvalidOperationException(
                        $"[Dg5fEvaluation] Duplicate evaluation area index {area}.");
                AreaByAgent.Add(agent, area);
            }
        }

        public static bool TryBeginEpisode(
            Dg5fGraspAgent agent,
            out int episodeId,
            out int seed)
        {
            EnsureConfigured();
            episodeId = -1;
            seed = 0;
            if (!_enabled) return false;

            lock (Gate)
            {
                if (!AreaByAgent.TryGetValue(agent, out int area))
                    throw new InvalidOperationException(
                        "[Dg5fEvaluation] Agent was not registered before its episode began.");

                for (int localEpisode = 0; ; localEpisode++)
                {
                    int candidate = EpisodeIdForArea(area, localEpisode, AreaCount);
                    if (candidate >= _episodeCount) return false;
                    if (!StartedEpisodeIds.Add(candidate)) continue;
                    episodeId = candidate;
                    seed = checked(_baseSeed + candidate);
                    return true;
                }
            }
        }

        public static void RecordEpisode(
            Dg5fGraspAgent agent,
            int episodeId,
            bool success,
            string failureReason,
            float completionSeconds,
            bool reachSuccess,
            float firstReachSeconds,
            float finalDistanceMeters,
            float bestDistanceMeters,
            float maxContactHoldSeconds)
        {
            EnsureConfigured();
            if (!_enabled) return;

            lock (Gate)
            {
                if (!AreaByAgent.TryGetValue(agent, out int area))
                    throw new InvalidOperationException("[Dg5fEvaluation] Unknown evaluation agent.");
                if (episodeId < 0 || episodeId >= _episodeCount)
                    throw new ArgumentOutOfRangeException(nameof(episodeId));
                if (!StartedEpisodeIds.Contains(episodeId))
                    throw new InvalidOperationException(
                        $"[Dg5fEvaluation] Episode {episodeId} was not assigned.");
                if (!RecordedEpisodeIds.Add(episodeId))
                    throw new InvalidOperationException(
                        $"[Dg5fEvaluation] Episode {episodeId} was recorded twice.");

                int seed = checked(_baseSeed + episodeId);
                string row = string.Join(
                    ",",
                    episodeId.ToString(CultureInfo.InvariantCulture),
                    seed.ToString(CultureInfo.InvariantCulture),
                    area.ToString(CultureInfo.InvariantCulture),
                    success ? "1" : "0",
                    EscapeCsv(success ? "None" : failureReason),
                    FormatFloat(completionSeconds),
                    reachSuccess ? "1" : "0",
                    FormatFloat(firstReachSeconds),
                    FormatFloat(finalDistanceMeters),
                    FormatFloat(bestDistanceMeters),
                    FormatFloat(maxContactHoldSeconds));
                File.AppendAllText(_csvPath, row + Environment.NewLine, new UTF8Encoding(false));

                if (RecordedEpisodeIds.Count == _episodeCount)
                {
                    Debug.Log(
                        $"[Dg5fEvaluation] Completed {_episodeCount} episodes: {_csvPath}");
                }
            }
        }

        public static int EpisodeIdForArea(int area, int localEpisode, int areaCount)
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
                    $"[Dg5fEvaluation] {EpisodesArgument}, {BaseSeedArgument}, and {CsvArgument} "
                    + "must be supplied together.");
            if (!int.TryParse(episodesText, NumberStyles.Integer, CultureInfo.InvariantCulture, out _episodeCount)
                || _episodeCount <= 0)
            {
                throw new InvalidOperationException(
                    $"[Dg5fEvaluation] Invalid episode count: {episodesText}");
            }
            if (!int.TryParse(baseSeedText, NumberStyles.Integer, CultureInfo.InvariantCulture, out _baseSeed)
                || _baseSeed < 0
                || (long)_baseSeed + _episodeCount - 1 > int.MaxValue)
            {
                throw new InvalidOperationException(
                    $"[Dg5fEvaluation] Invalid base seed: {baseSeedText}");
            }
            if (_episodeCount < AreaCount)
                throw new InvalidOperationException(
                    $"[Dg5fEvaluation] Episode count must be at least the {AreaCount} area count.");

            _csvPath = Path.GetFullPath(csvText);
            string directory = Path.GetDirectoryName(_csvPath);
            if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);
            File.WriteAllText(_csvPath, CsvHeader + Environment.NewLine, new UTF8Encoding(false));
            _enabled = true;
            Debug.Log(
                $"[Dg5fEvaluation] episodes={_episodeCount}, baseSeed={_baseSeed}, csv={_csvPath}");
        }

        static string FindArgument(string[] args, string name)
        {
            for (int index = 0; index < args.Length; index++)
            {
                if (args[index] == name)
                {
                    if (index + 1 >= args.Length)
                        throw new InvalidOperationException(
                            $"[Dg5fEvaluation] Missing value after {name}.");
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
                    $"[Dg5fEvaluation] Cannot parse area index 0..{AreaCount - 1} from '{rootName}'.");
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
            if (!value.Contains(",") && !value.Contains("\"") && !value.Contains("\n")) return value;
            return "\"" + value.Replace("\"", "\"\"") + "\"";
        }
    }
}
