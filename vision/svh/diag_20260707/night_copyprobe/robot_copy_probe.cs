// 로봇 전체 Instantiate 복제 프로브 (트리/인스턴스 가설 판별)
// 복제본에서 UDP·슬라이더·로거만 제거(SvhSelfCollisionIgnore/MimicJointController/SvhInitialPoseSync 유지),
// 손 9관절 0 -> 주먹 스텝 -> 8초 홀드. 원본 로봇·씬 파일은 건드리지 않음(런타임 비영속).

if (!EditorApplication.isPlaying) return "ERROR: not playing";
var orig = GameObject.Find("ur5e_svh");
if (orig == null) return "ERROR: ur5e_svh not found";

var oldCopy = GameObject.Find("ur5e_svh_COPY");
if (oldCopy != null) UnityEngine.Object.DestroyImmediate(oldCopy);

var copy = UnityEngine.Object.Instantiate(orig, orig.transform.position + new Vector3(2.5f, 0f, 0f), orig.transform.rotation);
copy.name = "ur5e_svh_COPY";

// 복제본에서 외부입력/로깅 스크립트 제거 — RequireComponent 의존자 먼저, SvhReceiver 마지막
string[] killOrder = { "SvhHandDriver", "SvhJointLogger", "HandSliderUI", "SvhReceiver" };
int killed = 0;
foreach (var typeName in killOrder) {
    foreach (var mb in copy.GetComponentsInChildren<MonoBehaviour>(true).ToArray()) {
        if (mb != null && mb.GetType().Name == typeName) { UnityEngine.Object.DestroyImmediate(mb); killed++; }
    }
}
var kept = copy.GetComponentsInChildren<MonoBehaviour>(true).Select(m => m.GetType().Name).Distinct().ToList();

// 손 9 구동관절 (링크명 -> 주먹 목표 deg)
var fist = new Dictionary<string, float> {
    {"right_hand_a", 55.6f}, {"right_hand_z", 56.6f},
    {"right_hand_p", 76.4f}, {"right_hand_l", 45.8f},
    {"right_hand_o", 76.4f}, {"right_hand_k", 45.8f},
    {"right_hand_j", 56.3f}, {"right_hand_i", 56.3f},
    {"right_hand_virtual_i", 33.4f}
};
var bodies = new Dictionary<string, ArticulationBody>();
foreach (var ab in copy.GetComponentsInChildren<ArticulationBody>())
    if (fist.ContainsKey(ab.name)) bodies[ab.name] = ab;
if (bodies.Count != 9) return "ERROR: drive joints found " + bodies.Count + "/9";

Action<float> setHand = (frac) => {
    foreach (var kv in bodies) {
        var d = kv.Value.xDrive; d.target = fist[kv.Key] * frac; kv.Value.xDrive = d;
    }
};
setHand(0f); // phase 0: 펴기

string outDir = @"C:\Users\dltmd\AppData\Local\Temp\claude\C--Users-dltmd-Desktop-KDT\4c2a4861-2a65-43c6-921b-9d9d0cd79dd1\scratchpad";
string csvPath = Path.Combine(outDir, "copy_probe_log.csv");
string sumPath = Path.Combine(outDir, "copy_probe_summary.txt");
var keys = fist.Keys.OrderBy(k => k).ToList();
var rows = new List<string>();
rows.Add("t," + string.Join(",", keys.Select(k => k.Replace("right_hand_", ""))));
double t0 = EditorApplication.timeSinceStartup;
int phase = 0;
EditorApplication.CallbackFunction cb = null;
cb = () => {
    try {
        if (!EditorApplication.isPlaying) {
            File.WriteAllLines(csvPath, rows);
            File.WriteAllText(sumPath, "ABORTED at " + rows.Count + " rows");
            EditorApplication.update -= cb; return;
        }
        float t = (float)(EditorApplication.timeSinceStartup - t0);
        rows.Add(t.ToString("F3") + "," + string.Join(",",
            keys.Select(k => (bodies[k].jointPosition[0] * Mathf.Rad2Deg).ToString("F3"))));
        if (phase == 0 && t > 1.5f) { setHand(1f); phase = 1; }  // 주먹 스텝
        if (phase == 1 && t > 10.5f) {
            File.WriteAllLines(csvPath, rows);
            // 정상상태 = 스텝 후 2초 제외(3.5s~)
            var data = rows.Skip(1).Select(r => r.Split(',').Select(float.Parse).ToArray())
                           .Where(c => c[0] > 3.5f).ToList();
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("DONE rows=" + (rows.Count - 1) + " killed=" + 0);
            for (int i = 0; i < keys.Count; i++) {
                var vals = data.Select(c => c[i + 1]).ToList();
                float tgt = fist[keys[i]];
                sb.AppendLine(string.Format("{0,-10} p2p={1,7:F2}  mean|err|={2,7:F2}  final={3,7:F2}  tgt={4:F1}",
                    keys[i].Replace("right_hand_", ""), vals.Max() - vals.Min(),
                    vals.Average(v => Mathf.Abs(v - tgt)), vals.Last(), tgt));
            }
            File.WriteAllText(sumPath, sb.ToString());
            var g = GameObject.Find("ur5e_svh_COPY");
            if (g != null) UnityEngine.Object.Destroy(g);
            EditorApplication.update -= cb;
        }
    } catch (Exception e) {
        try { File.WriteAllText(sumPath, "EXCEPTION: " + e); } catch {}
        EditorApplication.update -= cb;
    }
};
EditorApplication.update += cb;

return "SETUP OK. killed=" + killed + " keptScripts=[" + string.Join(",", kept) + "] joints=" + bodies.Count;
