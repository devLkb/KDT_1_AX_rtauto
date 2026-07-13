// 손 서브트리만 Instantiate 프로브 (팔 vs 손 이분법)
// right_hand_virtual_l 이하만 복제 -> 새 인스턴스는 부모 AB가 없어 루트 articulation이 됨 -> immovable.
// 콜라이더 전부 제거(변형 A에서 결백 확정, 자기충돌 confound 방지). 스크립트 없음(mimic도 없음 = mimic-OFF 조건).
// 비교 기준: 원본 mimic-OFF 테스트(ring 21, pinky 11, idx/mid distal 67~82 p2p).

if (!EditorApplication.isPlaying) return "ERROR: not playing";
var orig = GameObject.Find("ur5e_svh");
if (orig == null) return "ERROR: ur5e_svh not found";
Transform srcHand = null;
foreach (var t in orig.GetComponentsInChildren<Transform>())
    if (t.name == "right_hand_virtual_l") { srcHand = t; break; }
if (srcHand == null) return "ERROR: right_hand_virtual_l not found";

foreach (var n in new[] { "ur5e_svh_COPY", "svh_FINGER_COPY" }) {
    var g = GameObject.Find(n);
    if (g != null) UnityEngine.Object.DestroyImmediate(g);
}

var hand = UnityEngine.Object.Instantiate(srcHand.gameObject,
    srcHand.position + new Vector3(2.0f, 0.2f, 0f), srcHand.rotation);
hand.name = "svh_FINGER_COPY";

// [변형] t 서브트리(t + fftip) 제거
foreach (var tr in hand.GetComponentsInChildren<Transform>(true).ToArray())
    if (tr != null && tr.name == "right_hand_t") { UnityEngine.Object.DestroyImmediate(tr.gameObject); break; }

// 루트 AB immovable + 관성 동결 + 콜라이더 제거
var rootAb = hand.GetComponent<ArticulationBody>();
if (rootAb == null) return "ERROR: no AB on hand root";
rootAb.immovable = true;
int frozen = 0, colsKilled = 0;
foreach (var ab in hand.GetComponentsInChildren<ArticulationBody>()) {
    var it = ab.inertiaTensor; var ir = ab.inertiaTensorRotation; var com = ab.centerOfMass;
    ab.automaticInertiaTensor = false; ab.inertiaTensor = it; ab.inertiaTensorRotation = ir;
    if (ab.automaticCenterOfMass) { ab.automaticCenterOfMass = false; ab.centerOfMass = com; }
    frozen++;
}
foreach (var col in hand.GetComponentsInChildren<Collider>(true).ToArray()) {
    UnityEngine.Object.DestroyImmediate(col); colsKilled++;
}
// 혹시 서브트리에 붙어있을 스크립트 제거(안전)
foreach (var mb in hand.GetComponentsInChildren<MonoBehaviour>(true).ToArray()) {
    var n = mb.GetType().Name;
    if (n.StartsWith("Svh") || n == "HandSliderUI" || n == "MimicJointController")
        UnityEngine.Object.DestroyImmediate(mb);
}

var fist = new Dictionary<string, float> {
    {"right_hand_a", 55.6f}, {"right_hand_z", 56.6f},
    {"right_hand_p", 76.4f}, {"right_hand_l", 45.8f},
    {"right_hand_o", 76.4f}, {"right_hand_k", 45.8f},
    {"right_hand_j", 56.3f}, {"right_hand_i", 56.3f},
    {"right_hand_virtual_i", 33.4f}
};
var bodies = new Dictionary<string, ArticulationBody>();
foreach (var ab in hand.GetComponentsInChildren<ArticulationBody>())
    if (fist.ContainsKey(ab.name)) bodies[ab.name] = ab;
if (bodies.Count == 0) return "ERROR: no drive joints found in subtree";

Action<float> setHand = (frac) => {
    foreach (var kv in bodies) {
        var d = kv.Value.xDrive; d.target = fist[kv.Key] * frac; kv.Value.xDrive = d;
    }
};
setHand(0f);

string outDir = @"C:\Users\dltmd\AppData\Local\Temp\claude\C--Users-dltmd-Desktop-KDT\4c2a4861-2a65-43c6-921b-9d9d0cd79dd1\scratchpad";
string csvPath = Path.Combine(outDir, "finger_not_log.csv");
string sumPath = Path.Combine(outDir, "finger_not_summary.txt");
var keys = bodies.Keys.OrderBy(k => k).ToList();
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
        if (phase == 0 && t > 1.5f) { setHand(1f); phase = 1; }
        if (phase == 1 && t > 10.5f) {
            File.WriteAllLines(csvPath, rows);
            var data = rows.Skip(1).Select(r => r.Split(',').Select(float.Parse).ToArray())
                           .Where(c => c[0] > 3.5f).ToList();
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("DONE rows=" + (rows.Count - 1));
            for (int i = 0; i < keys.Count; i++) {
                var vals = data.Select(c => c[i + 1]).ToList();
                float tgt = fist[keys[i]];
                sb.AppendLine(string.Format("{0,-10} p2p={1,7:F2}  mean|err|={2,7:F2}  final={3,7:F2}  tgt={4:F1}",
                    keys[i].Replace("right_hand_", ""), vals.Max() - vals.Min(),
                    vals.Average(v => Mathf.Abs(v - tgt)), vals.Last(), tgt));
            }
            File.WriteAllText(sumPath, sb.ToString());
            var g = GameObject.Find("svh_FINGER_COPY");
            if (g != null) UnityEngine.Object.Destroy(g);
            EditorApplication.update -= cb;
        }
    } catch (Exception e) {
        try { File.WriteAllText(sumPath, "EXCEPTION: " + e); } catch {}
        EditorApplication.update -= cb;
    }
};
EditorApplication.update += cb;

return "SETUP OK (finger-no-t). bodies=" + hand.GetComponentsInChildren<ArticulationBody>().Length +
       " frozen=" + frozen + " colsRemoved=" + colsKilled + " rootImmovable=" + rootAb.immovable;
