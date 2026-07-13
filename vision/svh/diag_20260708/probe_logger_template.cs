var targets = new string[]{"right_hand_z","right_hand_virtual_i","right_hand_p","right_hand_l","right_hand_o","right_hand_k"};
var rootGo = GameObject.Find("ur5e_svh");
if (rootGo == null) return "ERROR no root";
var abs = rootGo.GetComponentsInChildren<ArticulationBody>(true);
var map = new Dictionary<string, ArticulationBody>();
foreach (var n in targets) { foreach (var b in abs) { if (b.name == n) { map[n] = b; break; } } }
if (map.Count != 6) return "ERROR resolved " + map.Count + "/6";
string outPath = "OUTPATH";
var sb = new System.Text.StringBuilder();
sb.AppendLine("t,link,jointPosDeg,targetDeg,lowerDeg,upperDeg");
double t0 = EditorApplication.timeSinceStartup;
float lastFixed = -1f;
EditorApplication.CallbackFunction cb = null;
cb = () => {
  double t = EditorApplication.timeSinceStartup - t0;
  bool done = t > 12.0 || !EditorApplication.isPlaying;
  if (!done) {
    if (Time.fixedTime != lastFixed) {
      lastFixed = Time.fixedTime;
      foreach (var kv in map) {
        var ab = kv.Value;
        float jp = ab.jointPosition.dofCount > 0 ? ab.jointPosition[0] * Mathf.Rad2Deg : 0f;
        var d = ab.xDrive;
        sb.AppendLine(t.ToString("F3") + "," + kv.Key + "," + jp.ToString("F3") + "," + d.target.ToString("F2") + "," + d.lowerLimit.ToString("F2") + "," + d.upperLimit.ToString("F2"));
      }
    }
    return;
  }
  EditorApplication.update -= cb;
  File.WriteAllText(outPath, sb.ToString());
};
EditorApplication.update += cb;
return "probe logger armed -> " + outPath;
