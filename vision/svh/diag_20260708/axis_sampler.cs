var targets = new string[]{"right_hand_z","right_hand_virtual_i","right_hand_p","right_hand_l","right_hand_o","right_hand_k"};
var rootGo = GameObject.Find("ur5e_svh");
if (rootGo == null) return "ERROR no root";
var abs = rootGo.GetComponentsInChildren<ArticulationBody>(true);
var map = new Dictionary<string, ArticulationBody>();
foreach (var n in targets) { foreach (var b in abs) { if (b.name == n) { map[n] = b; break; } } }
if (map.Count != 6) return "ERROR resolved " + map.Count + "/6";
string outPath = "C:/Users/dltmd/AppData/Local/Temp/claude/C--Users-dltmd-Desktop-KDT/cdbb52d1-44f1-4244-a998-555745a3a6f7/scratchpad/axis_samples.csv";
var sb = new System.Text.StringBuilder();
sb.AppendLine("t,fixedTime,link,wmag,angleFolded,jointPosDeg,targetDeg");
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
        Vector3 w = ab.angularVelocity;
        float wm = w.magnitude;
        Vector3 ax = (ab.transform.rotation * ab.anchorRotation) * Vector3.right;
        float ang = wm > 1e-6f ? Vector3.Angle(w, ax) : -1f;
        if (ang > 90f) ang = 180f - ang;
        float jp = ab.jointPosition.dofCount > 0 ? ab.jointPosition[0] * Mathf.Rad2Deg : 0f;
        sb.AppendLine(t.ToString("F3") + "," + Time.fixedTime.ToString("F3") + "," + kv.Key + "," + wm.ToString("F5") + "," + ang.ToString("F3") + "," + jp.ToString("F3") + "," + ab.xDrive.target.ToString("F2"));
      }
    }
    return;
  }
  EditorApplication.update -= cb;
  File.WriteAllText(outPath, sb.ToString());
};
EditorApplication.update += cb;
return "sampler armed, 12s window, out=" + outPath;
