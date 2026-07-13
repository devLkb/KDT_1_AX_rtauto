var targets = new string[]{"right_hand_z","right_hand_virtual_i","right_hand_p","right_hand_l","right_hand_o","right_hand_k","right_hand_e1","right_hand_e2"};
var rootGo = GameObject.Find("ur5e_svh");
if (rootGo == null) return "ERROR no root";
var abs = rootGo.GetComponentsInChildren<ArticulationBody>(true);
var map = new Dictionary<string, ArticulationBody>();
foreach (var n in targets) { foreach (var b in abs) { if (b.name == n) { map[n] = b; break; } } }
string outPath = "C:/Users/dltmd/AppData/Local/Temp/claude/C--Users-dltmd-Desktop-KDT/cdbb52d1-44f1-4244-a998-555745a3a6f7/scratchpad/axis_samples_rel.csv";
var sb = new System.Text.StringBuilder();
sb.AppendLine("t,link,wmag,angleFolded,wrelmag,angleRelFolded,jointPosDeg,targetDeg");
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
        var pab = ab.transform.parent != null ? ab.transform.parent.GetComponentInParent<ArticulationBody>() : null;
        Vector3 w = ab.angularVelocity;
        Vector3 wrel = pab != null ? w - pab.angularVelocity : w;
        Vector3 ax = (ab.transform.rotation * ab.anchorRotation) * Vector3.right;
        float wm = w.magnitude; float wrm = wrel.magnitude;
        float ang = wm > 1e-6f ? Vector3.Angle(w, ax) : -1f; if (ang > 90f) ang = 180f - ang;
        float angR = wrm > 1e-6f ? Vector3.Angle(wrel, ax) : -1f; if (angR > 90f) angR = 180f - angR;
        float jp = ab.jointPosition.dofCount > 0 ? ab.jointPosition[0] * Mathf.Rad2Deg : 0f;
        sb.AppendLine(t.ToString("F3") + "," + kv.Key + "," + wm.ToString("F5") + "," + ang.ToString("F3") + "," + wrm.ToString("F5") + "," + angR.ToString("F3") + "," + jp.ToString("F3") + "," + (ab.jointType == ArticulationJointType.RevoluteJoint ? ab.xDrive.target.ToString("F2") : "-1"));
      }
    }
    return;
  }
  EditorApplication.update -= cb;
  File.WriteAllText(outPath, sb.ToString());
};
EditorApplication.update += cb;
return "rel sampler armed";
