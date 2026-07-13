var targets = new string[]{"z","virtual_i","p","l","o","k"};
var rootGo = GameObject.Find("ur5e_svh");
if (rootGo == null) return "ERROR: ur5e_svh root not found";
var abs = rootGo.GetComponentsInChildren<ArticulationBody>(true);
var sb = new System.Text.StringBuilder();
System.Func<Vector3,string> v3 = v => string.Format("({0:F6}, {1:F6}, {2:F6})", v.x, v.y, v.z);
System.Func<Quaternion,string> qs = q => string.Format("euler({0:F3}, {1:F3}, {2:F3}) quat({3:F5}, {4:F5}, {5:F5}, {6:F5})", q.eulerAngles.x, q.eulerAngles.y, q.eulerAngles.z, q.x, q.y, q.z, q.w);
foreach (var t in targets) {
  ArticulationBody ab = null;
  foreach (var b in abs) { if (b.name == t) { ab = b; break; } }
  if (ab == null) foreach (var b in abs) { if (b.name == "right_hand_" + t) { ab = b; break; } }
  sb.AppendLine("==== target: " + t + " ====");
  if (ab == null) { sb.AppendLine("  NOT FOUND"); continue; }
  sb.AppendLine("  gameObject: " + ab.name + "  jointType: " + ab.jointType + "  twistLock: " + ab.twistLock);
  ArticulationBody pab = ab.transform.parent != null ? ab.transform.parent.GetComponentInParent<ArticulationBody>() : null;
  sb.AppendLine("  parentBody: " + (pab != null ? pab.name : "NULL"));
  sb.AppendLine("  matchAnchors: " + ab.matchAnchors);
  sb.AppendLine("  anchorPosition: " + v3(ab.anchorPosition));
  sb.AppendLine("  anchorRotation: " + qs(ab.anchorRotation));
  sb.AppendLine("  parentAnchorPosition: " + v3(ab.parentAnchorPosition));
  sb.AppendLine("  parentAnchorRotation: " + qs(ab.parentAnchorRotation));
  // world-space anchor comparison
  Vector3 cw = ab.transform.TransformPoint(ab.anchorPosition);
  Quaternion cwr = ab.transform.rotation * ab.anchorRotation;
  if (pab != null) {
    Vector3 pw = pab.transform.TransformPoint(ab.parentAnchorPosition);
    Quaternion pwr = pab.transform.rotation * ab.parentAnchorRotation;
    sb.AppendLine("  childAnchorWorld:  " + v3(cw));
    sb.AppendLine("  parentAnchorWorld: " + v3(pw));
    sb.AppendLine("  anchorMismatch: dist_mm=" + ((cw - pw).magnitude * 1000f).ToString("F4") + "  angle_deg=" + Quaternion.Angle(cwr, pwr).ToString("F4"));
    Vector3 axC = cwr * Vector3.right;
    Vector3 axP = pwr * Vector3.right;
    sb.AppendLine("  twistAxisWorld(child-side):  " + v3(axC));
    sb.AppendLine("  twistAxisWorld(parent-side): " + v3(axP));
    sb.AppendLine("  axisChildVsParent_deg: " + Vector3.Angle(axC, axP).ToString("F4"));
  }
  // scale chain up to robot root
  var tr = ab.transform;
  bool anyScale = false;
  var chain = new System.Text.StringBuilder();
  while (tr != null) {
    Vector3 s = tr.localScale;
    if (Mathf.Abs(s.x-1f) > 1e-4f || Mathf.Abs(s.y-1f) > 1e-4f || Mathf.Abs(s.z-1f) > 1e-4f) {
      anyScale = true;
      chain.Append("    " + tr.name + " localScale=" + v3(s) + "\n");
    }
    if (tr.gameObject == rootGo) break;
    tr = tr.parent;
  }
  sb.AppendLine("  lossyScale(link): " + v3(ab.transform.lossyScale));
  sb.AppendLine("  nonUnitScaleInAncestry: " + (anyScale ? ("YES\n" + chain.ToString()) : "none"));
  // current drive x setup
  var d = ab.xDrive;
  sb.AppendLine("  xDrive: lower=" + d.lowerLimit.ToString("F2") + " upper=" + d.upperLimit.ToString("F2") + " stiffness=" + d.stiffness + " damping=" + d.damping + " forceLimit=" + d.forceLimit + " target=" + d.target.ToString("F2"));
  sb.AppendLine("  anchor raw local T pos: " + v3(ab.transform.localPosition) + " rot " + qs(ab.transform.localRotation));
}
sb.AppendLine("isPlaying: " + EditorApplication.isPlaying);
return sb.ToString();
