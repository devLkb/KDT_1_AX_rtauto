// clone-props 재현체 실험 (DEBUG_OSCILLATION_20260707.md §7)
// 로봇 virtual_l -> l -> p 체인의 ArticulationBody 속성을 "전부" 복제한
// 고립 3-바디 체인을 런타임 생성 후 40deg급 스텝(주먹 목표) 응답을 샘플링.
// 비영속: 플레이 중 생성, 종료 시 소멸. 씬/코드 수정 없음.

if (!EditorApplication.isPlaying) return "ERROR: not playing";

var robotRoot = GameObject.Find("ur5e_svh");
if (robotRoot == null) return "ERROR: ur5e_svh not found";

Func<string, ArticulationBody> find = (name) => {
    foreach (var ab in robotRoot.GetComponentsInChildren<ArticulationBody>())
        if (ab.name == name) return ab;
    return null;
};
var srcRootAb = find("right_hand_virtual_l");
var srcB = find("right_hand_l");
var srcC = find("right_hand_p");
if (srcRootAb == null || srcB == null || srcC == null) return "ERROR: source links not found";

// 기존 실험체 잔재 제거
var old = GameObject.Find("TESTG_root");
if (old != null) UnityEngine.Object.DestroyImmediate(old);

// --- 체인 생성 ---
var goRoot = new GameObject("TESTG_root");
goRoot.transform.position = new Vector3(3f, 1f, 0.5f);
goRoot.transform.rotation = srcRootAb.transform.rotation; // 축 방향 동일하게
var goB = new GameObject("TESTG_B");
goB.transform.SetParent(goRoot.transform, false);
goB.transform.localPosition = srcB.transform.localPosition;
goB.transform.localRotation = srcB.transform.localRotation;
var goC = new GameObject("TESTG_C");
goC.transform.SetParent(goB.transform, false);
goC.transform.localPosition = srcC.transform.localPosition;
goC.transform.localRotation = srcC.transform.localRotation;

// --- 속성 전체 복제 ---
Action<ArticulationBody, ArticulationBody, bool> clone = (src, dst, isRoot) => {
    dst.useGravity = src.useGravity;
    dst.mass = src.mass;
    dst.linearDamping = src.linearDamping;
    dst.angularDamping = src.angularDamping;
    dst.jointFriction = src.jointFriction;
    dst.maxAngularVelocity = src.maxAngularVelocity;
    dst.maxLinearVelocity = src.maxLinearVelocity;
    dst.maxJointVelocity = src.maxJointVelocity;
    dst.maxDepenetrationVelocity = src.maxDepenetrationVelocity;
    dst.solverIterations = src.solverIterations;
    dst.solverVelocityIterations = src.solverVelocityIterations;
    dst.collisionDetectionMode = src.collisionDetectionMode;
    dst.automaticCenterOfMass = src.automaticCenterOfMass;
    if (!src.automaticCenterOfMass) dst.centerOfMass = src.centerOfMass;
    dst.automaticInertiaTensor = false;                 // 로봇 실측값 고정 이식
    dst.inertiaTensor = src.inertiaTensor;
    dst.inertiaTensorRotation = src.inertiaTensorRotation;
    if (isRoot) { dst.immovable = true; return; }
    dst.jointType = src.jointType;
    dst.matchAnchors = src.matchAnchors;
    dst.anchorPosition = src.anchorPosition;
    dst.anchorRotation = src.anchorRotation;
    dst.parentAnchorPosition = src.parentAnchorPosition;
    dst.parentAnchorRotation = src.parentAnchorRotation;
    dst.linearLockX = src.linearLockX;
    dst.linearLockY = src.linearLockY;
    dst.linearLockZ = src.linearLockZ;
    dst.swingYLock = src.swingYLock;
    dst.swingZLock = src.swingZLock;
    dst.twistLock = src.twistLock;
    var d = src.xDrive; d.target = 0f; d.targetVelocity = 0f;
    dst.xDrive = d;
};

var abRoot = goRoot.AddComponent<ArticulationBody>();
clone(srcRootAb, abRoot, true);
var abB = goB.AddComponent<ArticulationBody>();
clone(srcB, abB, false);
var abC = goC.AddComponent<ArticulationBody>();
clone(srcC, abC, false);

// --- 샘플링 러너 (EditorApplication.update, 자기 해제) ---
string outDir = @"C:\Users\dltmd\AppData\Local\Temp\claude\C--Users-dltmd-Desktop-KDT\4c2a4861-2a65-43c6-921b-9d9d0cd79dd1\scratchpad";
string csvPath = Path.Combine(outDir, "clone_props_log.csv");
string sumPath = Path.Combine(outDir, "clone_props_summary.txt");
var rows = new List<string>();
rows.Add("t,B_act,C_act,B_tgt,C_tgt,B_vel,C_vel");
float tgtB = 45.8f, tgtC = 76.4f;   // 주먹 프로브와 동일 목표(도)
double t0 = EditorApplication.timeSinceStartup;
int phase = 0;
EditorApplication.CallbackFunction cb = null;
cb = () => {
    try {
        if (!EditorApplication.isPlaying || abB == null || abC == null) {
            File.WriteAllLines(csvPath, rows);
            File.WriteAllText(sumPath, "ABORTED: play stopped at " + rows.Count + " rows");
            EditorApplication.update -= cb; return;
        }
        float t = (float)(EditorApplication.timeSinceStartup - t0);
        rows.Add(string.Format("{0:F3},{1:F3},{2:F3},{3:F1},{4:F1},{5:F4},{6:F4}",
            t, abB.jointPosition[0] * Mathf.Rad2Deg, abC.jointPosition[0] * Mathf.Rad2Deg,
            abB.xDrive.target, abC.xDrive.target,
            abB.jointVelocity[0], abC.jointVelocity[0]));
        if (phase == 0 && t > 1.5f) {
            var d = abB.xDrive; d.target = tgtB; abB.xDrive = d;
            d = abC.xDrive; d.target = tgtC; abC.xDrive = d;
            phase = 1;
        }
        if (phase == 1 && t > 9.5f) {
            File.WriteAllLines(csvPath, rows);
            // 정상상태(마지막 4초) 지표
            var ss = rows.Skip(1).Select(r => r.Split(',')).Where(c => float.Parse(c[0]) > 5.5f)
                         .Select(c => new { b = float.Parse(c[1]), cc = float.Parse(c[2]) }).ToList();
            float bp2p = ss.Max(x => x.b) - ss.Min(x => x.b);
            float cp2p = ss.Max(x => x.cc) - ss.Min(x => x.cc);
            float berr = ss.Average(x => Mathf.Abs(x.b - tgtB));
            float cerr = ss.Average(x => Mathf.Abs(x.cc - tgtC));
            File.WriteAllText(sumPath, string.Format(
                "DONE rows={0}\nB(l-clone): steady p2p={1:F2} deg, mean|err|={2:F2} deg, final={3:F2}\n" +
                "C(p-clone): steady p2p={4:F2} deg, mean|err|={5:F2} deg, final={6:F2}\n" +
                "targets: B={7}, C={8}",
                rows.Count - 1, bp2p, berr, ss.Last().b, cp2p, cerr, ss.Last().cc, tgtB, tgtC));
            var g = GameObject.Find("TESTG_root");
            if (g != null) UnityEngine.Object.Destroy(g);
            EditorApplication.update -= cb;
        }
    } catch (Exception e) {
        try { File.WriteAllText(sumPath, "EXCEPTION: " + e); } catch {}
        EditorApplication.update -= cb;
    }
};
EditorApplication.update += cb;

return string.Format(
    "SETUP OK. B clone: mass={0}, I={1}, lockX={2}, anchRot={3}, CoM={4}/{5}, drive {6}/{7}/{8}, solver {9}/{10} | C clone: mass={11}, I={12}",
    abB.mass, abB.inertiaTensor.ToString("E2"), abB.linearLockX, abB.anchorRotation.ToString("F5"),
    abB.automaticCenterOfMass, abB.centerOfMass.ToString("F3"),
    abB.xDrive.stiffness, abB.xDrive.damping, abB.xDrive.forceLimit,
    abB.solverIterations, abB.solverVelocityIterations,
    abC.mass, abC.inertiaTensor.ToString("E2"));
