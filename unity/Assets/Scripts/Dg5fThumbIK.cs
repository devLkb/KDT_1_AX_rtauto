// Dg5fThumbIK.cs
// 엄지 손끝 위치 리타게팅: v2 패킷의 "사람 엄지끝 위치(손바닥 해부학 좌표, 정규화)"를
// 로봇 치수로 복원해 엄지 4관절(1_1~1_4)을 순차 CCD로 그 위치에 보낸다.
//
// 왜: 채널별 선형 매핑은 엄지의 결합 운동(대향×굽힘×접기)을 재현 못함 —
//     OK 사인 끝 맞닿기, 손바닥 안으로 접기 같은 건 손끝 "위치"가 목표여야 한다.
// 핀치: 사람 엄지-검지 끝거리비(v3)에 따라 해부학 목표↔로봇 검지 끝을 연속 블렌딩 —
//     비율 차이와 무관하게 접촉을 보장하면서 임계 근처 목표 점프 없음. v2는 이진 스냅 폴백.
// 자세 prior: 패킷의 엄지 관절각 4채널(IK 활성 시 유휴)을 2순위 목표로 —
//     여유자유도(4DOF vs 위치 3D)가 사람과 다른 자세로 배회하는 것 방지.
//
// 해부학 좌표계(Python dg5f_angles.compute_thumb_tip과 계약, 2026-07-15 §26 방향별 도달 재정의):
//   축: ez=손목→중지MCP, ey=새끼MCP→검지MCP, ex=cross(ey,ez). 좌/우 모델 공통.
//   값: 리치벡터 = 방향(단위벡터) × '펴짐 비율'(0~1). 펴짐 비율 = 사람 엄지 직진도
//       (|끝−CMC| 직선 ÷ 같은 프레임 마디합, Python이 1.0 클램프) → 여기서
//       가상 앵커 + 방향 × 비율 × "그 방향 로봇 최대 도달거리"(Start에서 관절리밋
//       FK 스윕으로 구운 방향별 테이블)로 복원.
//   왜(§26, 2026-07-15): §25의 단일 스칼라(robotThumbMaxReach=0.124) 복원은
//       ① 1_2를 CCD로 반환(§25-2-3)한 뒤 재측정 안 된 구정책 실측값이고
//       ② 방향별 최대 도달이 체인의 77~100%로 변하는 걸 표현 못해 — 사람이 쭉 펴도
//       목표가 로봇 완전 폄보다 안쪽(라이브 실측: |n|=1.0에서 1_3≈−40°로 굽힌 채 도달).
//       방향별 테이블이면 사람 100% 폄 = 그 방향 작업공간 경계로 정의상 정렬.
//   로봇 쪽 대응점: 손목=palm 링크 원점, 중지MCP=3_2, 검지MCP=2_2, 새끼MCP=5_3.
//
// 순차 CCD는 ArmTargetIK(§18)와 동일 패턴: 관절마다 예상 손끝을 회전 갱신,
// 스텝 제한(오차 비례 연속 감쇠) + 리밋 클램프. 활성 시 Dg5fHandDriver가 엄지 채널 주입을 건너뜀.

using UnityEngine;

[RequireComponent(typeof(Dg5fReceiver))]
public class Dg5fThumbIK : MonoBehaviour
{
    [Tooltip("끄면 엄지도 관절각 채널(레거시)로 구동")]
    public bool enableIK = true;

    [Tooltip("관절당 스텝 제한(도/FixedUpdate). §25-4 리플레이 A/B(2026-07-15): 1.5×2iter(150°/s)는 비전 노이즈를 ×21 증폭(추격·오버슈트) — 0.8×1iter(40°/s)로 떨림 1/10 + 오차 절반(1.50→0.70cm). 엄지가 굼뜨면 1.0~1.2로 (iterations는 1 유지)")]
    public float maxStepDeg = 0.8f;

    [Tooltip("CCD 반복 횟수/FixedUpdate — §25-4: 1 초과는 노이즈 증폭에 가담(실효 슬루가 배수로 늘어남)")]
    public int iterations = 1;

    [Tooltip("핀치 스냅 시 검지 끝에서 이만큼 앞(접촉면)에 목표")]
    public float pinchOffset = 0.012f;

    [Tooltip("reachOffset의 거리 단위(m) + 방향별 도달 테이블 빌드 실패 시 폴백 반경. §26(2026-07-15)부터 목표 복원 스케일은 이 스칼라가 아니라 Start에서 FK 스윕으로 굽는 방향별 최대도달 테이블이 담당 — 이 값을 바꾸면 reachOffset의 실거리가 같이 변하므로(ICP 피팅이 이 단위로 저장됨) 0.124 유지")]
    public float robotThumbMaxReach = 0.124f;

    [Tooltip("가상 앵커 오프셋(해부학 축 정규화 단위, ×robotThumbMaxReach가 실거리): 사람 CMC와 로봇 1_1 피벗의 손바닥 대비 위치가 달라 명령 구름 전체가 작업공간에서 벗어나는 것을 평행이동으로 정합. DG5F 엄지 팁은 손바닥 법선(x)으로 최소 ~3cm 떠 있는데 사람 명령의 82%가 x<0.2라 오프셋 없이는 상시 도달 불가(2026-07-15 라이브 7,564명령 × URDF 작업공간 83k점 ICP 피팅 실측: 적용 시 작업공간 밖 89%→10%, 잔차 0.27cm). 왼손 모델 기준 측정 — 해부학 프레임이 좌우 대칭 정의라 우손도 동일 기대, 우손 사용 시 재검증 권장")]
    public Vector3 reachOffset = new Vector3(0.174f, -0.038f, -0.092f);

    // ---- 출렁임 방지 (§18/§18-2 팔 IK에서 확립한 처방) ----
    [Tooltip("목표 위치 스무딩 속도(/s) — 비전 노이즈·핀치 전환 점프 완화. 라이브 노이즈 추적 완화로 10→6 (2026-07-14)")]
    public float targetLerp = 6f;

    [Tooltip("연속 감쇠 소프트존(m): 오차가 이 이하로 줄수록 CCD 스텝을 비례 축소 — 경계 없는 정지(동결 대체)")]
    public float softZone = 0.015f;

    [Tooltip("와인드업 가드(도): 드라이브 목표가 실제 관절각보다 이 이상 못 앞서게")]
    public float windupDeg = 15f;

    // ---- 자세 prior: 패킷 인덱스 0..3(사람→DG5F 매핑 관절각)을 2순위 목표로 ----
    // 여유자유도(4관절 vs 위치 3D)가 배회하지 않게 사람 자세 쪽으로 약하게 당김.
    // CCD의 위치 보정이 항상 우선 — prior는 위치 오차가 작은 구간에서만 사실상 작동.
    [Tooltip("사람 관절각 prior 수렴 속도(/s). 0이면 prior 비활성")]
    public float priorGain = 3f;

    [Tooltip("관절별 prior 가중(1_1~1_4). 2단계(2026-07-15, §25): 1_2도 1 — CCD가 위치를 주도하고 elevation은 여기(약한 prior, 틱당 priorStepRatio×stepCap 캡)로만 들어옴. ⚠️구정책(ff 전담, cmcFeedforward>0)으로 되돌릴 땐 이중 제어 방지 위해 1_2를 0으로")]
    public float[] priorWeights = { 1f, 1f, 1f, 1f };

    [Tooltip("prior 틱당 이동 상한 = 이 비율 × CCD 스텝 상한. 1 이상이면 prior가 CCD를 이겨 리밋사이클(실측 p2p 17~23°)")]
    public float priorStepRatio = 0.3f;

    [Tooltip("prior/ff가 쓰는 사람 관절각의 시간 스무딩(/s). §25-4(2026-07-15): 각도 채널 노이즈가 prior를 통해 매 틱 관절을 당기고 CCD가 되당기는 줄다리기 → 정지 시 로봇 고주파가 목표의 3~5배로 증폭(실측 15~22cm/s vs 목표 5). prior는 자세 힌트라 대역폭 불필요 — 강하게 스무딩. 0이면 원시값")]
    public float priorAngleLerp = 4f;

    readonly float[] _priorSmoothed = new float[4];
    bool _priorSmoothedInit;

    // ---- 앞뒤(깊이) 피드포워드 (2단계에서 비활성, §25 2026-07-15) ----
    // 구정책(§20-5/20-7): 1_2를 elevation ff 전담, CCD 제외(권한=_pinchW). 당시 근거였던
    // "위치만으로 깊이 재현 불가"는 가상 앵커 정합(reachOffset)으로 소멸 — 오히려 ff 전담이
    // 작업공간을 x≥0.24로 묶어 명령 82%가 도달 불가(잔차 1.4~2.5cm+정지 떨림 연료)였음.
    // 2단계 = cmcFeedforward 0(CCD가 1_2 포함 4관절 전담) + elevation은 priorWeights[1]로만.
    // ⚠️원복 = cmcFeedforward 8 + priorWeights[1] 0 (이중 제어 금지 규칙 §20-4).
    [Tooltip("피드포워드 수렴 속도(/s). 0=비활성(2단계 기본): CCD가 1_2 포함 전담, elevation은 prior로만. >0=구정책(1_2를 ff 전담, CCD 제외 — 반드시 priorWeights[1]=0과 함께)")]
    public float cmcFeedforward = 0f;

    [Tooltip("(구정책용) 피드포워드가 전담할 엄지 관절 인덱스(0=1_1 .. 3=1_4). cmcFeedforward=0이면 미사용")]
    public int feedforwardJoint = 1;

    // ---- 핀치 연속 블렌딩: v3 끝거리비로 해부학 목표↔검지끝 스냅을 연속 전환 ----
    [Tooltip("끝거리비가 이 이하면 스냅 목표 100% (v3 패킷 필요)")]
    public float pinchNear = 0.32f;

    [Tooltip("끝거리비가 이 이상이면 해부학 목표 100%")]
    public float pinchFar = 0.55f;

    // ---- 디버그 시각화: 목표 계층을 눈으로 분리 (노랑=UDP 복원 해부학 목표 / 빨강=핀치+스무딩 후 CCD 목표 / 초록=로봇 엄지끝 실측) ----
    [Tooltip("엄지끝 목표를 씬에 구체 마커로 표시 — 노랑이 손을 잘 따라오면 비전 정상, 노랑은 안정인데 빨강·로봇이 떨면 Unity 쪽 문제")]
    public bool debugShowTip = true;

    [Tooltip("디버그 구체 지름(m)")]
    public float debugSphereSize = 0.012f;

    [Tooltip("엄지 UDP 원본·세 마커 위치·달성 비율을 CSV 기록 — Logs/thumbik_*.csv, 분석: dg5f/analyze_thumbik.py")]
    public bool debugLogCsv = true;

    [Tooltip("CSV N샘플마다 디스크 flush")]
    public int debugLogFlushEvery = 100;

    Transform _dbgRaw, _dbgTarget, _dbgTip;
    System.IO.StreamWriter _logW;
    int _logCount;

    public bool Active { get; private set; }

    Vector3 _smoothedTarget;
    bool _hasSmoothed;
    float _pinchW; // 이번 틱 핀치 블렌딩 가중(0~1) — prior 약화에 사용
    readonly float[] _priorAngles = new float[Dg5fReceiver.ChannelCount];

    // ---- 정체 감지 감쇠: 도달 불가 목표에서 CCD가 풀스텝으로 도는 리밋사이클 차단 ----
    // (2026-07-14 정지 프로브 실측: prior 무관하게 p2p 15~23° 지속 — 원인 = CCD 자체.
    //  최선 오차가 stallTime 동안 1mm도 안 줄면 스텝 권한을 지수 감쇠 → 최선 자세에서
    //  정지, 목표 3mm 이동 시 재가동. ⚠️틱당 개선 판정은 물리 추종 지연 탓에 정상 수렴까지
    //  감쇠시켜 핀치 2.4cm 정체 실측 — 시간창 판정이어야 함)
    [Tooltip("이 시간(초) 안에 최선 오차가 1mm도 안 줄면 정체로 판정해 감쇠 시작")]
    public float stallTime = 0.6f;

    [Tooltip("정체 감쇠 재가동에 필요한 목표 이동 거리(m). ⚠️라이브 비전 노이즈보다 커야 함 — 3mm였을 땐 노이즈가 상시 재가동시켜 감쇠가 전혀 안 걸림(2026-07-14 라이브 로그: 입력 정지 중 명령 p2p 8~10°)")]
    public float rearmDist = 0.008f;

    float _stallScale = 1f;
    float _bestErr = float.PositiveInfinity;
    float _bestErrTime;
    Vector3 _armedTarget;

    Dg5fReceiver _rx;
    ArticulationBody[] _thumb;      // 1_1..1_4
    Transform _thumbTip, _indexTip, _palm;
    Vector3 _thumbBaseL, _exL, _eyL, _ezL; // palm 로컬: 엄지 베이스(1_1 피벗) + 해부학 축 (Start에서 캐시)
    float _robotChainLen;                   // 로봇 엄지 체인 길이 |1_1→1_2→1_3→1_4→tip| (강체 마디합 — 포즈 불변)

    // ---- 방향별 최대도달 테이블 (§26, 2026-07-15) ----
    // Start에서 엄지 4관절 리밋 박스를 FK 스윕(13^4≈2.9만 포즈)해 가상 앵커 기준
    // "방향 → 그 방향 최대 |tip−앵커|"를 구·좌표 bin(방위36×고도18)에 max로 적층.
    // 사람 펴짐 비율 1.0 = 그 방향 작업공간 경계 → 로봇도 완전히 뻗어야 도달.
    // 단일 스칼라(구 robotThumbMaxReach 복원)는 방향별 도달 차(체인의 77~100%)를 못
    // 표현해 어떤 방향에선 목표가 안쪽(로봇이 덜 뻗음)·다른 방향에선 도달권 밖이었음.
    // ⚠️ 앵커(reachOffset) 기준으로 굽므로 Play 중 reachOffset을 바꾸면 테이블이 낡음.
    const int FK_STEPS = 13;   // 관절당 샘플 수 — 13^4=28,561 포즈, Start에서 수십 ms
    const int AZ_BINS = 36;    // 방위각(ey-ez 평면) 10°/bin
    const int EL_BINS = 18;    // 고도각(손바닥 법선 ex 성분) 10°/bin
    float[,] _reachTable;      // [az, el] = 방향별 최대 도달거리(m), 빈 bin은 이웃 max로 충전
    Vector3 _virtualBaseL;     // palm 로컬 가상 앵커 = 1_1 피벗 + reachOffset×robotThumbMaxReach

    void Start()
    {
        _rx = GetComponent<Dg5fReceiver>();
        _thumb = new ArticulationBody[4];
        Transform midMcp = null, idxMcp = null, pinkyMcp = null;
        foreach (var t in GetComponentsInChildren<Transform>())
        {
            string n = t.name;
            if (n.EndsWith("_dg_palm")) _palm = t;
            else if (n.EndsWith("_dg_1_tip")) _thumbTip = t;
            else if (n.EndsWith("_dg_2_tip")) _indexTip = t;
            else if (n.EndsWith("_dg_3_2")) midMcp = t;
            else if (n.EndsWith("_dg_2_2")) idxMcp = t;
            else if (n.EndsWith("_dg_5_3")) pinkyMcp = t;
        }
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
        {
            if (ab.jointType != ArticulationJointType.RevoluteJoint) continue;
            int k = ab.name.IndexOf("_dg_");
            if (k < 0) continue; // 결합 로봇의 팔 관절 제외
            string s = ab.name.Substring(k + 4);
            if (s[0] == '1') _thumb[s[2] - '1'] = ab;
        }
        if (_palm == null || midMcp == null || idxMcp == null || pinkyMcp == null || _thumbTip == null
            || _thumb[0] == null || _thumb[1] == null || _thumb[2] == null || _thumb[3] == null)
        {
            Debug.LogError("[Dg5fThumbIK] 기준 링크를 못 찾음 — 비활성화");
            enableIK = false;
            return;
        }

        // 로봇 해부학 좌표계 (0° 포즈 기준, palm 로컬로 캐시 — palm은 강체라 불변)
        Vector3 wrist = _palm.position;
        Vector3 ez = (midMcp.position - wrist).normalized;
        Vector3 ex = Vector3.Cross(idxMcp.position - pinkyMcp.position, ez).normalized;
        Vector3 ey = Vector3.Cross(ez, ex);
        _exL = _palm.InverseTransformDirection(ex);
        _eyL = _palm.InverseTransformDirection(ey);
        _ezL = _palm.InverseTransformDirection(ez);

        // 엄지 베이스(1_1 피벗)와 체인 길이 — 연속 관절 원점 간 거리 합이라 현재 포즈와 무관
        _thumbBaseL = _palm.InverseTransformPoint(_thumb[0].transform.position);
        _robotChainLen = 0f;
        Vector3 prev = _thumb[0].transform.position;
        for (int i = 1; i < 4; i++)
        {
            _robotChainLen += (_thumb[i].transform.position - prev).magnitude;
            prev = _thumb[i].transform.position;
        }
        _robotChainLen += (_thumbTip.position - prev).magnitude;

        _virtualBaseL = _thumbBaseL
            + (_exL * reachOffset.x + _eyL * reachOffset.y + _ezL * reachOffset.z) * robotThumbMaxReach;
        BuildReachTable();
    }

    // ---- 방향별 최대도달 테이블 빌드: rest 포즈(관절각 0) 기하를 palm 로컬로 캐시한 뒤
    // 관절 리밋 박스를 FK 스윕. 회전은 원위(1_4)→근위(1_1) 순으로 적용 — 근위가 아직
    // 안 돌았으므로 각 단계에서 rest 프레임의 피벗·축을 그대로 쓸 수 있다(§18 CCD와 동일 원리).
    void BuildReachTable()
    {
        var pivL = new Vector3[4];
        var axL = new Vector3[4];
        var lo = new float[4];
        var hi = new float[4];
        for (int i = 0; i < 4; i++)
        {
            pivL[i] = _palm.InverseTransformPoint(_thumb[i].transform.position);
            axL[i] = _palm.InverseTransformDirection(
                _thumb[i].transform.rotation * _thumb[i].anchorRotation * Vector3.right);
            lo[i] = _thumb[i].xDrive.lowerLimit;
            hi[i] = _thumb[i].xDrive.upperLimit;
        }
        Vector3 tipL = _palm.InverseTransformPoint(_thumbTip.position);

        var rot = new Quaternion[4, FK_STEPS];
        for (int i = 0; i < 4; i++)
            for (int s = 0; s < FK_STEPS; s++)
                rot[i, s] = Quaternion.AngleAxis(
                    Mathf.Lerp(lo[i], hi[i], s / (float)(FK_STEPS - 1)), axL[i]);

        _reachTable = new float[AZ_BINS, EL_BINS];
        for (int s0 = 0; s0 < FK_STEPS; s0++)
        {
            for (int s1 = 0; s1 < FK_STEPS; s1++)
            {
                for (int s2 = 0; s2 < FK_STEPS; s2++)
                {
                    // 1_4(원위)→1_2까지는 조합 공통 접두라 바깥 루프에서 재사용 불가한
                    // 구조 대신 단순 4중 루프 — 2.9만 회 AngleAxis 곱은 Start 1회 비용으로 무시 가능
                    for (int s3 = 0; s3 < FK_STEPS; s3++)
                    {
                        Vector3 p = pivL[3] + rot[3, s3] * (tipL - pivL[3]);
                        p = pivL[2] + rot[2, s2] * (p - pivL[2]);
                        p = pivL[1] + rot[1, s1] * (p - pivL[1]);
                        p = pivL[0] + rot[0, s0] * (p - pivL[0]);
                        Vector3 rel = p - _virtualBaseL;
                        float r = rel.magnitude;
                        if (r < 1e-6f) continue;
                        Vector3 d = rel / r;
                        DirToBin(Vector3.Dot(d, _exL), Vector3.Dot(d, _eyL), Vector3.Dot(d, _ezL),
                                 out int ia, out int ie);
                        if (r > _reachTable[ia, ie]) _reachTable[ia, ie] = r;
                    }
                }
            }
        }

        // 빈 bin 충전(이웃 max 확산): 로봇이 못 가리키는 방향도 노이즈로 조회될 수 있어
        // 가장 가까운 도달값을 준다 — 0이면 목표가 앵커로 붕괴해 엄지가 오므라듦.
        bool holes = true;
        for (int pass = 0; holes && pass < AZ_BINS + EL_BINS; pass++)
        {
            holes = false;
            var src = (float[,])_reachTable.Clone();
            for (int a = 0; a < AZ_BINS; a++)
            {
                for (int e = 0; e < EL_BINS; e++)
                {
                    if (src[a, e] > 0f) continue;
                    float best = 0f;
                    for (int da = -1; da <= 1; da++)
                    {
                        for (int de = -1; de <= 1; de++)
                        {
                            int na = (a + da + AZ_BINS) % AZ_BINS;             // 방위각 랩
                            int ne = Mathf.Clamp(e + de, 0, EL_BINS - 1);      // 고도각 클램프
                            if (src[na, ne] > best) best = src[na, ne];
                        }
                    }
                    if (best > 0f) _reachTable[a, e] = best;
                    else holes = true;
                }
            }
        }

        float rMin = float.PositiveInfinity, rMax = 0f;
        foreach (float v in _reachTable) { if (v < rMin) rMin = v; if (v > rMax) rMax = v; }
        Debug.Log($"[Dg5fThumbIK] 준비 완료 — 마디합 {_robotChainLen * 100:F1}cm, "
                  + $"방향별 최대도달 테이블 {rMin * 100:F1}~{rMax * 100:F1}cm "
                  + $"(FK {FK_STEPS}^4 스윕, 앵커=1_1+offset)");
    }

    static void DirToBin(float dx, float dy, float dz, out int ia, out int ie)
    {
        float az = Mathf.Atan2(dy, dz);                          // ey-ez 평면 방위각
        float el = Mathf.Asin(Mathf.Clamp(dx, -1f, 1f));         // 손바닥 법선(ex) 고도각
        ia = Mathf.Clamp((int)((az + Mathf.PI) / (2f * Mathf.PI) * AZ_BINS), 0, AZ_BINS - 1);
        ie = Mathf.Clamp((int)((el + Mathf.PI * 0.5f) / Mathf.PI * EL_BINS), 0, EL_BINS - 1);
    }

    // 방향(해부학 축 성분, 단위벡터) → 최대 도달거리. bin 경계 계단을 없애려 방위×고도 쌍선형 보간.
    float ReachIn(Vector3 dirAnat)
    {
        if (_reachTable == null) return robotThumbMaxReach;
        float az = Mathf.Atan2(dirAnat.y, dirAnat.z);
        float el = Mathf.Asin(Mathf.Clamp(dirAnat.x, -1f, 1f));
        float fa = (az + Mathf.PI) / (2f * Mathf.PI) * AZ_BINS - 0.5f;
        float fe = (el + Mathf.PI * 0.5f) / Mathf.PI * EL_BINS - 0.5f;
        int a0 = Mathf.FloorToInt(fa), e0 = Mathf.FloorToInt(fe);
        float ta = fa - a0, te = fe - e0;
        int a1 = (a0 + 1 + AZ_BINS * 2) % AZ_BINS;
        a0 = (a0 + AZ_BINS * 2) % AZ_BINS;
        int e1 = Mathf.Clamp(e0 + 1, 0, EL_BINS - 1);
        e0 = Mathf.Clamp(e0, 0, EL_BINS - 1);
        float r0 = Mathf.Lerp(_reachTable[a0, e0], _reachTable[a1, e0], ta);
        float r1 = Mathf.Lerp(_reachTable[a0, e1], _reachTable[a1, e1], ta);
        return Mathf.Lerp(r0, r1, te);
    }

    void FixedUpdate()
    {
        Active = false;
        if (!enableIK || _rx == null || _thumb[0] == null) { SetDebugMarkersVisible(false); return; }
        if (!_rx.GetThumbTip(out Vector3 tipN, out bool pinch)) { SetDebugMarkersVisible(false); return; }
        if (_rx.secondsSinceLastPacket > 1.0f) { SetDebugMarkersVisible(false); return; }
        Active = true;

        // 리치 복원(§26): 가상 앵커 + 방향 × 펴짐비율 × 그 방향 로봇 최대도달(FK 테이블).
        // Python이 크기 1.0으로 클램프하지만 구버전 송신기/오보정 패킷 대비 여기서도
        // 비율을 1.0으로 재클램프 — 목표가 그 방향 작업공간 경계를 절대 못 넘게.
        Vector3 exW = _palm.TransformDirection(_exL);
        Vector3 eyW = _palm.TransformDirection(_eyL);
        Vector3 ezW = _palm.TransformDirection(_ezL);
        // 가상 앵커: 1_1 피벗 + reachOffset — 사람/로봇 엄지 작업공간 정합용 평행이동(1단계, §25)
        Vector3 virtualBase = _palm.TransformPoint(_virtualBaseL);
        Vector3 anatomical = virtualBase;
        float ext = tipN.magnitude;                    // 사람 엄지 펴짐 비율(직진도)
        if (ext > 1e-5f)
        {
            Vector3 dirA = tipN / ext;                 // 해부학 축 성분 단위 방향
            if (ext > 1f) ext = 1f;
            Vector3 dirW = exW * dirA.x + eyW * dirA.y + ezW * dirA.z;
            anatomical = virtualBase + dirW * (ext * ReachIn(dirA));
        }
        Vector3 raw = anatomical;
        _pinchW = 0f;
        float pinchD = float.NaN; // v2 폴백이면 NaN 유지 (CSV 기록용)
        if (_indexTip != null)
        {
            // 스냅 목표(로봇 검지 끝 + 접촉 오프셋)와 해부학 목표를 끝거리비로 연속 블렌딩.
            // v3 미수신(구버전 vision_node)이면 기존 이진 핀치 플래그로 폴백 — 목표 점프 감수.
            Vector3 snap = _indexTip.position
                           + (_thumbTip.position - _indexTip.position).normalized * pinchOffset;
            bool hasPinchD = _rx.GetPinchDistance(out pinchD);
            if (!hasPinchD) pinchD = float.NaN;
            _pinchW = hasPinchD
                ? 1f - Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(pinchNear, pinchFar, pinchD))
                : (pinch ? 1f : 0f);
            raw = Vector3.Lerp(anatomical, snap, _pinchW);
        }

        // 목표 스무딩 — 비전 노이즈·핀치 전환 점프가 CCD에 직결되지 않게
        if (!_hasSmoothed) { _smoothedTarget = raw; _hasSmoothed = true; }
        float kT = 1f - Mathf.Exp(-targetLerp * Time.fixedDeltaTime);
        _smoothedTarget = Vector3.Lerp(_smoothedTarget, raw, kT);
        Vector3 target = _smoothedTarget;

        UpdateDebugMarkers(anatomical, target);

        // 연속 감쇠(동결 대체): 오차가 softZone 아래로 줄수록 스텝 상한을 비례 축소.
        // 동결↔재가동 경계가 없어 뚝뚝 끊기지 않고, prior와의 평형점에서 매끄럽게 정지.
        float err = Vector3.Distance(_thumbTip.position, target);
        float stepCap = maxStepDeg * Mathf.Clamp01(err / Mathf.Max(softZone, 1e-4f));

        // 정체 감지: 목표가 rearmDist 이상 움직이면 재가동, stallTime 내 1mm 개선 없으면 권한 감쇠.
        if ((target - _armedTarget).magnitude > rearmDist)
        {
            _armedTarget = target;
            _bestErr = float.PositiveInfinity;
            _bestErrTime = Time.fixedTime;
            _stallScale = 1f;
        }
        if (err < _bestErr - 0.001f)
        {
            _bestErr = err;
            _bestErrTime = Time.fixedTime;
            // 점진 회복(즉시 1.0 리셋 금지) — 리셋하면 감쇠된 리밋사이클이 재점화(실측 15~19° 재발)
            _stallScale = Mathf.Min(1f, _stallScale * 1.15f);
        }
        else if (Time.fixedTime - _bestErrTime > stallTime)
            _stallScale = Mathf.Max(0.02f, _stallScale * 0.95f);
        stepCap *= _stallScale;

        // 순차 CCD (base→tip): 예상 손끝을 회전 갱신하며 잔여 오차만 보정
        for (int it = 0; it < iterations; it++)
        {
            Vector3 tip = _thumbTip.position;
            for (int i = 0; i < 4; i++)
            {
                var ab = _thumb[i];
                Vector3 pivot = ab.transform.position;
                Vector3 axis = ab.transform.rotation * ab.anchorRotation * Vector3.right;
                Vector3 toTip = Vector3.ProjectOnPlane(tip - pivot, axis);
                Vector3 toTgt = Vector3.ProjectOnPlane(target - pivot, axis);
                if (toTip.sqrMagnitude < 1e-8f || toTgt.sqrMagnitude < 1e-8f) continue;
                float ang = Vector3.SignedAngle(toTip, toTgt, axis);
                if (i == feedforwardJoint && cmcFeedforward > 0f) ang *= _pinchW; // 평시 ff 관절은 CCD 제외
                ang = Mathf.Clamp(ang, -stepCap, stepCap);
                var d = ab.xDrive;
                // 와인드업 가드: 실제 관절각에서 ±windupDeg 이상 못 앞서게 (§15 교훈)
                float act = ab.jointPosition[0] * Mathf.Rad2Deg;
                float newTarget = Mathf.Clamp(d.target + ang, act - windupDeg, act + windupDeg);
                newTarget = Mathf.Clamp(newTarget, d.lowerLimit, d.upperLimit);
                float applied = newTarget - d.target;
                d.target = newTarget;
                ab.xDrive = d;
                tip = pivot + Quaternion.AngleAxis(applied, axis) * (tip - pivot);
            }
        }

        bool hasAngles = _rx.GetAngles(_priorAngles);

        // prior 각도 스무딩 — 원시 채널 노이즈가 틱 단위로 관절을 흔들지 않게 (§25-4)
        if (hasAngles)
        {
            if (!_priorSmoothedInit)
            {
                for (int i = 0; i < 4; i++) _priorSmoothed[i] = _priorAngles[i];
                _priorSmoothedInit = true;
            }
            else if (priorAngleLerp > 0f)
            {
                float kA = 1f - Mathf.Exp(-priorAngleLerp * Time.fixedDeltaTime);
                for (int i = 0; i < 4; i++)
                    _priorSmoothed[i] = Mathf.Lerp(_priorSmoothed[i], _priorAngles[i], kA);
            }
            else
            {
                for (int i = 0; i < 4; i++) _priorSmoothed[i] = _priorAngles[i];
            }
        }

        // 자세 prior: 여유자유도를 사람 관절각(패킷 0..3) 쪽으로 약하게 당김.
        // CCD 뒤에 적용 — 위치 오차가 커지면 다음 틱 CCD가 되돌리므로 위치 우선이 유지된다.
        // 핀치 블렌딩이 걸릴수록 (1-_pinchW)로 약화 — 접촉이 자세보다 우선.
        // ⚠️ 틱당 이동을 priorStepRatio×stepCap으로 캡 — prior가 CCD보다 세면 서로 싸워
        //    리밋사이클 발생(2026-07-14 정지 프로브 실측 p2p 17~23°). 오차↓ → 둘 다 함께 소멸.
        if (priorGain > 0f && _pinchW < 1f && hasAngles)
        {
            float kP = (1f - Mathf.Exp(-priorGain * Time.fixedDeltaTime)) * (1f - _pinchW);
            float priorCap = priorStepRatio * stepCap;
            for (int i = 0; i < 4; i++)
            {
                float w = (priorWeights != null && i < priorWeights.Length) ? priorWeights[i] : 0f;
                var ab = _thumb[i];
                if (w <= 0f || ab == null) continue;
                var d = ab.xDrive;
                float prior = Mathf.Clamp(_priorSmoothed[i], d.lowerLimit, d.upperLimit);
                float delta = Mathf.Clamp((prior - d.target) * kP * Mathf.Clamp01(w),
                                          -priorCap, priorCap);
                float act = ab.jointPosition[0] * Mathf.Rad2Deg;
                float nt = Mathf.Clamp(d.target + delta, act - windupDeg, act + windupDeg);
                d.target = Mathf.Clamp(nt, d.lowerLimit, d.upperLimit);
                ab.xDrive = d;
            }
        }

        // 앞뒤 피드포워드: elevation 채널 → feedforwardJoint(기본 1_2) 직결. 핀치 가중만큼 CCD에 역할 이양.
        if (cmcFeedforward > 0f && hasAngles && feedforwardJoint >= 0 && feedforwardJoint < 4
            && _thumb[feedforwardJoint] != null)
        {
            var ab = _thumb[feedforwardJoint];
            var d = ab.xDrive;
            float kF = (1f - Mathf.Exp(-cmcFeedforward * Time.fixedDeltaTime)) * (1f - _pinchW);
            float goal = Mathf.Clamp(_priorSmoothed[feedforwardJoint], d.lowerLimit, d.upperLimit);
            float nt = Mathf.Lerp(d.target, goal, kF);
            float act = ab.jointPosition[0] * Mathf.Rad2Deg;
            nt = Mathf.Clamp(nt, act - windupDeg, act + windupDeg);
            d.target = Mathf.Clamp(nt, d.lowerLimit, d.upperLimit);
            ab.xDrive = d;
        }

        if (debugLogCsv) LogDebugCsv(tipN, pinchD, anatomical, target, err, stepCap);
    }

    // ---- 디버그 CSV: UDP 원본 비율 vs 로봇 달성 비율 + 세 마커 위치 ----
    // ach_* = (로봇 엄지끝 − 엄지 베이스)를 해부학 축으로 분해 ÷ 로봇 체인 길이
    //       — UDP의 n_*과 같은 좌표계·같은 스케일이라 축별 직접 비교 가능.
    //       corr(n,ach)<0이면 축 부호 반전, 상시 편차면 프레임/오프셋, err_red 크고
    //       stall_scale 바닥이면 도달 불가 목표에서 damper가 정지시킨 것.
    void LogDebugCsv(Vector3 tipN, float pinchD, Vector3 yellow, Vector3 red, float err, float stepCap)
    {
        if (_logW == null)
        {
            string dir = System.IO.Path.Combine(Application.dataPath, "../Logs");
            System.IO.Directory.CreateDirectory(dir);
            string path = System.IO.Path.Combine(dir,
                "thumbik_" + System.DateTime.Now.ToString("yyyyMMdd_HHmm") + ".csv");
            _logW = new System.IO.StreamWriter(path, false, System.Text.Encoding.UTF8);
            _logW.WriteLine("t_unix,n_x,n_y,n_z,pinch_d,pinch_w,"
                + "yel_x,yel_y,yel_z,red_x,red_y,red_z,grn_x,grn_y,grn_z,"
                + "ach_x,ach_y,ach_z,err_red,err_yel,stall_scale,step_cap,"
                + "tgt_1_1,tgt_1_2,tgt_1_3,tgt_1_4,act_1_1,act_1_2,act_1_3,act_1_4");
            Debug.Log("[Dg5fThumbIK] 디버그 CSV 기록 시작: " + path);
        }
        Vector3 grn = _thumbTip.position;
        // ach도 '펴짐 비율' 단위(§26: 1.0 = 그 방향 최대도달), 기준점 = 가상 앵커(1_1+reachOffset)
        // — 패킷 n_*과 같은 원점·같은 스케일이라 축별 직접 비교(corr/bias) 유지
        Vector3 exW2 = _palm.TransformDirection(_exL);
        Vector3 eyW2 = _palm.TransformDirection(_eyL);
        Vector3 ezW2 = _palm.TransformDirection(_ezL);
        Vector3 relW = grn - _palm.TransformPoint(_virtualBaseL);
        Vector3 relA = new Vector3(
            Vector3.Dot(relW, exW2),
            Vector3.Dot(relW, eyW2),
            Vector3.Dot(relW, ezW2));
        float relR = relA.magnitude;
        Vector3 ach = relR > 1e-6f ? relA / ReachIn(relA / relR) : Vector3.zero;

        var ci = System.Globalization.CultureInfo.InvariantCulture;
        double t = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
        var sb = new System.Text.StringBuilder(t.ToString("F3", ci));
        System.Action<float> A = v => sb.Append(',').Append(v.ToString("F4", ci));
        A(tipN.x); A(tipN.y); A(tipN.z); A(pinchD); A(_pinchW);
        A(yellow.x); A(yellow.y); A(yellow.z);
        A(red.x); A(red.y); A(red.z);
        A(grn.x); A(grn.y); A(grn.z);
        A(ach.x); A(ach.y); A(ach.z);
        A(err); A(Vector3.Distance(yellow, grn)); A(_stallScale); A(stepCap);
        for (int i = 0; i < 4; i++) A(_thumb[i].xDrive.target);
        for (int i = 0; i < 4; i++) A(_thumb[i].jointPosition[0] * Mathf.Rad2Deg);
        _logW.WriteLine(sb.ToString());
        if (++_logCount % Mathf.Max(1, debugLogFlushEvery) == 0) _logW.Flush();
    }

    void OnDestroy()
    {
        if (_logW != null) { _logW.Flush(); _logW.Dispose(); _logW = null; }
    }

    // ---- 디버그 마커 ----
    // 씬에 저장되지 않는 런타임 전용 오브젝트 (Play 종료 시 자동 소멸). 콜라이더 없음(물리 간섭 0).
    Transform MakeMarker(string name, Color c, float scale)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.name = name;
        Destroy(go.GetComponent<Collider>());
        var sh = Shader.Find("Unlit/Color");
        var mat = new Material(sh != null ? sh : Shader.Find("Standard"));
        mat.color = c;
        go.GetComponent<Renderer>().material = mat;
        go.transform.localScale = Vector3.one * scale;
        return go.transform;
    }

    void SetDebugMarkersVisible(bool visible)
    {
        if (_dbgRaw == null) return;
        _dbgRaw.gameObject.SetActive(visible);
        _dbgTarget.gameObject.SetActive(visible);
        _dbgTip.gameObject.SetActive(visible);
    }

    void UpdateDebugMarkers(Vector3 rawUdp, Vector3 ccdTarget)
    {
        if (!debugShowTip) { SetDebugMarkersVisible(false); return; }
        if (_dbgRaw == null)
        {
            _dbgRaw = MakeMarker("DBG_ThumbTip_UDP (yellow)", Color.yellow, debugSphereSize);
            _dbgTarget = MakeMarker("DBG_ThumbTip_CCDTarget (red)", Color.red, debugSphereSize);
            _dbgTip = MakeMarker("DBG_ThumbTip_Robot (green)", Color.green, debugSphereSize * 0.8f);
        }
        SetDebugMarkersVisible(true);
        _dbgRaw.position = rawUdp;         // UDP 수신 복원 목표 (핀치 블렌딩 전)
        _dbgTarget.position = ccdTarget;   // 핀치 블렌딩 + 스무딩 후 CCD가 실제로 쫓는 목표
        _dbgTip.position = _thumbTip.position; // 로봇 엄지끝 실측
    }
}
