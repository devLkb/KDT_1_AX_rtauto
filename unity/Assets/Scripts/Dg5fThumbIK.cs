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
// 해부학 좌표계(Python dg5f_angles.compute_thumb_tip과 계약):
//   원점=중지 MCP, ez=손목→중지MCP, ey=새끼MCP→검지MCP, ex=cross(ey,ez).
//   로봇 쪽 대응점: 손목=palm 링크 원점, 중지MCP=3_2, 검지MCP=2_2, 새끼MCP=5_3.
//   랜드마크가 해부학 이름 기준이라 좌/우 모델 공통 (미러 표 불필요).
//
// 순차 CCD는 ArmTargetIK(§18)와 동일 패턴: 관절마다 예상 손끝을 회전 갱신,
// 스텝 제한(오차 비례 연속 감쇠) + 리밋 클램프. 활성 시 Dg5fHandDriver가 엄지 채널 주입을 건너뜀.

using UnityEngine;

[RequireComponent(typeof(Dg5fReceiver))]
public class Dg5fThumbIK : MonoBehaviour
{
    [Tooltip("끄면 엄지도 관절각 채널(레거시)로 구동")]
    public bool enableIK = true;

    [Tooltip("관절당 스텝 제한(도/FixedUpdate) — 급출발 방지")]
    public float maxStepDeg = 2.5f;

    [Tooltip("CCD 반복 횟수/FixedUpdate")]
    public int iterations = 3;

    [Tooltip("핀치 스냅 시 검지 끝에서 이만큼 앞(접촉면)에 목표")]
    public float pinchOffset = 0.012f;

    // ---- 출렁임 방지 (§18/§18-2 팔 IK에서 확립한 처방) ----
    [Tooltip("목표 위치 스무딩 속도(/s) — 비전 노이즈·핀치 전환 점프 완화")]
    public float targetLerp = 10f;

    [Tooltip("연속 감쇠 소프트존(m): 오차가 이 이하로 줄수록 CCD 스텝을 비례 축소 — 경계 없는 정지(동결 대체)")]
    public float softZone = 0.015f;

    [Tooltip("와인드업 가드(도): 드라이브 목표가 실제 관절각보다 이 이상 못 앞서게")]
    public float windupDeg = 15f;

    // ---- 자세 prior: 패킷 인덱스 0..3(사람→DG5F 매핑 관절각)을 2순위 목표로 ----
    // 여유자유도(4관절 vs 위치 3D)가 배회하지 않게 사람 자세 쪽으로 약하게 당김.
    // CCD의 위치 보정이 항상 우선 — prior는 위치 오차가 작은 구간에서만 사실상 작동.
    [Tooltip("사람 관절각 prior 수렴 속도(/s). 0이면 prior 비활성")]
    public float priorGain = 3f;

    [Tooltip("관절별 prior 가중(1_1~1_4). 1_1=0 고정: Python이 thumb_cmc를 게이트(항상 0°)라 자세 정보가 아님")]
    public float[] priorWeights = { 0f, 1f, 1f, 1f };

    [Tooltip("prior 틱당 이동 상한 = 이 비율 × CCD 스텝 상한. 1 이상이면 prior가 CCD를 이겨 리밋사이클(실측 p2p 17~23°)")]
    public float priorStepRatio = 0.3f;

    // ---- CMC 피드포워드: 채널0(엄지 앞뒤, 신프록시)을 1_1에 직결 ----
    // 엄지끝 위치만으로는 앞뒤 성분이 깊이(z) 노이즈 축에 실리고 여유자유도에 흡수돼
    // 1_1이 안 움직임(2026-07-14 분석). 평시 CCD는 1_1을 건드리지 않고(권한 = _pinchW)
    // ff가 전담 — 같은 관절을 두 컨트롤러가 다투지 않아 리밋사이클 없음. 핀치 시 역할 교대.
    [Tooltip("1_1(CMC) 채널0 직결 수렴 속도(/s). 0이면 구버전(1_1도 CCD 전담)")]
    public float cmcFeedforward = 8f;

    // ---- 핀치 연속 블렌딩: v3 끝거리비로 해부학 목표↔검지끝 스냅을 연속 전환 ----
    [Tooltip("끝거리비가 이 이하면 스냅 목표 100% (v3 패킷 필요)")]
    public float pinchNear = 0.32f;

    [Tooltip("끝거리비가 이 이상이면 해부학 목표 100%")]
    public float pinchFar = 0.55f;

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

    float _stallScale = 1f;
    float _bestErr = float.PositiveInfinity;
    float _bestErrTime;
    Vector3 _armedTarget;

    Dg5fReceiver _rx;
    ArticulationBody[] _thumb;      // 1_1..1_4
    Transform _thumbTip, _indexTip, _palm;
    Vector3 _originL, _exL, _eyL, _ezL; // palm 로컬 해부학 좌표계 (Start에서 캐시)
    float _handLen;                      // 로봇 손 기준 길이 |palm→중지MCP|

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
        if (_palm == null || midMcp == null || idxMcp == null || pinkyMcp == null || _thumbTip == null)
        {
            Debug.LogError("[Dg5fThumbIK] 기준 링크를 못 찾음 — 비활성화");
            enableIK = false;
            return;
        }

        // 로봇 해부학 좌표계 (0° 포즈 기준, palm 로컬로 캐시 — palm은 강체라 불변)
        Vector3 wrist = _palm.position;
        Vector3 mid = midMcp.position;
        _handLen = (mid - wrist).magnitude;
        Vector3 ez = (mid - wrist).normalized;
        Vector3 ex = Vector3.Cross(idxMcp.position - pinkyMcp.position, ez).normalized;
        Vector3 ey = Vector3.Cross(ez, ex);
        _originL = _palm.InverseTransformPoint(mid);
        _exL = _palm.InverseTransformDirection(ex);
        _eyL = _palm.InverseTransformDirection(ey);
        _ezL = _palm.InverseTransformDirection(ez);
        Debug.Log($"[Dg5fThumbIK] 준비 완료 — 손 기준길이 {_handLen * 100:F1}cm");
    }

    void FixedUpdate()
    {
        Active = false;
        if (!enableIK || _rx == null || _thumb[0] == null) return;
        if (!_rx.GetThumbTip(out Vector3 tipN, out bool pinch)) return;
        if (_rx.secondsSinceLastPacket > 1.0f) return;
        Active = true;

        Vector3 origin = _palm.TransformPoint(_originL);
        Vector3 anatomical = origin + (_palm.TransformDirection(_exL) * tipN.x
                                       + _palm.TransformDirection(_eyL) * tipN.y
                                       + _palm.TransformDirection(_ezL) * tipN.z) * _handLen;
        Vector3 raw = anatomical;
        _pinchW = 0f;
        if (_indexTip != null)
        {
            // 스냅 목표(로봇 검지 끝 + 접촉 오프셋)와 해부학 목표를 끝거리비로 연속 블렌딩.
            // v3 미수신(구버전 vision_node)이면 기존 이진 핀치 플래그로 폴백 — 목표 점프 감수.
            Vector3 snap = _indexTip.position
                           + (_thumbTip.position - _indexTip.position).normalized * pinchOffset;
            _pinchW = _rx.GetPinchDistance(out float pinchD)
                ? 1f - Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(pinchNear, pinchFar, pinchD))
                : (pinch ? 1f : 0f);
            raw = Vector3.Lerp(anatomical, snap, _pinchW);
        }

        // 목표 스무딩 — 비전 노이즈·핀치 전환 점프가 CCD에 직결되지 않게
        if (!_hasSmoothed) { _smoothedTarget = raw; _hasSmoothed = true; }
        float kT = 1f - Mathf.Exp(-targetLerp * Time.fixedDeltaTime);
        _smoothedTarget = Vector3.Lerp(_smoothedTarget, raw, kT);
        Vector3 target = _smoothedTarget;

        // 연속 감쇠(동결 대체): 오차가 softZone 아래로 줄수록 스텝 상한을 비례 축소.
        // 동결↔재가동 경계가 없어 뚝뚝 끊기지 않고, prior와의 평형점에서 매끄럽게 정지.
        float err = Vector3.Distance(_thumbTip.position, target);
        float stepCap = maxStepDeg * Mathf.Clamp01(err / Mathf.Max(softZone, 1e-4f));

        // 정체 감지: 목표가 3mm 이상 움직이면 재가동, stallTime 내 1mm 개선 없으면 권한 감쇠.
        if ((target - _armedTarget).magnitude > 0.003f)
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
                if (i == 0 && cmcFeedforward > 0f) ang *= _pinchW; // 평시 1_1은 ff 전담
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
                float prior = Mathf.Clamp(_priorAngles[i], d.lowerLimit, d.upperLimit);
                float delta = Mathf.Clamp((prior - d.target) * kP * Mathf.Clamp01(w),
                                          -priorCap, priorCap);
                float act = ab.jointPosition[0] * Mathf.Rad2Deg;
                float nt = Mathf.Clamp(d.target + delta, act - windupDeg, act + windupDeg);
                d.target = Mathf.Clamp(nt, d.lowerLimit, d.upperLimit);
                ab.xDrive = d;
            }
        }

        // CMC 피드포워드: 채널0(엄지 앞뒤) → 1_1 직결. 핀치 가중만큼 CCD에 역할 이양.
        if (cmcFeedforward > 0f && hasAngles && _thumb[0] != null)
        {
            var ab = _thumb[0];
            var d = ab.xDrive;
            float kF = (1f - Mathf.Exp(-cmcFeedforward * Time.fixedDeltaTime)) * (1f - _pinchW);
            float goal = Mathf.Clamp(_priorAngles[0], d.lowerLimit, d.upperLimit);
            float nt = Mathf.Lerp(d.target, goal, kF);
            float act = ab.jointPosition[0] * Mathf.Rad2Deg;
            nt = Mathf.Clamp(nt, act - windupDeg, act + windupDeg);
            d.target = Mathf.Clamp(nt, d.lowerLimit, d.upperLimit);
            ab.xDrive = d;
        }
    }
}
