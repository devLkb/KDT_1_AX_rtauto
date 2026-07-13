// Dg5fThumbIK.cs
// 엄지 손끝 위치 리타게팅: v2 패킷의 "사람 엄지끝 위치(손바닥 해부학 좌표, 정규화)"를
// 로봇 치수로 복원해 엄지 4관절(1_1~1_4)을 순차 CCD로 그 위치에 보낸다.
//
// 왜: 채널별 선형 매핑은 엄지의 결합 운동(대향×굽힘×접기)을 재현 못함 —
//     OK 사인 끝 맞닿기, 손바닥 안으로 접기 같은 건 손끝 "위치"가 목표여야 한다.
// 핀치 스냅: 사람이 엄지-검지 끝을 붙이면(핀치 플래그) 목표를 로봇 검지 끝으로
//     직접 전환 — 비율 차이와 무관하게 접촉을 보장.
//
// 해부학 좌표계(Python dg5f_angles.compute_thumb_tip과 계약):
//   원점=중지 MCP, ez=손목→중지MCP, ey=새끼MCP→검지MCP, ex=cross(ey,ez).
//   로봇 쪽 대응점: 손목=palm 링크 원점, 중지MCP=3_2, 검지MCP=2_2, 새끼MCP=5_3.
//   랜드마크가 해부학 이름 기준이라 좌/우 모델 공통 (미러 표 불필요).
//
// 순차 CCD는 ArmTargetIK(§18)와 동일 패턴: 관절마다 예상 손끝을 회전 갱신,
// 스텝 제한 + 리밋 클램프. 활성 시 Dg5fHandDriver가 엄지 채널 주입을 건너뜀.

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

    [Tooltip("도달 데드밴드(m): 오차가 이보다 작으면 동결")]
    public float deadband = 0.008f;

    [Tooltip("재가동 임계(m): 동결 후 오차가 이보다 커져야 재추종 (히스테리시스)")]
    public float rearmBand = 0.018f;

    [Tooltip("와인드업 가드(도): 드라이브 목표가 실제 관절각보다 이 이상 못 앞서게")]
    public float windupDeg = 15f;

    public bool Active { get; private set; }

    Vector3 _smoothedTarget;
    bool _hasSmoothed;
    bool _frozen;

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
            string s = ab.name.Substring(ab.name.IndexOf("_dg_") + 4);
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
        Vector3 raw = pinch && _indexTip != null
            ? _indexTip.position + (_thumbTip.position - _indexTip.position).normalized * pinchOffset
            : origin + (_palm.TransformDirection(_exL) * tipN.x
                        + _palm.TransformDirection(_eyL) * tipN.y
                        + _palm.TransformDirection(_ezL) * tipN.z) * _handLen;

        // 목표 스무딩 — 비전 노이즈·핀치 전환 점프가 CCD에 직결되지 않게
        if (!_hasSmoothed) { _smoothedTarget = raw; _hasSmoothed = true; }
        float kT = 1f - Mathf.Exp(-targetLerp * Time.fixedDeltaTime);
        _smoothedTarget = Vector3.Lerp(_smoothedTarget, raw, kT);
        Vector3 target = _smoothedTarget;

        // 도달 동결 + 히스테리시스 — 데드밴드 안에서 여유자유도가 배회하는 것 차단
        float err = Vector3.Distance(_thumbTip.position, target);
        if (_frozen)
        {
            if (err < rearmBand) return;
            _frozen = false;
        }
        else if (err < deadband)
        {
            _frozen = true;
            return;
        }

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
                ang = Mathf.Clamp(ang, -maxStepDeg, maxStepDeg);
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
    }
}
