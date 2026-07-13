using UnityEngine;

/// <summary>
/// RobotArm 패키지 JointOperation.cs의 CCD IK를 UR5e ArticulationBody 체인에 이식.
/// 원본은 Transform을 직접 회전시키지만(ArticulationBody와 충돌), 여기서는
/// HandSliderUI 팔 슬라이더 값에 각도를 주입해 기존 lerp 스무딩/xDrive 경로를
/// 재사용한다(SvhHandDriver와 동일 패턴 — xDrive 이중 기록 방지).
/// target을 지정하면 endEffector(tool0)가 그 위치로 향한다. 위치만 추종(자세는 미제어).
/// </summary>
[RequireComponent(typeof(HandSliderUI))]
public class ArmTargetIK : MonoBehaviour
{
    public Transform target;          // 목표 오브젝트 (씬의 IK_Target)
    public Transform endEffector;     // 말단 기준점. 비우면 tool0 자동 탐색
    public bool enableIK = true;

    public float threshold = 0.02f;   // 도달 판정 거리[m]
    [Range(0.5f, 20f)]
    public float rotationSpeed = 5f;  // 원본과 동일 의미: 매 스텝 오차각×speed×dt 만큼 전진
    public float maxStepDeg = 1.8f;   // FixedUpdate당 관절 최대 변화[deg] (50Hz 기준 90°/s 상한)
    public float maxAccelDeg = 0.12f; // 스텝 "증가"의 프레임당 상한[deg] (가속 램프 — 급발진 채찍질 방지)
    [Range(0.2f, 1f)]
    public float distalStepScale = 0.6f; // 손목 3관절(4~6축) 스텝 상한 비율. 손목은 레버가 짧아
                                         // 위치 기여가 작은데 방향이 자주 뒤집혀 지그재그의 주범 → 축소.
    public float deadbandDeg = 0.25f;    // 이 이하 관절 오차각은 무시(미세 헌팅 방지). 0=비활성.
    [Range(1.1f, 3f)]
    public float rearmFactor = 1.5f;     // 도달 후 dist가 threshold×이 배수를 넘어야 재가동(히스테리시스)
    public float windupDeg = 15f;     // 명령이 실제 관절각보다 앞설 수 있는 최대치[deg].
                                      // 오차각은 실제 포즈로 계산하므로, 드라이브 추종이 느릴 때
                                      // 명령만 한계까지 폭주하는 적분 와인드업을 여기서 차단.
    public float[] signs = { 1, 1, 1, 1, 1, 1 }; // 회전 부호가 반대인 관절만 -1로
    [Range(0.5f, 1f)]
    public float reachMargin = 0.88f; // 링크 길이 합 대비 사용 반경 비율.
                                      // 링크 합산 반경은 손목 오프셋 때문에 실제보다 과대 →
                                      // 0.9 이상이면 투영점이 도달 불가능해 미세 배회가 재발함.
    public float stallFreezeSec = 1.5f;  // 이 시간 동안 5mm 이상 개선 없으면 동결(관절 한계·특이점 대비)
    public float stallResumeMove = 0.03f; // 타겟이 이만큼[m] 움직이면 동결 해제

    HandSliderUI _ui;
    ArticulationBody[] _bodies;
    float _reach;           // 어깨 리프트 피벗 기준 최대 도달 반경[m]
    float _bestDist = float.MaxValue;
    float _stallTimer;
    Vector3 _lastTgt;
    float[] _stepPrev;      // 관절별 직전 프레임 적용 스텝[deg] (가속 램프용)
    bool _arrived;          // 도달 상태(히스테리시스): threshold 진입 시 true, threshold*rearmFactor 이탈 시 false

    void Awake()
    {
        _ui = GetComponent<HandSliderUI>();
    }

    void Start()
    {
        // HandSliderUI.Awake에서 armJoints가 구성된 후 링크 이름으로 해석
        var all = GetComponentsInChildren<ArticulationBody>();
        _bodies = new ArticulationBody[_ui.armJoints.Length];
        for (int i = 0; i < _ui.armJoints.Length; i++)
        {
            string link = _ui.armJoints[i].link;
            foreach (var b in all)
                if (b.name == link) { _bodies[i] = b; break; }
            if (_bodies[i] == null)
                Debug.LogError($"[ArmTargetIK] 팔 링크 해석 실패: {link}");
        }

        if (endEffector == null)
        {
            // GraspPoint(파지 중심) 우선, 없으면 tool0(팔 플랜지)로 폴백
            foreach (var t in GetComponentsInChildren<Transform>())
                if (t.name == "GraspPoint") { endEffector = t; break; }
            if (endEffector == null)
                foreach (var t in GetComponentsInChildren<Transform>())
                    if (t.name == "tool0") { endEffector = t; break; }
            if (endEffector == null)
                Debug.LogError("[ArmTargetIK] endEffector 자동 탐색 실패");
        }

        // 도달 반경 = 어깨 리프트 피벗→각 관절 피벗→말단까지 강체 구간 길이 합(자세 불변이라 1회 계산)
        if (_bodies[1] != null && endEffector != null)
        {
            _reach = 0f;
            Vector3 prev = Pivot(_bodies[1]);
            for (int i = 2; i < _bodies.Length; i++)
            {
                if (_bodies[i] == null) continue;
                Vector3 p = Pivot(_bodies[i]);
                _reach += (p - prev).magnitude;
                prev = p;
            }
            _reach += (endEffector.position - prev).magnitude;
            _reach *= reachMargin;
        }
    }

    static Vector3 Pivot(ArticulationBody b)
    {
        return b.transform.TransformPoint(b.anchorPosition);
    }

    void FixedUpdate()
    {
        if (!enableIK || target == null || endEffector == null || _bodies == null) return;

        // 도달 불가능한 타겟은 "그 방향의 최대 도달점"으로 투영.
        // (안 하면 도달 조건이 영원히 안 만족돼 오차 적분이 멈추지 않고 완전 신전 자세에서 진동)
        Vector3 tgt = target.position;
        bool projected = false;
        if (_reach > 0f && _bodies[1] != null)
        {
            Vector3 center = Pivot(_bodies[1]);
            Vector3 off = tgt - center;
            if (off.magnitude > _reach)
            {
                tgt = center + off.normalized * _reach;
                projected = true;
            }
        }

        float dist = (endEffector.position - tgt).magnitude;
        // 범위 밖 타겟의 투영점은 방향에 따라 실제로는 정확히 못 닿는 지점일 수 있어,
        // 빡빡한 threshold로는 경계에서 끝없이 갈아대는 헌팅이 생긴다 → 도달 판정 완화.
        float th = projected ? threshold * 3f : threshold;

        // 도달 히스테리시스: threshold 진입 후에는 threshold*rearmFactor를 벗어나야 재가동.
        // (경계값 언저리에서 매 프레임 on/off를 반복하며 미세 헌팅하는 것을 방지)
        if (_arrived)
        {
            if (dist > th * rearmFactor) _arrived = false;
        }
        else if (dist <= th) _arrived = true;

        // 도달/정체 시에도 즉시 return하지 않고 active=false로 두고 아래 루프를 돌려
        // 마지막 스텝이 0으로 정리되게 한다.
        bool active = !_arrived;
        if (active)
        {
            // 정체 감지: 한동안 거리 개선이 없으면(관절 한계·미도달 지점) 동결해 배회를 막고,
            // 타겟이 유의미하게 움직이면 재개.
            if ((tgt - _lastTgt).magnitude > stallResumeMove)
            {
                _lastTgt = tgt;
                _bestDist = float.MaxValue;
                _stallTimer = 0f;
            }
            if (dist < _bestDist - 0.005f)
            {
                _bestDist = dist;
                _stallTimer = 0f;
            }
            else
            {
                _stallTimer += Time.fixedDeltaTime;
                if (_stallTimer >= stallFreezeSec) active = false; // 동결(감속 정지)
            }
        }

        if (_stepPrev == null) _stepPrev = new float[_bodies.Length];

        float dt = Time.fixedDeltaTime;
        // 순차 CCD: 말단 쪽 관절부터 베이스 방향으로 1패스.
        // 관절 하나를 돌릴 때마다 말단 "예상" 위치(end)를 갱신해, 다음 관절은 남은 오차만
        // 보정한다. (전엔 6관절이 모두 같은 시점의 오차를 각자 전부 보정하려 해 합산
        // 과보정 → 목표를 지나치고 되돌아오는 출렁임이 발생했음)
        Vector3 end = endEffector.position;
        for (int i = _bodies.Length - 1; i >= 0; i--)
        {
            var b = _bodies[i];
            if (b == null) continue;

            // revolute ArticulationBody의 회전축 = 앵커 프레임의 X축
            Vector3 pivot = Pivot(b);
            Vector3 axis = (b.transform.rotation * b.anchorRotation) * Vector3.right;

            // 목표 스텝: 비활성(도달·동결)이면 0으로 감속, 활성이면 오차 비례(데드밴드·상한 적용)
            float desired = 0f;
            if (active)
            {
                Vector3 toEnd = end - pivot;
                Vector3 toTarget = tgt - pivot;
                if (toEnd.sqrMagnitude > 1e-8f && toTarget.sqrMagnitude > 1e-8f)
                {
                    float err = Vector3.SignedAngle(toEnd, toTarget, axis) * signs[i];
                    if (Mathf.Abs(err) > deadbandDeg)
                    {
                        float limit = maxStepDeg * (i >= 3 ? distalStepScale : 1f);
                        desired = Mathf.Clamp(err * rotationSpeed * dt, -limit, limit);
                    }
                }
            }

            // 비대칭 가속 램프: 속도(스텝 크기)를 "키우는" 것만 프레임당 maxAccelDeg로 제한.
            // 감속·방향 전환(브레이크)은 즉시 허용 — 브레이크까지 램프로 묶으면 오버슈트 후에도
            // 지난 방향으로 계속 밀어 저주파 왕복(리밋사이클)이 생긴다(실측으로 확인).
            float prevStep = _stepPrev[i];
            bool sameDir = desired == 0f || prevStep == 0f || (desired > 0f) == (prevStep > 0f);
            float allowed = (sameDir ? Mathf.Abs(prevStep) : 0f) + maxAccelDeg;
            float step = Mathf.Clamp(Mathf.Abs(desired), 0f, allowed) * Mathf.Sign(desired);

            float prev = _ui.armJoints[i].value;
            float v = prev + step;
            float act = b.jointPosition[0] * Mathf.Rad2Deg;
            v = Mathf.Clamp(v, act - windupDeg, act + windupDeg);
            v = Mathf.Clamp(v, _ui.armJoints[i].minDeg, _ui.armJoints[i].maxDeg);
            _ui.armJoints[i].value = v;

            // 클램프까지 반영된 "실제 적용량" 기준으로 램프 상태와 말단 예상 위치를 갱신
            float applied = v - prev;
            _stepPrev[i] = applied;
            if (Mathf.Abs(applied) > 1e-5f)
                end = pivot + Quaternion.AngleAxis(applied * signs[i], axis) * (end - pivot);
        }
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(Screen.width - 250, 10, 240, 95), GUI.skin.box);
        GUILayout.Label("<b>팔 IK (RobotArm 이식)</b>");
        enableIK = GUILayout.Toggle(enableIK, "IK 활성 (타겟 추종)");
        if (target != null && endEffector != null)
            GUILayout.Label($"타겟 거리: {Vector3.Distance(endEffector.position, target.position) * 100f:F1} cm");
        GUILayout.EndArea();
    }
}
