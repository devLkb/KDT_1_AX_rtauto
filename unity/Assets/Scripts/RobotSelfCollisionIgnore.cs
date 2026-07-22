using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 로봇(팔+손) 자기 콜라이더끼리의 충돌을 켜고 끌 수 있게 한다(런타임 토글).
///
/// 이유: dex-urdf SVH/DG5F는 인접 지골 콜라이더가 설계상 서로 겹쳐 있고(최대 ~13mm),
/// 손 베이스가 팔 wrist_3/플랜지 콜라이더와도 겹쳐 있어, 자기충돌을 켜면 PhysX가 매 스텝
/// 밀어내고 xDrive가 되끌어오는 접촉 진동(limit cycle)이 생긴다
/// (관절이 명령 0인데 ±20~100° 진동, 2026-07-02 진단 — 콜라이더 off 시 진동 소멸).
/// 디지털 트윈(추종)은 자기침투 명령이 안 오므로 평소엔 '무시(ignore)'가 정답이지만,
/// 파지(grasp) 실험 등에서 손가락끼리 물리 접촉이 필요하면 잠깐 켤 수 있어야 한다.
/// 외부 물체(Ground 등)와의 충돌은 이 설정과 무관하게 항상 유지된다.
///
/// 사용법: 로봇 루트(ur5e_svh / DG5F 루트)에 부착.
///   • ignoreSelfCollision 체크박스 = 자기충돌 무시(기본 ON=통과). 플레이 중 토글 가능.
///   • toggleKey = 지정 시 그 키로 런타임 토글(레거시 Input Manager 필요, None이면 미사용).
///   • SetIgnoreSelfCollision(bool) = UI 토글/버튼(onValueChanged)이나 외부 스크립트에서 호출.
/// ⚠️ 자기충돌을 켜면(ignore=false) 겹친 콜라이더 때문에 위 접촉 진동이 다시 생길 수 있다.
/// </summary>
public class RobotSelfCollisionIgnore : MonoBehaviour
{
    [Tooltip("켜짐(기본): 로봇 자기충돌 무시 — 손가락끼리 통과, 접촉 진동 없음. "
             + "꺼짐: 자기충돌 활성 — 손가락끼리 물리 충돌(겹친 지골 때문에 진동 생길 수 있음). "
             + "플레이 중에도 이 체크박스로 토글 가능.")]
    public bool ignoreSelfCollision = true;

    [Tooltip("이 키로 런타임 토글(레거시 Input Manager 필요). None이면 키 토글 사용 안 함.")]
    public KeyCode toggleKey = KeyCode.None;

    readonly List<Collider> _cols = new List<Collider>();
    bool _applied;          // 최초 1회 적용 여부
    bool _lastState;        // 마지막으로 적용한 ignore 상태(변경 감지용)

    void Start()
    {
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
            foreach (var c in ab.GetComponentsInChildren<Collider>())
                // 자식 링크의 콜라이더는 그 링크가 처리하므로 이 링크 소속만
                if (c.GetComponentInParent<ArticulationBody>() == ab)
                    _cols.Add(c);
        Apply(ignoreSelfCollision);
    }

    void Update()
    {
        if (toggleKey != KeyCode.None && Input.GetKeyDown(toggleKey))
            ignoreSelfCollision = !ignoreSelfCollision;
        // 체크박스/키/외부호출로 값이 바뀌었으면 그때만 전 쌍에 재적용(매 프레임 재적용 안 함)
        if (!_applied || ignoreSelfCollision != _lastState)
            Apply(ignoreSelfCollision);
    }

    /// UI Toggle의 onValueChanged(bool)나 외부 스크립트에서 호출.
    public void SetIgnoreSelfCollision(bool ignore) => ignoreSelfCollision = ignore;

    /// 코드에서 즉시 뒤집고 싶을 때.
    public void ToggleSelfCollision() => ignoreSelfCollision = !ignoreSelfCollision;

    void Apply(bool ignore)
    {
        int pairs = 0;
        for (int i = 0; i < _cols.Count; i++)
        {
            if (_cols[i] == null) continue;
            for (int j = i + 1; j < _cols.Count; j++)
            {
                if (_cols[j] == null) continue;
                Physics.IgnoreCollision(_cols[i], _cols[j], ignore);
                pairs++;
            }
        }
        _applied = true;
        _lastState = ignore;
        string state = ignore ? "무시(OFF·통과)" : "활성(ON·충돌)";
        Debug.Log($"[RobotSelfCollisionIgnore] 자기충돌 {state}: 콜라이더 {_cols.Count}개, {pairs}쌍 적용");
    }
}
