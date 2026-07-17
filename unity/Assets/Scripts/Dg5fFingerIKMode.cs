// Dg5fFingerIKMode.cs
// 한 곳에서 이 손의 모든 손가락 IK 방식을 일괄 통제한다.
// 로봇 루트(Dg5fReceiver·Dg5fFingerIK들과 같은 GameObject)에 붙인다.
//   → Inspector에서 ikMode 드롭다운 하나만 바꾸면 5개 Dg5fFingerIK.ikMode가 전부 그 값으로 바뀜.
//     (5개를 손가락마다 하나하나 안 바꿔도 됨. Play 중 실시간 반영 → A/B 즉시 비교)
// applyToAll을 끄면 통제를 멈추고 각 Dg5fFingerIK가 자기 ikMode를 개별 사용(손가락별 다른 방식 실험용).

using UnityEngine;

public class Dg5fFingerIKMode : MonoBehaviour
{
    [Tooltip("여기 하나만 바꾸면 이 손의 모든 Dg5fFingerIK가 같은 방식으로 전환된다(5개 개별 설정 불필요). "
             + "AnatomicalReach=§26 방식(해부학 방향×FK 도달테이블) / RobotRootTipVector=손목 기준(손목→끝 벡터×로봇 손길이) / "
             + "ChainRatioReach=마디합 비율(뿌리관절→끝 ÷ 사람 마디합 × 로봇 마디합). Play 중에도 실시간 반영")]
    public Dg5fFingerIK.FingerIKMode ikMode = Dg5fFingerIK.FingerIKMode.AnatomicalReach;

    [Tooltip("끄면 일괄 통제 중지 — 각 Dg5fFingerIK가 자기 ikMode를 개별 사용(손가락마다 다른 방식 실험용)")]
    public bool applyToAll = true;

    Dg5fFingerIK[] _fingers;

    void Start()
    {
        _fingers = GetComponents<Dg5fFingerIK>();
        Apply();
    }

    void Update()
    {
        Apply();   // 매 프레임 반영 — Play 중 드롭다운을 바꾸면 그 자리에서 전환
    }

    void Apply()
    {
        if (!applyToAll || _fingers == null) return;
        foreach (var f in _fingers)
            if (f != null && f.ikMode != ikMode) f.ikMode = ikMode;
    }
}
