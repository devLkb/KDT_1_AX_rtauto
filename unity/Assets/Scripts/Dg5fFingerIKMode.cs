// Dg5fFingerIKMode.cs
// 한 곳에서 이 손의 모든 손가락 구동 방식을 일괄 통제한다.
// 로봇 루트(Dg5fReceiver·Dg5fFingerIK들과 같은 GameObject)에 붙인다.
//   → Inspector에서 ikMode 드롭다운 하나만 바꾸면 5개 Dg5fFingerIK가 전부 그 방식으로 바뀜.
//     (5개를 손가락마다 하나하나 안 바꿔도 됨. Play 중 실시간 반영 → A/B 즉시 비교)
// applyToAll을 끄면 통제를 멈추고 각 Dg5fFingerIK가 자기 설정을 개별 사용(손가락별 다른 방식 실험용).

using UnityEngine;

public class Dg5fFingerIKMode : MonoBehaviour
{
    // 앞 3개는 Dg5fFingerIK.FingerIKMode와 **같은 순서·정수값**이어야 한다(캐스팅 + 직렬화 호환).
    // JointAnglesOnly는 IK 목표 계산이 아니라 "IK 전부 끄기" — Dg5fFingerIK.Active=false가 되면서
    // Dg5fHandDriver가 패킷 [0..19] 관절각 채널로 20관절을 직접 구동한다(실물 SDK와 같은 방식).
    public enum HandDriveMode
    {
        AnatomicalReach,     // §26 방식: 해부학 방향 × 펴짐비율 × 방향별 FK 최대도달 테이블
        RobotRootTipVector,  // 손목 기준: 사람 손목→끝 벡터(v5) × 로봇 손길이
        ChainRatioReach,     // 마디합 비율: 3:3 마디 대응, 앵커 n_2 피벗
        JointAnglesOnly,     // IK 미사용 — 관절각 채널(사람각→로봇각 매핑, dg5f_angles) 직접 구동
    }

    [Tooltip("여기 하나만 바꾸면 이 손의 모든 손가락 구동 방식이 전환된다(5개 개별 설정 불필요). "
             + "AnatomicalReach=§26(해부학 방향×FK 도달테이블) / RobotRootTipVector=손목 기준(손목→끝 벡터×로봇 손길이) / "
             + "ChainRatioReach=마디합 비율(3:3 마디 대응) / JointAnglesOnly=IK 끄고 관절각 채널로만 구동"
             + "(실물 Tesollo SDK와 동일한 관절각 인터페이스 — 트윈/실물 동작 비교용). Play 중에도 실시간 반영")]
    public HandDriveMode ikMode = HandDriveMode.AnatomicalReach;

    [Tooltip("끄면 일괄 통제 중지 — 각 Dg5fFingerIK가 자기 ikMode/enableIK를 개별 사용(손가락마다 다른 방식 실험용)")]
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
        bool anglesOnly = ikMode == HandDriveMode.JointAnglesOnly;
        foreach (var f in _fingers)
        {
            if (f == null) continue;
            if (anglesOnly)
            {
                // enableIK=false → FingerIK.Active=false → Dg5fHandDriver가 이 손가락
                // 4채널을 관절각으로 구동(기존 폴백 경로 그대로 재사용, 드라이버 수정 불필요)
                if (f.enableIK) f.enableIK = false;
            }
            else
            {
                if (!f.enableIK) f.enableIK = true;
                var m = (Dg5fFingerIK.FingerIKMode)(int)ikMode;
                if (f.ikMode != m) f.ikMode = m;
            }
        }
    }
}
