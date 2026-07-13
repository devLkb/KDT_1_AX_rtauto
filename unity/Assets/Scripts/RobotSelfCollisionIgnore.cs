using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// 로봇(팔+손) 자기 콜라이더끼리의 충돌을 전부 비활성화한다.
///
/// 이유: dex-urdf SVH는 인접 지골 콜라이더가 설계상 서로 겹쳐 있고(최대 ~13mm),
/// 손 베이스가 팔 wrist_3/플랜지 콜라이더와도 겹쳐 있어, PhysX가 매 스텝 밀어내고
/// xDrive가 되끌어오는 접촉 진동(limit cycle)이 발생
/// (관절이 명령 0인데 ±20~100° 진동, 2026-07-02 tracking_report + 콜라이더 on/off
/// 이분법 실험으로 진단 — 콜라이더 전체 off 시 진동 소멸).
/// 디지털 트윈은 실제 손/조작 명령이 출처라 자기침투 명령이 오지 않으므로
/// 자기충돌 처리는 불필요. 외부 물체(Ground 등)와의 충돌은 그대로 유지된다.
///
/// 사용법: ur5e_svh 루트에 부착. Start에서 1회 처리.
/// </summary>
public class RobotSelfCollisionIgnore : MonoBehaviour
{
    void Start()
    {
        var cols = new List<Collider>();
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
            foreach (var c in ab.GetComponentsInChildren<Collider>())
                // 자식 링크의 콜라이더는 그 링크가 처리하므로 이 링크 소속만
                if (c.GetComponentInParent<ArticulationBody>() == ab)
                    cols.Add(c);

        int pairs = 0;
        for (int i = 0; i < cols.Count; i++)
            for (int j = i + 1; j < cols.Count; j++)
            {
                Physics.IgnoreCollision(cols[i], cols[j], true);
                pairs++;
            }
        Debug.Log($"[RobotSelfCollisionIgnore] 로봇 자기충돌 비활성화: 콜라이더 {cols.Count}개, {pairs}쌍");
    }
}
