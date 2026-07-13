using UnityEngine;

/// <summary>
/// Play 시작 시 모든 revolute 관절의 jointPosition을 xDrive.target으로 텔레포트하고
/// 속도를 0으로 초기화한다.
///
/// 이유: ArticulationBody는 Play 시작 시 URDF 영점 포즈에서 출발해 드라이브가
/// 목표 포즈로 팔을 홱 들어올리는데, 이때 체인 끝 손가락들이 채찍처럼 휘둘려
/// 수십 초간 남는 출렁임의 에너지원이 된다(2026-07-02 진단).
/// 처음부터 목표 포즈에서 시작하면 이 초기 킥이 원천 차단된다.
///
/// 사용법: ur5e_svh 루트에 부착. Start에서 1회 실행.
/// </summary>
public class RobotInitialPoseSync : MonoBehaviour
{
    void Start()
    {
        int n = 0;
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
        {
            if (ab.jointType != ArticulationJointType.RevoluteJoint) continue;
            if (ab.dofCount < 1) continue;
            float targetRad = ab.xDrive.target * Mathf.Deg2Rad;
            ab.jointPosition = new ArticulationReducedSpace(targetRad);
            ab.jointVelocity = new ArticulationReducedSpace(0f);
            n++;
        }
        Debug.Log($"[RobotInitialPoseSync] {n}개 관절을 목표 포즈로 초기화");
    }
}
