using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// UR5e + SCHUNK SVH 디지털 트윈의 "튜닝 파라미터 단일 출처(single source of truth)".
///
/// 값의 출처를 3가지로 명확히 구분한다:
///   [REAL-UR]  : UR 공식 config/ur5e/joint_limits.yaml (UR5e User Manual v5.8 + UR max-joint-torques article)
///   [REAL-URDF]: 모델 URDF에 정의된 값 (각도 한계 등 — 기구학적으로 실제 모델 값)
///   [PLACEHOLDER]: 실제 기기/데이터시트로 교체해야 하는 임시값 (제어 게인, SVH 액추에이터 사양 등)
///
/// 실제 기기값을 받으면 *이 파일 한 곳만* 고치면 된다.
/// Apply(robotRoot) 호출 시 모든 ArticulationBody에 반영된다.
/// </summary>
public class RobotConfig : MonoBehaviour
{
    [Serializable]
    public struct JointSpec
    {
        public string label;
        public string link;       // child link = ArticulationBody GameObject 이름
        public float minDeg;      // 관절 하한 (도)
        public float maxDeg;      // 관절 상한 (도)
        public float maxVelDeg;   // 최대 각속도 (도/s)
        public float maxEffortNm; // 최대 토크 (N·m)
    }

    // ───────────────────────── 제어 게인 (xDrive) ─────────────────────────
    // [PLACEHOLDER] stiffness/damping는 실제 컨트롤러 PD 게인이 아니라
    // "명령 포즈를 안정적으로 추종"하게 만드는 임시 튜닝값이다.
    // UR 실기는 자체 내부 컨트롤러가 동역학을 처리하므로, 위치 미러링 목적엔
    // 이 값을 실기와 똑같이 맞출 필요는 없다. (sim-to-real이 목표면 실측 튜닝 필요)
    [Header("Arm drive gains  [PLACEHOLDER]")]
    public float armStiffness = 10000f; // [PLACEHOLDER]
    public float armDamping = 200f;     // [PLACEHOLDER]
    // forceLimit는 관절별 max_effort(REAL-UR)로 Apply에서 덮어쓴다.

    [Header("Hand drive gains  [PLACEHOLDER]")]
    // 주의: 아래 값은 "실제 SVH 액추에이터 힘"이 아니다.
    // 실제 SVH 손가락 힘은 수 N 수준으로 더 작지만, Unity ArticulationBody에서
    // 손가락 자가 충돌(self-collision)을 이기고 명령 각도를 안정적으로 추종하려면
    // 드라이브가 충분히 강해야 한다(시뮬레이터 특성). 너무 낮추면 관절이 충돌에 밀려 엉뚱한 각도로 감.
    // → 물리 정확 트윈이 목표면 SVH 데이터시트 힘 + self-collision 모델 재검토 필요.
    public float handStiffness = 10000f;  // [PLACEHOLDER] 위치추종 안정성용 (실제 PD 게인 아님)
    public float handDamping = 200f;      // [PLACEHOLDER]
    public float handForceLimit = 1000f;  // [PLACEHOLDER] SVH URDF effort=1000(generic) 기준. 실제 손가락 힘은 더 작음.

    [Header("Digital-twin 물리 설정")]
    [Tooltip("미러링은 명령 포즈를 그대로 유지해야 하므로 보통 false. 물리 트윈이면 true + 중력보상 필요")]
    public bool useGravity = false;     // [선택] 미러링 기본값
    [Tooltip("베이스를 월드에 고정")]
    public bool baseImmovable = true;
    [Tooltip("키네마틱 미러 모드: 모든 Collider 비활성화. " +
             "손가락이 굽을 때 self-collision(vHACD 충돌 메시 겹침)이 솔버를 불안정하게 만들어 " +
             "관절이 엉뚱한 각도로 튕기는 것을 방지한다. 미러링엔 충돌이 불필요. " +
             "물건 잡기 등 물리 상호작용이 필요하면 false로 두고 self-collision 예외를 따로 설정해야 함.")]
    public bool kinematicMirrorMode = true;

    // ───────────────────────── 관절 사양 테이블 ─────────────────────────
    public JointSpec[] armJoints;   // UR5e 6축
    public JointSpec[] handJoints;  // SVH 구동 9 DOF

    void Reset() { LoadDefaults(); }

    public void LoadDefaults()
    {
        // ===== 스칼라 게인/플래그 (기존 직렬화 값 덮어쓰기) =====
        // 필드 초기화자는 새 인스턴스에만 적용되므로, 기존 컴포넌트 리셋용으로 여기서도 설정한다.
        armStiffness = 10000f; armDamping = 200f;
        handStiffness = 10000f; handDamping = 200f; handForceLimit = 1000f;
        useGravity = false; baseImmovable = true;

        // ===== UR5e =====
        // [REAL-UR] 출처: Universal_Robots_ROS2_Description/config/ur5e/joint_limits.yaml
        //   (UR5e User Manual v5.8 / UR "Max. joint torques" article)
        //   - shoulder/elbow 계열 max_effort = 150 N·m, wrist = 28 N·m
        //   - 모든 관절 max_velocity = 180 deg/s
        //   - position: 대부분 ±360°, elbow만 ±180°(플래닝 회피 목적으로 config에서 인위적 제한)
        armJoints = new[]
        {
            new JointSpec { label = "Shoulder Pan",  link = "shoulder_link",  minDeg = -360, maxDeg = 360, maxVelDeg = 180, maxEffortNm = 150 }, // [REAL-UR]
            new JointSpec { label = "Shoulder Lift", link = "upper_arm_link", minDeg = -360, maxDeg = 360, maxVelDeg = 180, maxEffortNm = 150 }, // [REAL-UR]
            new JointSpec { label = "Elbow",         link = "forearm_link",   minDeg = -180, maxDeg = 180, maxVelDeg = 180, maxEffortNm = 150 }, // [REAL-UR] elbow는 ±180°로 제한됨
            new JointSpec { label = "Wrist 1",       link = "wrist_1_link",   minDeg = -360, maxDeg = 360, maxVelDeg = 180, maxEffortNm = 28 },  // [REAL-UR]
            new JointSpec { label = "Wrist 2",       link = "wrist_2_link",   minDeg = -360, maxDeg = 360, maxVelDeg = 180, maxEffortNm = 28 },  // [REAL-UR]
            new JointSpec { label = "Wrist 3",       link = "wrist_3_link",   minDeg = -360, maxDeg = 360, maxVelDeg = 180, maxEffortNm = 28 },  // [REAL-UR]
        };

        // ===== SCHUNK SVH 오른손 (구동 9 DOF) =====
        // [REAL-URDF] minDeg/maxDeg: dex-urdf schunk_svh_hand_right.urdf의 <limit lower/upper>(rad)를 도로 환산 — 실제 모델 가동범위.
        // [PLACEHOLDER] maxVelDeg: URDF velocity=1 rad/s(=57.3 deg/s)는 generic 값. 실제 SVH는 손가락별 속도가 다름 → SCHUNK SVH 데이터시트로 교체.
        // [PLACEHOLDER] effort: URDF effort=1000은 generic → handForceLimit(위)로 통일 관리.
        const float R2D = 57.29578f;
        handJoints = new[]
        {
            new JointSpec { label = "Thumb Flexion",    link = "right_hand_a",         minDeg = 0, maxDeg = 0.9704f * R2D,  maxVelDeg = 1f * R2D, maxEffortNm = 0 }, // [REAL-URDF] range / [PLACEHOLDER] vel
            new JointSpec { label = "Thumb Opposition", link = "right_hand_z",         minDeg = 0, maxDeg = 0.9879f * R2D,  maxVelDeg = 1f * R2D, maxEffortNm = 0 },
            new JointSpec { label = "Index Proximal",   link = "right_hand_l",         minDeg = 0, maxDeg = 0.79849f * R2D, maxVelDeg = 1f * R2D, maxEffortNm = 0 },
            new JointSpec { label = "Index Distal",     link = "right_hand_p",         minDeg = 0, maxDeg = 1.334f * R2D,   maxVelDeg = 1f * R2D, maxEffortNm = 0 },
            new JointSpec { label = "Middle Proximal",  link = "right_hand_k",         minDeg = 0, maxDeg = 0.79849f * R2D, maxVelDeg = 1f * R2D, maxEffortNm = 0 },
            new JointSpec { label = "Middle Distal",    link = "right_hand_o",         minDeg = 0, maxDeg = 1.334f * R2D,   maxVelDeg = 1f * R2D, maxEffortNm = 0 },
            new JointSpec { label = "Ring Finger",      link = "right_hand_j",         minDeg = 0, maxDeg = 0.98175f * R2D, maxVelDeg = 1f * R2D, maxEffortNm = 0 },
            new JointSpec { label = "Pinky",            link = "right_hand_i",         minDeg = 0, maxDeg = 0.98175f * R2D, maxVelDeg = 1f * R2D, maxEffortNm = 0 },
            new JointSpec { label = "Finger Spread",    link = "right_hand_virtual_i", minDeg = 0, maxDeg = 0.5829f * R2D,  maxVelDeg = 1f * R2D, maxEffortNm = 0 },
        };
    }

    /// <summary>설정값을 로봇의 모든 ArticulationBody에 반영.</summary>
    public void Apply()
    {
        if (armJoints == null || armJoints.Length == 0) LoadDefaults();

        var byName = new Dictionary<string, ArticulationBody>();
        foreach (var b in GetComponentsInChildren<ArticulationBody>())
            if (!byName.ContainsKey(b.name)) byName[b.name] = b;

        // 전역: 중력/베이스 고정
        foreach (var b in GetComponentsInChildren<ArticulationBody>())
        {
            b.useGravity = useGravity;
            if (b.isRoot) b.immovable = baseImmovable;
        }

        // 키네마틱 미러: self-collision 불안정 방지를 위해 Collider 비활성화
        int colOff = 0;
        foreach (var col in GetComponentsInChildren<Collider>())
        {
            col.enabled = !kinematicMirrorMode;
            if (kinematicMirrorMode) colOff++;
        }
        if (kinematicMirrorMode) Debug.Log($"[RobotConfig] disabled {colOff} colliders (kinematic mirror mode)");

        int n = 0;
        n += ApplyGroup(armJoints, byName, armStiffness, armDamping, useArmEffortAsForceLimit: true);
        n += ApplyGroup(handJoints, byName, handStiffness, handDamping, useArmEffortAsForceLimit: false);
        Debug.Log($"[RobotConfig] applied to {n} joints (gravity={useGravity}, baseImmovable={baseImmovable})");
    }

    int ApplyGroup(JointSpec[] specs, Dictionary<string, ArticulationBody> byName,
                   float stiffness, float damping, bool useArmEffortAsForceLimit)
    {
        int n = 0;
        foreach (var s in specs)
        {
            if (!byName.TryGetValue(s.link, out var b)) continue;
            if (b.jointType != ArticulationJointType.RevoluteJoint) continue;
            var d = b.xDrive;
            d.stiffness = stiffness;
            d.damping = damping;
            d.forceLimit = useArmEffortAsForceLimit ? s.maxEffortNm : handForceLimit; // [REAL-UR] arm은 실제 토크 한계 사용
            d.lowerLimit = s.minDeg;
            d.upperLimit = s.maxDeg;
            b.xDrive = d;
            n++;
        }
        return n;
    }

    /// <summary>주어진 관절 link의 사양 조회 (HandSliderUI 등에서 사용).</summary>
    public bool TryGetSpec(string link, out JointSpec spec)
    {
        foreach (var s in armJoints) if (s.link == link) { spec = s; return true; }
        foreach (var s in handJoints) if (s.link == link) { spec = s; return true; }
        spec = default;
        return false;
    }
}
