using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// SvhReceiver가 UDP로 받은 SVH 9관절 각도(rad, 이미 SVH URDF 가동범위)를
/// 로봇 손 관절에 실시간 주입한다. (Phase 3: 웹캠 손 → 가상 SVH 미러링)
///
/// 패킷 순서(= Python svh_angles.CHANNEL_NAMES = SVH 드라이버 JTC/SVHChannel enum):
///   [0]Thumb_Flexion  [1]Thumb_Opposition
///   [2]Index_Distal   [3]Index_Proximal
///   [4]Middle_Distal  [5]Middle_Proximal
///   [6]Ring           [7]Pinky            [8]Finger_Spread
/// ⚠️ 검지·중지는 패킷이 Distal→Proximal 순이라 Unity 링크(l=Prox, p=Dist)와
///    위치가 다르다. 반드시 아래 PacketIndexToLink 이름 매핑대로 꽂는다(위치 매핑 금지).
///
/// 각도는 rad*Rad2Deg로 도 변환(xDrive.target 단위=도). HandSliderUI가 있으면
/// 그 슬라이더 값에 주입해 기존 스무딩/GUI/xDrive 적용을 재사용하고,
/// 없으면 xDrive.target에 직접 쓴다. mimic 관절은 MimicJointController가 전파.
/// </summary>
[RequireComponent(typeof(SvhReceiver))]
public class SvhHandDriver : MonoBehaviour
{
    [Tooltip("체크 해제하면 추적 중단(HandSliderUI 수동 슬라이더 조작 가능)")]
    public bool enableTracking = true;

    [Tooltip("엄지 대향이 반대로 움직이면 체크(가동범위 내에서 방향 반전). " +
             "최종 확정 시엔 Python svh_angles.py의 thumb_opposition (svh_min,svh_max) 스왑 권장.")]
    public bool invertThumbOpposition = false;

    // 패킷 인덱스 -> child link 이름(GameObject 이름). rad->deg는 공통.
    static readonly string[] PacketIndexToLink =
    {
        "right_hand_a",          // 0 Thumb_Flexion
        "right_hand_z",          // 1 Thumb_Opposition
        "right_hand_p",          // 2 Index_Finger_Distal
        "right_hand_l",          // 3 Index_Finger_Proximal
        "right_hand_o",          // 4 Middle_Finger_Distal
        "right_hand_k",          // 5 Middle_Finger_Proximal
        "right_hand_j",          // 6 Ring_Finger
        "right_hand_i",          // 7 Pinky
        "right_hand_virtual_i",  // 8 Finger_Spread
    };
    const int THUMB_OPP_IDX = 1;

    private SvhReceiver _rx;
    private HandSliderUI _sliderUI;
    private Dictionary<string, ArticulationBody> _bodies;

    void Start()
    {
        _rx = GetComponent<SvhReceiver>();
        _sliderUI = GetComponent<HandSliderUI>();
        _bodies = new Dictionary<string, ArticulationBody>();
        foreach (var b in GetComponentsInChildren<ArticulationBody>())
            if (!_bodies.ContainsKey(b.name)) _bodies[b.name] = b;
    }

    void FixedUpdate()
    {
        if (!enableTracking || _rx == null) return;
        float[] a = _rx.GetAngles();   // rad, 길이 9
        if (a == null || a.Length < PacketIndexToLink.Length) return;

        for (int i = 0; i < PacketIndexToLink.Length; i++)
        {
            string link = PacketIndexToLink[i];
            float deg = a[i] * Mathf.Rad2Deg;
            if (i == THUMB_OPP_IDX && invertThumbOpposition &&
                _bodies.TryGetValue(link, out var tb))
                deg = tb.xDrive.upperLimit - deg;   // 범위 내 방향 반전
            SetJoint(link, deg);
        }
    }

    void SetJoint(string link, float deg)
    {
        // 1) HandSliderUI가 있으면 슬라이더 값에 주입 → 기존 lerp 스무딩/GUI 재사용.
        if (_sliderUI != null && _sliderUI.handJoints != null)
        {
            for (int i = 0; i < _sliderUI.handJoints.Length; i++)
            {
                if (_sliderUI.handJoints[i].link == link)
                {
                    _sliderUI.handJoints[i].value = deg;
                    return;
                }
            }
        }
        // 2) 폴백: xDrive.target 직접 설정(HandSliderUI 없을 때).
        if (_bodies != null && _bodies.TryGetValue(link, out var body))
        {
            var d = body.xDrive;
            d.target = deg;
            body.xDrive = d;
        }
    }
}
