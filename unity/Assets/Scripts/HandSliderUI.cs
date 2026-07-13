using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Play 모드에서 화면 슬라이더로 UR5e 팔(6축) + SVH 손(9 DOF)을 직접 조작한다.
/// 각 관절의 child link 이름으로 ArticulationBody를 찾아 xDrive.target(도)를 설정.
/// mimic 관절은 MimicJointController가 자동 전파.
/// </summary>
public class HandSliderUI : MonoBehaviour
{
    [Serializable]
    public struct JointSlider
    {
        public string label;
        public string link;     // child link = GameObject 이름
        public float minDeg;
        public float maxDeg;
        [HideInInspector] public float value;
    }

    public JointSlider[] armJoints;
    public JointSlider[] handJoints;

    [Range(1f, 30f)] public float lerpSpeed = 12f;   // 슬라이더→타깃 부드럽게
    public bool showArm = true;

    private Dictionary<string, ArticulationBody> _bodies;
    private bool _showHand = true;

    void Reset() { LoadDefaults(); }

    void Awake()
    {
        // 관절 범위/목록은 RobotConfig(단일 출처)에서 가져온다. 없으면 자체 기본값.
        var cfg = GetComponent<RobotConfig>();
        if (cfg != null)
            BuildFromConfig(cfg);
        else if (armJoints == null || armJoints.Length == 0 ||
                 handJoints == null || handJoints.Length == 0)
            LoadDefaults();

        ResolveBodies();
        // 슬라이더 초기값 = 현재 target
        InitValues(ref armJoints);
        InitValues(ref handJoints);
    }

    /// <summary>RobotConfig의 관절 사양으로 슬라이더 목록 구성 (범위를 실제 한계로 스냅).</summary>
    void BuildFromConfig(RobotConfig cfg)
    {
        if (cfg.armJoints == null || cfg.armJoints.Length == 0) cfg.LoadDefaults();
        armJoints = ToSliders(cfg.armJoints);
        handJoints = ToSliders(cfg.handJoints);
    }

    JointSlider[] ToSliders(RobotConfig.JointSpec[] specs)
    {
        var arr = new JointSlider[specs.Length];
        for (int i = 0; i < specs.Length; i++)
            arr[i] = new JointSlider { label = specs[i].label, link = specs[i].link, minDeg = specs[i].minDeg, maxDeg = specs[i].maxDeg };
        return arr;
    }

    public void LoadDefaults()
    {
        armJoints = new[]
        {
            new JointSlider { label = "Shoulder Pan",  link = "shoulder_link",  minDeg = -180, maxDeg = 180 },
            new JointSlider { label = "Shoulder Lift", link = "upper_arm_link", minDeg = -180, maxDeg = 180 },
            new JointSlider { label = "Elbow",         link = "forearm_link",   minDeg = -180, maxDeg = 180 },
            new JointSlider { label = "Wrist 1",       link = "wrist_1_link",   minDeg = -180, maxDeg = 180 },
            new JointSlider { label = "Wrist 2",       link = "wrist_2_link",   minDeg = -180, maxDeg = 180 },
            new JointSlider { label = "Wrist 3",       link = "wrist_3_link",   minDeg = -180, maxDeg = 180 },
        };
        handJoints = new[]
        {
            new JointSlider { label = "Thumb Flexion",    link = "right_hand_a",         minDeg = 0, maxDeg = 55.6f },
            new JointSlider { label = "Thumb Opposition", link = "right_hand_z",         minDeg = 0, maxDeg = 56.6f },
            new JointSlider { label = "Index Proximal",   link = "right_hand_l",         minDeg = 0, maxDeg = 45.7f },
            new JointSlider { label = "Index Distal",     link = "right_hand_p",         minDeg = 0, maxDeg = 76.4f },
            new JointSlider { label = "Middle Proximal",  link = "right_hand_k",         minDeg = 0, maxDeg = 45.7f },
            new JointSlider { label = "Middle Distal",    link = "right_hand_o",         minDeg = 0, maxDeg = 76.4f },
            new JointSlider { label = "Ring Finger",      link = "right_hand_j",         minDeg = 0, maxDeg = 56.2f },
            new JointSlider { label = "Pinky",            link = "right_hand_i",         minDeg = 0, maxDeg = 56.2f },
            new JointSlider { label = "Finger Spread",    link = "right_hand_virtual_i", minDeg = 0, maxDeg = 33.4f },
        };
    }

    void ResolveBodies()
    {
        _bodies = new Dictionary<string, ArticulationBody>();
        foreach (var b in GetComponentsInChildren<ArticulationBody>())
            if (!_bodies.ContainsKey(b.name)) _bodies[b.name] = b;
    }

    void InitValues(ref JointSlider[] arr)
    {
        for (int i = 0; i < arr.Length; i++)
            if (_bodies.TryGetValue(arr[i].link, out var b))
                arr[i].value = b.xDrive.target;
    }

    void FixedUpdate()
    {
        if (_bodies == null) return;
        Apply(armJoints);
        Apply(handJoints);
    }

    void Apply(JointSlider[] arr)
    {
        float t = Mathf.Clamp01(Time.fixedDeltaTime * lerpSpeed);
        foreach (var j in arr)
        {
            if (_bodies.TryGetValue(j.link, out var b))
            {
                var d = b.xDrive;
                d.target = Mathf.Lerp(d.target, j.value, t);
                b.xDrive = d;
            }
        }
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10, 10, 320, Screen.height - 20), GUI.skin.box);
        GUILayout.Label("<b>UR5e + SVH 직접 조작</b>");

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("손 펴기")) SetAll(handJoints, 0f);
        if (GUILayout.Button("주먹 쥐기")) GripAll(0.85f);
        GUILayout.EndHorizontal();

        _showHand = GUILayout.Toggle(_showHand, "손가락 슬라이더 표시");
        if (_showHand) DrawSliders(ref handJoints);

        showArm = GUILayout.Toggle(showArm, "팔 슬라이더 표시");
        if (showArm) DrawSliders(ref armJoints);

        GUILayout.EndArea();
    }

    void DrawSliders(ref JointSlider[] arr)
    {
        for (int i = 0; i < arr.Length; i++)
        {
            GUILayout.Label($"{arr[i].label}: {arr[i].value:F0}°");
            arr[i].value = GUILayout.HorizontalSlider(arr[i].value, arr[i].minDeg, arr[i].maxDeg);
        }
    }

    void SetAll(JointSlider[] arr, float deg)
    {
        for (int i = 0; i < arr.Length; i++) arr[i].value = deg;
    }

    // 손가락 굴곡 관절만 비율(0~1)로 쥐기. 벌림(spread)은 0 유지.
    void GripAll(float ratio)
    {
        for (int i = 0; i < handJoints.Length; i++)
        {
            bool isSpread = handJoints[i].link == "right_hand_virtual_i";
            handJoints[i].value = isSpread ? 0f : handJoints[i].maxDeg * ratio;
        }
    }
}
