using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// SCHUNK SVH 핸드의 mimic 관절을 소스 관절에 연동한다.
/// Unity URDF-Importer는 URDF의 &lt;mimic&gt;을 자동 처리하지 않으므로,
/// 매 FixedUpdate마다 mimic 관절의 xDrive.target = multiplier * source.target + offset 으로 맞춘다.
/// ArticulationBody는 child link 이름의 GameObject에 붙으므로 link 이름으로 해석한다.
/// </summary>
public class MimicJointController : MonoBehaviour
{
    [Serializable]
    public struct MimicMap
    {
        public string mimicLink;   // mimic 관절의 child link (= GameObject 이름)
        public string sourceLink;  // 소스 관절의 child link
        public float multiplier;
        public float offsetDeg;    // 보통 0
    }

    [Tooltip("비우면 SVH 오른손 기본 매핑(11개)을 사용")]
    public MimicMap[] mimics;

    private ArticulationBody[] _mimicBodies;
    private ArticulationBody[] _sourceBodies;

    // SVH 오른손 기본 mimic 매핑 (결합 URDF에서 추출)
    public static MimicMap[] DefaultSvhRight()
    {
        return new[]
        {
            new MimicMap { mimicLink = "right_hand_e2", sourceLink = "right_hand_z", multiplier = 1.0f,     offsetDeg = 0f }, // j5  <- Thumb_Opposition
            new MimicMap { mimicLink = "right_hand_b",  sourceLink = "right_hand_a", multiplier = 1.01511f, offsetDeg = 0f }, // j3  <- Thumb_Flexion
            new MimicMap { mimicLink = "right_hand_c",  sourceLink = "right_hand_a", multiplier = 1.44889f, offsetDeg = 0f }, // j4  <- Thumb_Flexion
            new MimicMap { mimicLink = "right_hand_t",  sourceLink = "right_hand_p", multiplier = 1.045f,   offsetDeg = 0f }, // j14 <- Index_Distal
            new MimicMap { mimicLink = "right_hand_s",  sourceLink = "right_hand_o", multiplier = 1.0454f,  offsetDeg = 0f }, // j15 <- Middle_Distal
            new MimicMap { mimicLink = "right_hand_n",  sourceLink = "right_hand_j", multiplier = 1.3588f,  offsetDeg = 0f }, // j12 <- Ring_Finger
            new MimicMap { mimicLink = "right_hand_r",  sourceLink = "right_hand_j", multiplier = 1.42093f, offsetDeg = 0f }, // j16 <- Ring_Finger
            new MimicMap { mimicLink = "right_hand_m",  sourceLink = "right_hand_i", multiplier = 1.3588f,  offsetDeg = 0f }, // j13 <- Pinky
            new MimicMap { mimicLink = "right_hand_q",  sourceLink = "right_hand_i", multiplier = 1.42307f, offsetDeg = 0f }, // j17 <- Pinky
            new MimicMap { mimicLink = "right_hand_virtual_l", sourceLink = "right_hand_virtual_i", multiplier = 0.5f, offsetDeg = 0f }, // index_spread <- Finger_Spread
            new MimicMap { mimicLink = "right_hand_virtual_j", sourceLink = "right_hand_virtual_i", multiplier = 0.5f, offsetDeg = 0f }, // ring_spread  <- Finger_Spread
        };
    }

    void Awake()
    {
        if (mimics == null || mimics.Length == 0)
            mimics = DefaultSvhRight();
        Resolve();
    }

    /// <summary>link 이름으로 ArticulationBody를 찾아 캐싱.</summary>
    public void Resolve()
    {
        if (mimics == null || mimics.Length == 0)
            mimics = DefaultSvhRight();

        var byName = new Dictionary<string, ArticulationBody>();
        foreach (var b in GetComponentsInChildren<ArticulationBody>())
            if (!byName.ContainsKey(b.name)) byName[b.name] = b;

        _mimicBodies = new ArticulationBody[mimics.Length];
        _sourceBodies = new ArticulationBody[mimics.Length];
        int ok = 0;
        for (int i = 0; i < mimics.Length; i++)
        {
            byName.TryGetValue(mimics[i].mimicLink, out _mimicBodies[i]);
            byName.TryGetValue(mimics[i].sourceLink, out _sourceBodies[i]);
            if (_mimicBodies[i] != null && _sourceBodies[i] != null) ok++;
        }
        Debug.Log($"[MimicJointController] resolved {ok}/{mimics.Length} mimic pairs");
    }

    void FixedUpdate()
    {
        if (_mimicBodies == null) return;
        for (int i = 0; i < _mimicBodies.Length; i++)
        {
            var mb = _mimicBodies[i];
            var sb = _sourceBodies[i];
            if (mb == null || sb == null) continue;
            var drive = mb.xDrive;
            drive.target = mimics[i].multiplier * sb.xDrive.target + mimics[i].offsetDeg;
            mb.xDrive = drive;
        }
    }
}
