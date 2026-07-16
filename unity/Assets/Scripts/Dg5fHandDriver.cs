// Dg5fHandDriver.cs
// Dg5fReceiver의 20채널 각도[deg]를 DG5F 20관절 xDrive.target에 주입.
//
// 매핑: 패킷 인덱스 → 관절 링크 이름 접미사 "_dg_<손가락>_<마디>" (이름 매칭 — 위치 매칭 금지).
//   접미사 매칭이라 오른손(rl_dg_*)/왼손(ll_dg_*) 프리팹 모두 동작.
// 순서(계약, Python dg5f_angles.py와 동일):
//   [0..3] 엄지 1_1~1_4 / [4..7] 검지 2_1~2_4 / [8..11] 중지 3_x / [12..15] 약지 4_x / [16..19] 새끼 5_x
//
// 값은 관절공간 각도[deg]를 그대로 받는다(사람→관절 매핑·보정·방향은 Python 담당).
// 여기서는 ① URDF 리밋으로 clamp ② lerp 스무딩 ③ xDrive.target 기록만 한다.
// 로봇 루트에 부착 (Dg5fReceiver와 같은 GameObject).

using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Dg5fReceiver))]
public class Dg5fHandDriver : MonoBehaviour
{
    [Tooltip("끄면 수신값 주입 중단 (수동 포즈 테스트용 — IK/파지 실험 시 끌 것)")]
    public bool enableTracking = true;

    [Tooltip("목표각 스무딩 속도 (HandSliderUI lerp와 동일 컨셉)")]
    public float lerpSpeed = 12f;

    [Tooltip("이 시간(초) 이상 패킷이 없으면 마지막 포즈 유지(추가 주입 안 함)")]
    public float staleTimeout = 1.0f;

    Dg5fReceiver _receiver;
    // 손가락별 IK — 활성인 손가락의 4채널은 그 IK가 담당하고 여기선 주입을 건너뛴다.
    // (같은 GameObject에 fingerIndex만 다르게 여러 개 붙는다. 없거나 비활성이면 각도 방식.)
    Dg5fFingerIK[] _fingerIKs;
    readonly bool[] _ikOwned = new bool[5];   // [손가락-1] = 이번 틱에 IK가 구동 중인가
    ArticulationBody[] _joints;          // 패킷 인덱스 순서
    readonly float[] _angles = new float[Dg5fReceiver.ChannelCount];
    readonly float[] _smoothed = new float[Dg5fReceiver.ChannelCount];
    bool _initialized;

    void Start()
    {
        _receiver = GetComponent<Dg5fReceiver>();
        _fingerIKs = GetComponents<Dg5fFingerIK>();
        _joints = new ArticulationBody[Dg5fReceiver.ChannelCount];

        var bySuffix = new Dictionary<string, ArticulationBody>();
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
        {
            if (ab.jointType != ArticulationJointType.RevoluteJoint) continue;
            int k = ab.name.IndexOf("_dg_");
            if (k < 0) continue; // 결합 로봇의 팔 관절 — 손 채널 매핑 대상 아님
            bySuffix[ab.name.Substring(k)] = ab;
        }

        int found = 0;
        for (int f = 1; f <= 5; f++)
            for (int j = 1; j <= 4; j++)
            {
                int idx = (f - 1) * 4 + (j - 1);
                if (bySuffix.TryGetValue($"_dg_{f}_{j}", out var ab))
                {
                    _joints[idx] = ab;
                    found++;
                }
                else
                    Debug.LogError($"[Dg5fHandDriver] 관절 못 찾음: _dg_{f}_{j}");
            }
        Debug.Log($"[Dg5fHandDriver] 관절 매핑 {found}/20, 포트 {_receiver.port} 수신 대기");
    }

    void FixedUpdate()
    {
        if (!enableTracking || _joints == null) return;
        if (!_receiver.GetAngles(_angles)) return;
        if (_receiver.secondsSinceLastPacket > staleTimeout) return;

        if (!_initialized)
        {
            // 첫 패킷: 현재 target에서 시작 (홱 움직임 방지)
            for (int i = 0; i < _joints.Length; i++)
                if (_joints[i] != null) _smoothed[i] = _joints[i].xDrive.target;
            _initialized = true;
        }

        float k = 1f - Mathf.Exp(-lerpSpeed * Time.fixedDeltaTime);
        // IK가 맡은 손가락 갱신 — Active는 IK가 매 틱 스스로 판정(패킷에 그 손가락 리치벡터가
        // 실제로 오는지까지 반영)하므로, 송신기가 구버전이면 자동으로 각도 방식이 살아난다.
        for (int f = 0; f < _ikOwned.Length; f++) _ikOwned[f] = false;
        if (_fingerIKs != null)
            foreach (var ik in _fingerIKs)
                if (ik != null && ik.Active && ik.fingerIndex >= 1 && ik.fingerIndex <= _ikOwned.Length)
                    _ikOwned[ik.fingerIndex - 1] = true;
        for (int i = 0; i < _joints.Length; i++)
        {
            if (_ikOwned[i / 4]) continue; // 이 손가락 4채널은 Dg5fFingerIK가 구동
            var ab = _joints[i];
            if (ab == null) continue;
            var d = ab.xDrive;
            float target = Mathf.Clamp(_angles[i], d.lowerLimit, d.upperLimit);
            _smoothed[i] = Mathf.Lerp(_smoothed[i], target, k);
            d.target = _smoothed[i];
            ab.xDrive = d;
        }
    }
}
