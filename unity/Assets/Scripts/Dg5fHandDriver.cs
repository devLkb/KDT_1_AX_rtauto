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
using System.Globalization;
using System.IO;
using System.Text;
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

    [Tooltip("켜면 20관절 전체의 [비전 라디안 vs 로봇 관절 라디안] + 수신→클램프→적용→실제 각도를 "
             + "0.5초마다 Console에 찍음(파이썬 로그와 대조용). IK 구동 중인 손가락은 제외. "
             + "디버깅용 임시 기본 ON — 다 쓰면 끌 것")]
    public bool debugThumbLog = true;   // 디버그 임시 기본 ON
    float _lastDbgLog;

    // 20채널 사람-관절 대응 이름(파이썬 dg5f_angles.CHANNEL_NAMES와 동일 순서) — 디버그 로그 라벨.
    static readonly string[] JointLabels = {
        "1_1 thumb_cmc(평면)", "1_2 thumb_opp(깊이)", "1_3 thumb_mcp", "1_4 thumb_ip",
        "2_1 index_abd", "2_2 index_mcp", "2_3 index_pip", "2_4 index_dip",
        "3_1 middle_abd", "3_2 middle_mcp", "3_3 middle_pip", "3_4 middle_dip",
        "4_1 ring_abd", "4_2 ring_mcp", "4_3 ring_pip", "4_4 ring_dip",
        "5_1 pinky_cmc", "5_2 pinky_lat", "5_3 pinky_mcp", "5_4 pinky_pip",
    };

    [Tooltip("켜면 매 FixedUpdate(50Hz)에 20관절의 [비전 라디안(v6 수신) / 로봇 관절 실제 라디안]을 "
             + "별도 CSV(Logs/rad_dg5f_*.csv)에 저장. 비전↔로봇 라디안 비교용")]
    public bool logRadiansToFile = true;
    readonly float[] _rawRadBuf = new float[Dg5fReceiver.RawRadCount];  // v6 비전 라디안 수신 버퍼
    StreamWriter _radW;
    int _radCount;

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

        if (logRadiansToFile)
        {
            _radW = Dg5fLogFile.Create("rad_dg5f", out string radPath);
            var sb = new StringBuilder("t_unix");
            for (int f = 1; f <= 5; f++)
                for (int j = 1; j <= 4; j++)
                    sb.Append($",vis_rad_{f}_{j},joint_rad_{f}_{j}");
            _radW.WriteLine(sb.ToString());
            Debug.Log($"[Dg5fHandDriver] 라디안 로그 시작: {radPath}");
        }
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
        // v6 비전 라디안 원값(매핑·필터 전 프록시 출력) — 있으면 로봇 관절 라디안과 비교/로깅
        bool hasRad = _receiver.GetRawRadians(_rawRadBuf);
        bool doDbg = debugThumbLog && (Time.time - _lastDbgLog >= 0.5f);
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

            if (doDbg)
            {
                // 20관절 전 구간 추적(IK 미구동 관절만 — IK 손가락은 위에서 continue):
                //   비전 라디안 = 파이썬 프록시 원값(사람 각) / 관절 라디안 = 로봇 실제 관절각
                //   수신 rx = 파이썬이 보낸 deg / clamp → 적용(스무딩) → 실제(관절 현재각)
                string jn = JointLabels[i];
                float jointRad = (ab.dofCount > 0) ? ab.jointPosition[0] : 0f;
                string visRad = hasRad ? _rawRadBuf[i].ToString("F4") : "N/A";
                Debug.Log($"[HandDriver {jn}] 비전 라디안={visRad} vs 관절 라디안={jointRad:F4}  |  "
                          + $"수신 rx={_angles[i]:F1} → clamp[{d.lowerLimit:F0},{d.upperLimit:F0}]={target:F1} → "
                          + $"적용={d.target:F1} → 실제={jointRad * Mathf.Rad2Deg:F1} (deg)");
            }
        }
        if (doDbg) _lastDbgLog = Time.time;

        // 라디안 로그파일: 20관절 [비전 라디안(v6 수신) / 로봇 관절 실제 라디안] 매 틱 기록
        if (_radW != null)
        {
            double t = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
            var sb = new StringBuilder(t.ToString("F3", CultureInfo.InvariantCulture));
            for (int i = 0; i < _joints.Length; i++)
            {
                float visRad = hasRad ? _rawRadBuf[i] : float.NaN;
                var ab = _joints[i];
                float jointRad = (ab != null && ab.dofCount > 0) ? ab.jointPosition[0] : float.NaN;
                sb.Append(',').Append(visRad.ToString("F5", CultureInfo.InvariantCulture));
                sb.Append(',').Append(jointRad.ToString("F5", CultureInfo.InvariantCulture));
            }
            _radW.WriteLine(sb.ToString());
            if (++_radCount % 100 == 0) _radW.Flush();
        }
    }

    void OnDestroy()
    {
        if (_radW != null) { _radW.Flush(); _radW.Dispose(); _radW = null; }
    }
}
