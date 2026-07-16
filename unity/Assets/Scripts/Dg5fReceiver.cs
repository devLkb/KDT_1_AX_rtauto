// Dg5fReceiver.cs
// Python 비전 프로세스(dg5f/vision_node_dg5f.py)가 보내는 DG5F 20관절 각도(UDP) 수신.
// 패킷: float32 little-endian. v1='<20f'(관절각[deg]) / v2='<24f'(+엄지끝 정규화좌표 3, 핀치 플래그 1)
//       / v3='<25f'(+엄지-검지 끝거리 비율 1)
//       / v4='<37f'(+검지·중지·약지·새끼 리치벡터 각 3 = 12, Python TIP_FINGERS 순서).
// 길이로 버전 판별 — 상위 버전 필드는 없으면 미제공 처리. 비교가 '>='라 상위 버전 패킷을
// 하위 버전만 아는 빌드에 쏴도 앞부분만 읽고 뒤는 무시된다(양방향 호환).
// 채널 순서(계약): [0..3]엄지 1_1~1_4 / [4..7]검지 2_1~2_4 / [8..11]중지 / [12..15]약지 / [16..19]새끼
// 포트 5006 — SVH(5005)와 공존.
// ⚠️ udp_test_receiver 같은 로컬 수신기가 같은 포트에 살아있으면 패킷을 뺏김 (SVH 포트 함정과 동일).

using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class Dg5fReceiver : MonoBehaviour
{
    public const int ChannelCount = 20;

    public int port = 5006;
    [Tooltip("마지막 패킷 수신 후 경과(초). 0.5 이상이면 송신 끊김.")]
    public float secondsSinceLastPacket = float.PositiveInfinity;

    /// v4 리치벡터를 싣는 손가락 수 (검지·중지·약지·새끼 — 엄지는 v2 필드에 따로 있음)
    public const int FingerTipCount = 4;

    readonly float[] _latest = new float[ChannelCount];
    // v2 패킷(24f): 20 관절각 + 엄지끝 정규화좌표(ex,ey,ez) + 핀치 플래그
    // v3 패킷(25f): + 엄지-검지 끝거리 비율(손길이 정규화, 연속값) — 핀치 연속 블렌딩용
    readonly float[] _tip = new float[4];
    // v4 패킷(37f): + 검지→새끼 리치벡터 4×3. 엄지와 같은 해부학 축·같은 '펴짐 비율' 의미.
    readonly float[] _fingerTips = new float[FingerTipCount * 3];
    float _pinchDist;
    volatile bool _hasData;
    volatile bool _hasTip;
    volatile bool _hasPinchDist;
    volatile bool _hasFingerTips;
    long _lastPacketUtcTicks; // 수신 스레드에서 Unity Time API 사용 불가 → DateTime 사용

    UdpClient _client;
    Thread _thread;
    volatile bool _running;
    readonly object _lock = new object();

    public bool HasData => _hasData;

    void Start()
    {
        _client = new UdpClient(port);
        _running = true;
        _thread = new Thread(ReceiveLoop) { IsBackground = true };
        _thread.Start();
    }

    void Update()
    {
        long ticks = Interlocked.Read(ref _lastPacketUtcTicks);
        secondsSinceLastPacket = _hasData
            ? (float)(DateTime.UtcNow - new DateTime(ticks, DateTimeKind.Utc)).TotalSeconds
            : float.PositiveInfinity;
    }

    void ReceiveLoop()
    {
        var remote = new IPEndPoint(IPAddress.Any, port);
        while (_running)
        {
            try
            {
                byte[] data = _client.Receive(ref remote);
                if (data.Length < ChannelCount * 4) continue;
                bool v2 = data.Length >= (ChannelCount + 4) * 4;
                bool v3 = data.Length >= (ChannelCount + 5) * 4;
                bool v4 = data.Length >= (ChannelCount + 5 + FingerTipCount * 3) * 4;
                lock (_lock)
                {
                    for (int i = 0; i < ChannelCount; i++)
                        _latest[i] = BitConverter.ToSingle(data, i * 4);
                    if (v2)
                        for (int i = 0; i < 4; i++)
                            _tip[i] = BitConverter.ToSingle(data, (ChannelCount + i) * 4);
                    if (v3)
                        _pinchDist = BitConverter.ToSingle(data, (ChannelCount + 4) * 4);
                    if (v4)
                        for (int i = 0; i < FingerTipCount * 3; i++)
                            _fingerTips[i] = BitConverter.ToSingle(data, (ChannelCount + 5 + i) * 4);
                }
                Interlocked.Exchange(ref _lastPacketUtcTicks, DateTime.UtcNow.Ticks);
                _hasData = true;
                if (v2) _hasTip = true;
                if (v3) _hasPinchDist = true;
                if (v4) _hasFingerTips = true;
            }
            catch (Exception e)
            {
                if (_running) Debug.LogWarning("[Dg5fReceiver] " + e.Message);
            }
        }
    }

    /// 메인 스레드에서 최신 20채널 각도[deg]를 buffer에 복사. 데이터 없으면 false.
    public bool GetAngles(float[] buffer)
    {
        if (!_hasData) return false;
        lock (_lock) { Array.Copy(_latest, buffer, ChannelCount); }
        return true;
    }

    /// v2 패킷의 엄지끝 목표(정규화 해부학 좌표)와 핀치 플래그. v2 미수신이면 false.
    public bool GetThumbTip(out Vector3 tipNormalized, out bool pinch)
    {
        tipNormalized = Vector3.zero;
        pinch = false;
        if (!_hasTip) return false;
        lock (_lock)
        {
            tipNormalized = new Vector3(_tip[0], _tip[1], _tip[2]);
            pinch = _tip[3] > 0.5f;
        }
        return true;
    }

    /// 손가락 리치벡터(정규화 해부학 좌표) — fingerIndex 1=엄지(v2 필드), 2~5=검지~새끼(v4 필드).
    /// 해당 필드 미수신이면 false → 호출자는 그 손가락을 각도 방식으로 폴백해야 한다.
    /// 엄지를 1로 넘기면 GetThumbTip과 같은 값을 준다(핀치 플래그는 별도 조회).
    public bool GetFingerTip(int fingerIndex, out Vector3 tipNormalized)
    {
        tipNormalized = Vector3.zero;
        if (fingerIndex == 1) return GetThumbTip(out tipNormalized, out _);
        if (!_hasFingerTips || fingerIndex < 2 || fingerIndex > 1 + FingerTipCount) return false;
        lock (_lock)
        {
            int o = (fingerIndex - 2) * 3;
            tipNormalized = new Vector3(_fingerTips[o], _fingerTips[o + 1], _fingerTips[o + 2]);
        }
        return true;
    }

    /// v3 패킷의 엄지-검지 끝거리 비율(손길이 정규화). v3 미수신이면 false — 이진 핀치로 폴백할 것.
    public bool GetPinchDistance(out float distanceRatio)
    {
        distanceRatio = float.PositiveInfinity;
        if (!_hasPinchDist) return false;
        lock (_lock) { distanceRatio = _pinchDist; }
        return true;
    }

    void OnDestroy()
    {
        _running = false;
        _client?.Close();
        _thread?.Join(200);
    }
}
