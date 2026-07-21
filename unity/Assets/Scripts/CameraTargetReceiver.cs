// CameraTargetReceiver.cs
// 외부 3D 카메라 프로세스가 UDP로 보내는 타겟 좌표를 받아, 로봇팔(robotBase) 기준
// 로컬 좌표계 위치에 빨간 타겟 공(target)을 배치한다.
// README의 "3D 카메라 목표 좌표(후속 입력 경계)"를 구현하는 부분 — 학습 중에는
// Dg5fGraspPointReachAgent.ResetTarget()이 같은 target을 랜덤으로 옮기므로, 이 리시버는
// 학습이 아닌 라이브 운용(카메라로 실제 목표를 인식하는 모드)에서만 사용한다.
// 패킷: float32 little-endian '<3f'.
//   inputIsCameraSpace=true(기본)  → 카메라 자체 좌표계 기준 (x, y, z)[m].
//     cameraTransform(캘리브레이션된 카메라 위치/방향)로 월드 좌표를 구한 뒤 robotBase
//     기준으로 다시 변환한다. cameraAxisSign으로 카메라 SDK 축 관례(OpenCV 등은 보통
//     Y-down/Z-forward)를 Unity(Y-up) 관례로 보정한다.
//   inputIsCameraSpace=false        → 이미 robotBase 로컬 좌표로 계산되어 오는 값(기존 동작).
// vision/dg5f의 UDP 수신 패턴(Dg5fReceiver.cs)과 동일한 구조.
// 포트 5007 — SVH(5005)·DG5F 손(5006)과 공존.

using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class CameraTargetReceiver : MonoBehaviour
{
    public int port = 5007;

    [Tooltip("좌표계 원점 — 로봇팔 베이스 Transform")]
    public Transform robotBase;
    [Tooltip("이동시킬 타겟 오브젝트(빨간 공) — Reach 학습 씬의 Target/RedBall과 동일 오브젝트")]
    public Transform target;

    [Tooltip("true: 수신 좌표를 cameraTransform 기준 카메라 로컬 좌표로 해석. " +
             "false: 수신 좌표가 이미 robotBase 로컬 좌표(기존 동작).")]
    public bool inputIsCameraSpace = true;
    [Tooltip("실제 카메라 위치/방향에 맞게 캘리브레이션된 Transform. inputIsCameraSpace=true일 때 필수.")]
    public Transform cameraTransform;
    [Tooltip("카메라 SDK 축 관례 보정 — 예: OpenCV/RealSense(X-right, Y-down, Z-forward) → " +
             "Unity(X-right, Y-up, Z-forward)는 Y만 뒤집으면 되므로 기본값 (1,-1,1).")]
    public Vector3 cameraAxisSign = new Vector3(1f, -1f, 1f);

    [Tooltip("target을 로봇팔 작업반경 안으로 강제 클램프할지 여부 (카메라 오검출 안전망)")]
    public bool clampToWorkspace = true;
    [Tooltip("Dg5fReachSpec.MinimumTargetRadius와 동일 기본값")]
    public float minRadius = 0.20f;
    [Tooltip("Dg5fReachSpec.MaximumTargetRadius와 동일 기본값")]
    public float maxRadius = 0.85f;

    [Tooltip("마지막 패킷 수신 후 경과(초). 0.5 이상이면 카메라 신호 끊김.")]
    public float secondsSinceLastPacket = float.PositiveInfinity;

    const int ChannelCount = 3;
    readonly float[] _latest = new float[ChannelCount];
    volatile bool _hasData;
    long _lastPacketUtcTicks;

    UdpClient _client;
    Thread _thread;
    volatile bool _running;
    readonly object _lock = new object();

    public bool HasData => _hasData;

    void Start()
    {
        if (robotBase == null || target == null)
        {
            Debug.LogError("[CameraTargetReceiver] robotBase, target을 Inspector에서 지정해야 함.");
            enabled = false;
            return;
        }
        if (inputIsCameraSpace && cameraTransform == null)
        {
            Debug.LogError("[CameraTargetReceiver] inputIsCameraSpace=true인데 cameraTransform이 없음.");
            enabled = false;
            return;
        }
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

        if (!TryGetLocalPosition(out Vector3 raw)) return;

        Vector3 robotLocal = inputIsCameraSpace
            ? robotBase.InverseTransformPoint(
                cameraTransform.TransformPoint(Vector3.Scale(raw, cameraAxisSign)))
            : raw;

        if (clampToWorkspace) robotLocal = ClampToWorkspace(robotLocal);
        target.position = robotBase.TransformPoint(robotLocal);
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
                lock (_lock)
                {
                    for (int i = 0; i < ChannelCount; i++)
                        _latest[i] = BitConverter.ToSingle(data, i * 4);
                }
                Interlocked.Exchange(ref _lastPacketUtcTicks, DateTime.UtcNow.Ticks);
                _hasData = true;
            }
            catch (Exception e)
            {
                if (_running) Debug.LogWarning("[CameraTargetReceiver] " + e.Message);
            }
        }
    }

    /// 메인 스레드에서 최신 수신 좌표(원본, 미변환)를 읽는다. 데이터 없으면 false.
    bool TryGetLocalPosition(out Vector3 raw)
    {
        raw = Vector3.zero;
        if (!_hasData) return false;
        lock (_lock) { raw = new Vector3(_latest[0], _latest[1], _latest[2]); }
        return true;
    }

    /// 평면(XZ) 반경만 [minRadius, maxRadius]로 클램프 — 방향은 유지, 높이(Y)는 그대로 둔다.
    Vector3 ClampToWorkspace(Vector3 local)
    {
        float planarRadius = new Vector2(local.x, local.z).magnitude;
        if (planarRadius < 1e-6f) return local;
        float clampedRadius = Mathf.Clamp(planarRadius, minRadius, maxRadius);
        float scale = clampedRadius / planarRadius;
        return new Vector3(local.x * scale, local.y, local.z * scale);
    }

    void OnDestroy()
    {
        _running = false;
        _client?.Close();
        _thread?.Join(200);
    }
}
