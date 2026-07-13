// SvhReceiver.cs
// 유니티에서 Python 비전 프로세스가 보내는 SVH 9개 관절 각도(UDP)를 수신.
// 패킷 포맷: float32 x 9, little-endian (Python struct '<9f'와 일치).
//
// 사용법: 빈 GameObject에 이 스크립트를 붙이면 됩니다.
// LatestAngles[0..8]에 최신 각도(rad)가 들어옵니다. 순서는 Python의
// CHANNEL_NAMES = SVH 드라이버 JTC(cfg/schunk_svh_driver.yaml) / SVHChannel
// enum 순서와 동일합니다. 소비자 스크립트에서 아래 순서대로 joint에 주입하세요:
//   [0] Thumb_Flexion         [1] Thumb_Opposition
//   [2] Index_Finger_Distal   [3] Index_Finger_Proximal
//   [4] Middle_Finger_Distal  [5] Middle_Finger_Proximal
//   [6] Ring_Finger           [7] Pinky
//   [8] Finger_Spread
// (dex-urdf joint 이름은 right_hand_ prefix, 드라이버는 Right_Hand_ prefix)

using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class SvhReceiver : MonoBehaviour
{
    public int port = 5005;
    public float[] LatestAngles = new float[9];

    private UdpClient _client;
    private Thread _thread;
    private volatile bool _running;
    private readonly object _lock = new object();

    void Start()
    {
        _client = new UdpClient(port);
        _running = true;
        _thread = new Thread(ReceiveLoop) { IsBackground = true };
        _thread.Start();
    }

    void ReceiveLoop()
    {
        IPEndPoint remote = new IPEndPoint(IPAddress.Any, port);
        while (_running)
        {
            try
            {
                byte[] data = _client.Receive(ref remote);
                if (data.Length >= 36) // 9 * 4 bytes
                {
                    float[] parsed = new float[9];
                    for (int i = 0; i < 9; i++)
                        parsed[i] = BitConverter.ToSingle(data, i * 4);
                    lock (_lock)
                    {
                        Array.Copy(parsed, LatestAngles, 9);
                    }
                }
            }
            catch (Exception e)
            {
                if (_running) Debug.LogWarning("SVH UDP recv: " + e.Message);
            }
        }
    }

    // 메인 스레드에서 안전하게 최신값 읽기
    public float[] GetAngles()
    {
        float[] copy = new float[9];
        lock (_lock) { Array.Copy(LatestAngles, copy, 9); }
        return copy;
    }

    void OnDestroy()
    {
        _running = false;
        _client?.Close();
        _thread?.Join(200);
    }
}
