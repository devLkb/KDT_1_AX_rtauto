// Unity 프로젝트에 그대로 추가해서 쓰는 UDP 좌표 수신 스크립트.
// 빈 GameObject에 이 스크립트를 붙이고 Port 값이 zed_sender.py의 UNITY_PORT와 같은지 확인하세요.
// 주의: 이 프로젝트에서 5005=SVH핸드, 5006=Dg5fReceiver(손가락 관절)가 이미 쓰고 있음 — 겹치지 않게 5007 사용.

using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

[Serializable]
public class DetectedObject
{
    public int id;
    public string label;
    public float x;
    public float y;
    public float z;
}

[Serializable]
public class DetectionPayload
{
    public DetectedObject[] objects;
}

public class ObjectCoordinateReceiver : MonoBehaviour
{
    public int port = 5007;

    private UdpClient udpClient;
    private Thread receiveThread;
    private volatile string latestJson;
    private readonly object lockObj = new object();

    void Start()
    {
        udpClient = new UdpClient(port);
        receiveThread = new Thread(ReceiveLoop) { IsBackground = true };
        receiveThread.Start();
    }

    void ReceiveLoop()
    {
        IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, port);
        while (true)
        {
            try
            {
                byte[] data = udpClient.Receive(ref anyIP);
                string json = Encoding.UTF8.GetString(data);
                lock (lockObj)
                {
                    latestJson = json;
                }
            }
            catch (SocketException)
            {
                // 소켓이 닫히면(OnApplicationQuit) 여기로 빠져나옴
                break;
            }
        }
    }

    void Update()
    {
        string json;
        lock (lockObj)
        {
            json = latestJson;
            latestJson = null;
        }

        if (string.IsNullOrEmpty(json))
        {
            return;
        }

        DetectionPayload payload = JsonUtility.FromJson<DetectionPayload>(json);
        if (payload?.objects == null)
        {
            return;
        }

        foreach (DetectedObject obj in payload.objects)
        {
            Vector3 worldPos = new Vector3(obj.x, obj.y, obj.z);
            // TODO: 여기서 worldPos를 실제 오브젝트 이동/스폰 등에 연결
            Debug.Log($"[{obj.label}] id={obj.id} pos={worldPos}");
        }
    }

    void OnApplicationQuit()
    {
        udpClient?.Close();
        receiveThread?.Join(200);
    }
}
