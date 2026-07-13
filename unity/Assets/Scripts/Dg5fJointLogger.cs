// Dg5fJointLogger.cs
// DG5F 텔레옵 검증용 관절 로거 — Play 동안 매 FixedUpdate(50Hz)에
// [수신값 rx / 드라이브 목표 tgt / 실측 관절각 act] 20관절을 CSV 기록.
// 타임스탬프는 unix 초(UTC) — Python 비전 로그(time.time())와 직접 시간 정렬 가능.
//
// 출력: <프로젝트>/Logs/unity_dg5f_YYYYMMDD_HHmm.csv (Play마다 새 파일)
// 로봇 루트에 부착 (Dg5fReceiver 옆). 분석: KDT/dg5f/analyze_teleop.py

using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

public class Dg5fJointLogger : MonoBehaviour
{
    [Tooltip("N샘플마다 디스크 flush")]
    public int flushEverySamples = 100;

    StreamWriter _w;
    ArticulationBody[] _joints;
    Dg5fReceiver _rx;
    readonly float[] _rxBuf = new float[Dg5fReceiver.ChannelCount];
    int _count;

    void Start()
    {
        _rx = GetComponent<Dg5fReceiver>();
        _joints = new ArticulationBody[Dg5fReceiver.ChannelCount];
        foreach (var ab in GetComponentsInChildren<ArticulationBody>())
        {
            if (ab.jointType != ArticulationJointType.RevoluteJoint) continue;
            string s = ab.name.Substring(ab.name.IndexOf("_dg_") + 4); // "f_j"
            int f = s[0] - '0';
            int j = s[2] - '0';
            _joints[(f - 1) * 4 + (j - 1)] = ab;
        }

        string dir = Path.Combine(Application.dataPath, "../Logs");
        Directory.CreateDirectory(dir);
        string path = Path.Combine(dir, "unity_dg5f_" + DateTime.Now.ToString("yyyyMMdd_HHmm") + ".csv");
        _w = new StreamWriter(path, false, Encoding.UTF8);
        var sb = new StringBuilder("t_unix");
        for (int f = 1; f <= 5; f++)
            for (int j = 1; j <= 4; j++)
                sb.Append($",rx_{f}_{j},tgt_{f}_{j},act_{f}_{j}");
        _w.WriteLine(sb.ToString());
        Debug.Log($"[Dg5fJointLogger] 기록 시작: {path}");
    }

    void FixedUpdate()
    {
        if (_w == null) return;
        bool has = _rx != null && _rx.GetAngles(_rxBuf);
        double t = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
        var sb = new StringBuilder(t.ToString("F3", CultureInfo.InvariantCulture));
        for (int i = 0; i < _joints.Length; i++)
        {
            var ab = _joints[i];
            float rx = has ? _rxBuf[i] : float.NaN;
            sb.Append(',').Append(rx.ToString("F2", CultureInfo.InvariantCulture));
            sb.Append(',').Append((ab ? ab.xDrive.target : float.NaN).ToString("F2", CultureInfo.InvariantCulture));
            sb.Append(',').Append((ab ? ab.jointPosition[0] * Mathf.Rad2Deg : float.NaN).ToString("F2", CultureInfo.InvariantCulture));
        }
        _w.WriteLine(sb.ToString());
        if (++_count % flushEverySamples == 0) _w.Flush();
    }

    void OnDestroy()
    {
        if (_w != null) { _w.Flush(); _w.Dispose(); _w = null; }
    }
}
