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
            int k = ab.name.IndexOf("_dg_");
            if (k < 0) continue; // 결합 로봇의 팔 관절(UR 등) — DG5F 채널 아님
            string s = ab.name.Substring(k + 4); // "f_j"
            int f = s[0] - '0';
            int j = s[2] - '0';
            _joints[(f - 1) * 4 + (j - 1)] = ab;
        }

        // 경로·중복 회피 규칙은 Dg5fLogFile이 소유 — 초 단위 + 접미사라 덮어쓰기 불가(2026-07-16)
        _w = Dg5fLogFile.Create("unity_dg5f", out string path);
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
