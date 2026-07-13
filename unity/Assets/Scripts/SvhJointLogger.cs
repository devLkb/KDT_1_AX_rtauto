using System;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

/// <summary>
/// Phase 3 검증용: SVH 9관절의 (UDP 수신값, xDrive.target, 실제 jointPosition)을
/// 매 FixedUpdate마다 CSV로 기록한다. Python vision_log.csv와 unix time으로 정렬해
/// "흔들림이 비전 노이즈인지 / 필터 문제인지 / 유니티 물리 문제인지"를 분리 진단.
///
/// 사용법: ur5e_svh 루트(SvhReceiver 옆)에 부착. record 체크 시 Play 진입에 자동 기록.
/// 출력: 프로젝트 루트의 Logs/unity_joint_log.csv (환경 독립적, 자동 생성).
///       열: t_unix, rx/tgt/act x 9ch, 단위=도. 채널 순서 = 패킷 순서(SvhHandDriver와 동일).
/// </summary>
[RequireComponent(typeof(SvhReceiver))]
public class SvhJointLogger : MonoBehaviour
{
    public bool record = true;

    // 프로젝트 루트(Assets의 상위)의 Logs 폴더. 어느 PC에서 열어도 유효.
    // (이전엔 개발 PC 절대경로가 박혀 있어 다른 환경에서 Play 시 예외 발생)
    static string DefaultLogPath =>
        Path.GetFullPath(Path.Combine(Application.dataPath, "..", "Logs", "unity_joint_log.csv"));

    // 패킷 인덱스 -> child link 이름 (SvhHandDriver와 동일 순서 유지)
    static readonly string[] Links =
    {
        "right_hand_a", "right_hand_z", "right_hand_p", "right_hand_l",
        "right_hand_o", "right_hand_k", "right_hand_j", "right_hand_i",
        "right_hand_virtual_i",
    };
    static readonly string[] Names =
    {
        "thmFlex", "thmOpp", "idxDist", "idxProx",
        "midDist", "midProx", "ring", "pinky", "spread",
    };

    private SvhReceiver _rx;
    private ArticulationBody[] _bodies = new ArticulationBody[9];
    private StreamWriter _writer;
    private readonly StringBuilder _sb = new StringBuilder(256);
    private int _linesSinceFlush;

    void Start()
    {
        _rx = GetComponent<SvhReceiver>();
        foreach (var b in GetComponentsInChildren<ArticulationBody>())
        {
            int idx = Array.IndexOf(Links, b.name);
            if (idx >= 0) _bodies[idx] = b;
        }

        if (!record) return;

        string path = DefaultLogPath;
        try
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            _writer = new StreamWriter(path, false, Encoding.UTF8);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[SvhJointLogger] 로그 파일을 열 수 없어 기록을 건너뜁니다: {path}\n{e.Message}");
            _writer = null;
            return;
        }

        _sb.Append("t_unix");
        foreach (var n in Names) _sb.Append(",rx_").Append(n);
        foreach (var n in Names) _sb.Append(",tgt_").Append(n);
        foreach (var n in Names) _sb.Append(",act_").Append(n);
        _writer.WriteLine(_sb.ToString());
        Debug.Log($"[SvhJointLogger] 기록 시작: {path}");
    }

    void FixedUpdate()
    {
        if (_writer == null) return;

        float[] rx = _rx.GetAngles(); // rad
        double t = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

        _sb.Clear();
        _sb.Append(t.ToString("F3", CultureInfo.InvariantCulture));
        for (int i = 0; i < 9; i++)
            _sb.Append(',').Append((rx[i] * Mathf.Rad2Deg).ToString("F3", CultureInfo.InvariantCulture));
        for (int i = 0; i < 9; i++)
            _sb.Append(',').Append((_bodies[i] != null ? _bodies[i].xDrive.target : 0f).ToString("F3", CultureInfo.InvariantCulture));
        for (int i = 0; i < 9; i++)
        {
            // jointPosition은 rad (revolute), 도로 변환해 기록
            float act = _bodies[i] != null && _bodies[i].jointPosition.dofCount > 0
                ? _bodies[i].jointPosition[0] * Mathf.Rad2Deg : 0f;
            _sb.Append(',').Append(act.ToString("F3", CultureInfo.InvariantCulture));
        }
        _writer.WriteLine(_sb.ToString());

        if (++_linesSinceFlush >= 100) { _writer.Flush(); _linesSinceFlush = 0; }
    }

    void OnDestroy()
    {
        if (_writer != null)
        {
            _writer.Flush();
            _writer.Close();
            _writer = null;
            Debug.Log("[SvhJointLogger] 기록 종료");
        }
    }
}
