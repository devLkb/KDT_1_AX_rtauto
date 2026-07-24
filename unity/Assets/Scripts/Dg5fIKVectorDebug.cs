// Dg5fIKVectorDebug.cs  (구 Dg5fWristVectorDebug/Dg5fRootMarker — GUID 동일해 프리팹 부착 유지)
// 손가락별 IK 벡터(앵커→base 목표)를 선으로 그린다 — **활성 ikMode를 자동 반영**.
// 산식을 복제하지 않고 각 Dg5fFingerIK가 이번 틱에 실제로 계산한 DebugAnchor/DebugBaseTarget을
// 읽는다. 그래서 모드별 시작점이 항상 맞는다:
//   AnatomicalReach   = 가상앵커(n_1+reachOffset) → 목표
//   RobotRootTipVector= 로봇 손목(palm 원점) → 목표   (사람 손목→tip 벡터 대응)
//   ChainRatioReach   = n_2 피벗 → 목표               (사람 landmark 1·5·9·13·17→tip 대응)
// v5 폴백 등 예외 경로도 ComputeTarget*가 세팅한 실제 앵커를 따라간다(시각화≡계산 보장).
// 선 끝 = 노랑 마커(base 목표, 핀치 블렌딩 전)와 항상 일치해야 정상.
// 런타임 전용 LineRenderer(콜라이더 없음, Play 종료 시 자동 소멸 — 씬에 저장되지 않음).

using UnityEngine;

public class Dg5fIKVectorDebug : MonoBehaviour
{
    [Tooltip("손가락별 IK 벡터(앵커→base 목표)를 선으로 표시 — 활성 ikMode의 앵커를 자동 반영")]
    public bool show = true;

    [Tooltip("선 굵기(m)")]
    public float lineWidth = 0.002f;

    // 손끝 마커(노랑/빨강/초록)와 겹치지 않는 색 — 엄지부터 새끼 순서
    static readonly Color[] FingerColors =
    {
        new Color(1f, 0f, 1f),      // f1 엄지: 마젠타
        new Color(0f, 1f, 1f),      // f2 검지: 시안
        new Color(0.2f, 0.5f, 1f),  // f3 중지: 파랑
        new Color(1f, 0.5f, 0f),    // f4 약지: 주황
        Color.white,                // f5 새끼: 흰색
    };
    static readonly string[] FingerNames = { "thumb", "index", "middle", "ring", "pinky" };

    Dg5fFingerIK[] _fingers;                            // 배열 순서≠fingerIndex — f.fingerIndex로 식별
    readonly LineRenderer[] _lines = new LineRenderer[5];

    void Start()
    {
        _fingers = GetComponents<Dg5fFingerIK>();
        if (_fingers.Length == 0)
            Debug.LogWarning("[Dg5fIKVectorDebug] 같은 GameObject에 Dg5fFingerIK가 없음 — 선 없음");
    }

    void Update()
    {
        if (_fingers == null) return;
        foreach (var f in _fingers)
        {
            if (f == null || f.fingerIndex < 1 || f.fingerIndex > 5) continue;
            int i = f.fingerIndex - 1;
            // IK 비활성(패킷 끊김·미수신·enableIK off)이면 FingerIK가 Active=false — 선 숨김
            if (!show || !f.Active)
            {
                if (_lines[i] != null) _lines[i].gameObject.SetActive(false);
                continue;
            }
            var line = _lines[i];
            if (line == null) line = _lines[i] = MakeLine(f.fingerIndex);
            line.gameObject.SetActive(true);
            line.SetPosition(0, f.DebugAnchor);       // 이번 틱 활성 방식의 실제 시작점
            line.SetPosition(1, f.DebugBaseTarget);   // base 목표(노랑 마커와 동일점)
            line.startWidth = line.endWidth = lineWidth;
        }
    }

    LineRenderer MakeLine(int fingerIndex)
    {
        var go = new GameObject("DBG_IKVec_f" + fingerIndex + " (" + FingerNames[fingerIndex - 1] + ")");
        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = true;
        lr.positionCount = 2;
        var sh = Shader.Find("Unlit/Color");
        var mat = new Material(sh != null ? sh : Shader.Find("Standard"));
        mat.color = FingerColors[fingerIndex - 1];
        lr.material = mat;
        lr.startWidth = lr.endWidth = lineWidth;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
        return lr;
    }

    void OnDestroy()
    {
        foreach (var l in _lines)
            if (l != null) Destroy(l.gameObject);
    }
}
