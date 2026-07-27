using System;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// 라이브 데모(Pipeline_Demo)에서 로봇이 접근할 물체를 여러 후보 중 하나로 전환한다.
    /// 후보 중 활성화된(active) 물체는 항상 하나뿐이고, 나머지는 비활성 상태로 대기한다.
    /// 전환 시 Dg5fGraspAgent.SetActiveTarget()을 호출해 ball 참조/콘택트 센서를 갈아끼우고
    /// 새 에피소드를 시작한다.
    /// </summary>
    public sealed class GraspTargetSwitcher : MonoBehaviour
    {
        public Dg5fGraspAgent agent;
        public Rigidbody[] targets = Array.Empty<Rigidbody>();
        public string[] targetLabels = Array.Empty<string>();

        [Tooltip("Game 뷰에 물체 전환 UI(OnGUI)를 그릴지 여부")]
        public bool showModeUI = true;

        [Tooltip("크기 슬라이더 배율 범위 — 1.0x = 씬 빌드 시 크기(기본값)")]
        public float minScaleMultiplier = 0.3f;
        public float maxScaleMultiplier = 3f;

        int _activeIndex;
        Vector3[] _baseScales;
        float[] _scaleMultipliers;

        public int ActiveIndex => _activeIndex;

        void Awake()
        {
            CacheBaseScales();
        }

        void CacheBaseScales()
        {
            int count = targets != null ? targets.Length : 0;
            _baseScales = new Vector3[count];
            _scaleMultipliers = new float[count];
            for (int i = 0; i < count; i++)
            {
                _baseScales[i] = targets[i] != null ? targets[i].transform.localScale : Vector3.one;
                _scaleMultipliers[i] = 1f;
            }
        }

        public void SetActiveTarget(int index)
        {
            if (agent == null || targets == null || index < 0 || index >= targets.Length) return;
            if (_baseScales == null || _baseScales.Length != targets.Length) CacheBaseScales();
            for (int i = 0; i < targets.Length; i++)
                if (targets[i] != null) targets[i].gameObject.SetActive(i == index);
            _activeIndex = index;
            agent.SetActiveTarget(targets[index]);
        }

        /// 지금 활성화된 물체의 크기를 씬 빌드 시 크기 기준 배율로 바꾼다(예: 1.5 = 150%).
        public void SetActiveTargetScale(float multiplier)
        {
            if (targets == null || _activeIndex < 0 || _activeIndex >= targets.Length) return;
            Rigidbody active = targets[_activeIndex];
            if (active == null) return;
            if (_baseScales == null || _baseScales.Length != targets.Length) CacheBaseScales();

            multiplier = Mathf.Clamp(multiplier, minScaleMultiplier, maxScaleMultiplier);
            _scaleMultipliers[_activeIndex] = multiplier;
            active.transform.localScale = _baseScales[_activeIndex] * multiplier;
        }

        void OnGUI()
        {
            if (!showModeUI || targets == null || targets.Length == 0) return;

            GUILayout.BeginArea(new Rect(10, 210, 260, 90 + targets.Length * 25), GUI.skin.box);
            GUILayout.Label("집을 물체");
            for (int i = 0; i < targets.Length; i++)
            {
                string label = i < targetLabels.Length && !string.IsNullOrEmpty(targetLabels[i])
                    ? targetLabels[i]
                    : (targets[i] != null ? targets[i].name : $"Target {i}");
                if (GUILayout.Button(i == _activeIndex ? $"[{label}]" : label))
                    SetActiveTarget(i);
            }

            if (_scaleMultipliers != null && _activeIndex < _scaleMultipliers.Length)
            {
                GUILayout.Label($"크기: {_scaleMultipliers[_activeIndex]:F2}x");
                float next = GUILayout.HorizontalSlider(
                    _scaleMultipliers[_activeIndex], minScaleMultiplier, maxScaleMultiplier);
                if (!Mathf.Approximately(next, _scaleMultipliers[_activeIndex]))
                    SetActiveTargetScale(next);
            }
            GUILayout.EndArea();
        }
    }
}
