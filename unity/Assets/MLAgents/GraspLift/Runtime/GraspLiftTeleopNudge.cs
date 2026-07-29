using UnityEngine;

namespace KDT.GraspLiftTraining
{
    /// <summary>
    /// GraspLift 라이브 데모의 수동(텔레옵) 모드에서 사람이 팔을 조종하게 해준다 —
    /// reach 데모의 ArmTeleopNudge와 같은 조작감(높이 슬라이더 + 화면 조이스틱, ArmTargetIK로
    /// 위치 유지)이지만 두 가지가 다르다:
    /// 1) reach 쪽은 agent.IsArmLocked(정책이 도달 후 스스로 잠그는 상태)를 폴링해서 켜지지만,
    ///    여기는 GraspLiftControlModeSwitcher가 SetActive()를 직접 호출한다 — 정책이 처음부터
    ///    끝까지 자동으로 처리하는 GraspLift에는 "도달 후 lock" 같은 중간 상태가 없기 때문.
    /// 2) 사람이 블록까지 처음부터 이동해야 하므로 범위가 nudge 수준(±10cm)이 아니라 스폰
    ///    반경 전체(0.35~0.55m)를 덮도록 넓다.
    /// 3) 손목을 수평으로 강제하지 않는다 — 자동 정책은 실제로 손을 살짝 옆으로 기울여서
    ///    블록을 잡는데(측면 파지), 손목을 평탄화로 고정하면 그 기울임 자체가 불가능해져서
    ///    좁은 블록을 못 잡게 된다. 그래서 positionJointCount를 -1로 둬서 손목 3관절도 CCD
    ///    위치 풀이에 참여시키고, 결과 방향은 IK가 자연스럽게 정하도록 둔다.
    /// Dg5fGraspLiftAgent는 _episodeActive가 꺼지면 스스로 xDrive를 전혀 건드리지 않으므로
    /// (reach의 ExternalArmControl 같은) 별도 상호배제 플래그가 필요 없다.
    /// </summary>
    public sealed class GraspLiftTeleopNudge : MonoBehaviour
    {
        public Dg5fGraspLiftAgent agent;
        public ArmTargetIK armIK;
        public HandSliderUI armSliderUI;

        [Tooltip("Game 뷰에 조절 UI(OnGUI)를 그릴지 여부")]
        public bool showControlUI = true;
        [Tooltip("기준점 위/아래로 조절 가능한 범위[m] — 스폰 반경 전체를 덮도록 nudge보다 크다")]
        public float maxHeightOffset = 0.5f;
        [Tooltip("기준점 기준 수평(로봇 기준 좌우/전후)으로 조절 가능한 최대 반경[m]")]
        public float maxHorizontalOffset = 0.6f;
        [Tooltip("조이스틱을 끝까지 밀었을 때 수평 이동 속도[m/s]")]
        public float horizontalMoveSpeed = 0.05f;

        const float JoystickSize = 110f;
        const float JoystickKnobSize = 26f;

        Transform _ikTarget;
        Vector3 _basePosition;
        float _heightOffset;
        Vector2 _horizontalOffset; // (right, forward)
        Vector2 _joystickInput;    // -1..1 per axis, read in FixedUpdate
        bool _dragging;
        bool _active;

        public bool IsActive => _active;

        void Awake()
        {
            if (agent == null) agent = GetComponent<Dg5fGraspLiftAgent>();
            if (armIK == null) armIK = GetComponent<ArmTargetIK>();
            if (armSliderUI == null) armSliderUI = GetComponent<HandSliderUI>();

            // ArmTargetIK/HandSliderUI는 원래 사람이 직접 조작하려고 만든 컴포넌트라 자체 OnGUI
            // 패널이 있다 — 이 스크립트가 대신 조작하므로 그 패널들은 항상 숨긴다.
            if (armIK != null) armIK.showUI = false;
            if (armSliderUI != null) armSliderUI.showUI = false;

            _ikTarget = new GameObject("GraspLiftTeleopNudgeTarget").transform;
        }

        void FixedUpdate()
        {
            if (!_active) return;

            Vector3 up = agent.robotBase != null ? agent.robotBase.up : Vector3.up;
            Vector3 right = agent.robotBase != null ? agent.robotBase.right : Vector3.right;
            Vector3 forward = agent.robotBase != null ? agent.robotBase.forward : Vector3.forward;

            if (_joystickInput.sqrMagnitude > 1e-6f)
            {
                Vector2 delta = _joystickInput * horizontalMoveSpeed * Time.fixedDeltaTime;
                _horizontalOffset += delta;
                if (_horizontalOffset.magnitude > maxHorizontalOffset)
                    _horizontalOffset = _horizontalOffset.normalized * maxHorizontalOffset;
            }

            _ikTarget.position = _basePosition
                + up * _heightOffset
                + right * _horizontalOffset.x
                + forward * _horizontalOffset.y;
        }

        /// GraspLiftControlModeSwitcher가 자동/수동 전환 시 직접 호출한다.
        public void SetActive(bool active)
        {
            _active = active;
            if (active)
            {
                Transform endEffector = armIK != null ? armIK.endEffector : null;
                if (endEffector == null && agent != null) endEffector = agent.graspPoint;
                _basePosition = endEffector != null ? endEffector.position : transform.position;
                _heightOffset = 0f;
                _horizontalOffset = Vector2.zero;
                _joystickInput = Vector2.zero;
                _dragging = false;
                _ikTarget.position = _basePosition;

                if (armSliderUI != null)
                {
                    armSliderUI.driveHandJoints = false;
                    // 켜기 직전 실제 자세로 재동기화해야 팔이 튀지 않는다.
                    armSliderUI.ResyncArmValuesFromCurrentPose();
                    armSliderUI.enabled = true;
                }
                if (armIK != null)
                {
                    armIK.target = _ikTarget;
                    armIK.enableIK = true;
                    // 조이스틱이 타겟을 느리게 계속 옮기는 상황에서는 도달/정체 안전장치가
                    // "다 왔다"로 오판해 멈췄다 확 움직이는 끊김을 만든다 — 텔레옵 중엔 끈다.
                    armIK.ignoreArrivalAndStallGating = true;
                    // 손목을 특정 각도로 강제하지 않는다 — 6관절 전부 위치 CCD에 참여시켜
                    // 손이 자연스러운(필요하면 기운) 각도로 다가가게 둔다.
                    armIK.positionJointCount = -1;
                    armIK.enabled = true;
                }
            }
            else
            {
                if (armIK != null)
                {
                    armIK.enabled = false;
                    armIK.ignoreArrivalAndStallGating = false;
                    armIK.positionJointCount = -1;
                }
                if (armSliderUI != null) armSliderUI.enabled = false;
            }
        }

        void OnGUI()
        {
            if (!showControlUI || !_active) return;

            GUILayout.BeginArea(new Rect(Screen.width - 260, Screen.height - 210, 250, 120), GUI.skin.box);
            GUILayout.Label($"손 높이: {_heightOffset * 100f:+0.0;-0.0}cm");
            float nextHeight = GUILayout.HorizontalSlider(_heightOffset, -maxHeightOffset, maxHeightOffset);
            if (!Mathf.Approximately(nextHeight, _heightOffset)) _heightOffset = nextHeight;
            GUILayout.Label($"수평 이동: ({_horizontalOffset.x * 100f:F1}, {_horizontalOffset.y * 100f:F1})cm");
            GUILayout.EndArea();

            DrawJoystick();
        }

        void DrawJoystick()
        {
            var joyRect = new Rect(Screen.width - 260 + 62f, Screen.height - 210 - JoystickSize - 10f,
                JoystickSize, JoystickSize);
            GUI.Box(joyRect, "이동 (드래그)");
            Vector2 center = new Vector2(joyRect.x + joyRect.width / 2f, joyRect.y + joyRect.height / 2f);
            float maxRadius = joyRect.width / 2f - JoystickKnobSize / 2f;

            Event e = Event.current;
            if (e.type == EventType.MouseDown && joyRect.Contains(e.mousePosition))
                _dragging = true;
            else if (e.type == EventType.MouseUp)
                _dragging = false;

            Vector2 knobOffset = Vector2.zero;
            if (_dragging)
            {
                knobOffset = (Vector2)e.mousePosition - center;
                if (knobOffset.magnitude > maxRadius) knobOffset = knobOffset.normalized * maxRadius;
            }
            // 화면 아래쪽(+y)으로 드래그하면 로봇 기준 앞쪽(+z)으로 가도록 y축 부호를 뒤집는다.
            _joystickInput = _dragging
                ? new Vector2(knobOffset.x / maxRadius, -knobOffset.y / maxRadius)
                : Vector2.zero;

            Vector2 knobPos = center + knobOffset;
            GUI.Box(new Rect(knobPos.x - JoystickKnobSize / 2f, knobPos.y - JoystickKnobSize / 2f,
                JoystickKnobSize, JoystickKnobSize), "");
        }

        void OnDestroy()
        {
            if (_ikTarget != null) Destroy(_ikTarget.gameObject);
        }
    }
}
