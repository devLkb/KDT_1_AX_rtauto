using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// 팔이 잡고 잠긴(텔레옵 핸드오프) 동안, RL이 도달한 자세를 기준점 삼아 사람이 살짝
    /// 다시 위치를 조절할 수 있게 한다 — 학습된 정책/모델은 건드리지 않고, 손가락이
    /// 바닥/받침대에 걸릴 때 잡기 전에 들어올리거나, 살짝 옆으로 옮기는 용도.
    /// 높이는 슬라이더로, 그 평면 안에서의 좌우/전후는 화면의 조이스틱 패드(드래그)로 조절한다
    /// — 조이스틱은 위치가 아니라 속도를 지정하는 방식(놓으면 그 자리에 멈춤)이라 슬라이더와
    /// 조작감이 다르다. 이동 축은 robotBase의 up/right/forward를 쓴다(Dg5fGraspSpec의
    /// TopDownAlignment 등이 이미 robotBase.up을 "바닥 수직" 기준으로 쓰고 있는 것과 통일).
    /// 데모 내내 꺼져있던 ArmTargetIK(+HandSliderUI)를 이 구간에서만 빌려 써서 위치는 CCD IK로
    /// 유지하고, 손바닥 평탄화는 이 스크립트가 손목 3관절(Wrist1~3)만 별도로 보정한다.
    /// Dg5fGraspAgent.ExternalArmControl로 에이전트 쪽의 xDrive 재적용은 막는다.
    /// </summary>
    public sealed class ArmTeleopNudge : MonoBehaviour
    {
        public Dg5fGraspAgent agent;
        public ArmTargetIK armIK;
        public HandSliderUI armSliderUI;

        [Tooltip("Game 뷰에 조절 UI(OnGUI)를 그릴지 여부")]
        public bool showControlUI = true;
        [Tooltip("잡은 지점 기준 위/아래로 조절 가능한 범위[m]")]
        public float maxHeightOffset = 0.12f;
        [Tooltip("잡은 지점 기준 수평(로봇 기준 좌우/전후)으로 조절 가능한 최대 반경[m]")]
        public float maxHorizontalOffset = 0.10f;
        [Tooltip("조이스틱을 끝까지 밀었을 때 수평 이동 속도[m/s]")]
        public float horizontalMoveSpeed = 0.05f;

        [Header("손바닥 평탄화 (Wrist 1~3만 사용, 위치 IK와 별도)")]
        [Tooltip("true면 텔레옵 중 손바닥(graspPoint.forward)이 항상 바닥과 평행(-robotBase.up 방향)하도록 " +
                 "손목 3관절을 매 틱 조금씩 보정한다.")]
        public bool flattenPalm = true;
        [Tooltip("FixedUpdate당 손목 관절 보정 최대량[deg] — 흔들리면 이 값을 줄인다.")]
        public float maxFlattenStepDeg = 1.2f;
        [Tooltip("이 이하 정렬 오차는 무시(미세 헌팅 방지)[deg]")]
        public float flattenDeadbandDeg = 0.3f;

        const float JoystickSize = 110f;
        const float JoystickKnobSize = 26f;
        // HandSliderUI.LoadDefaults() 순서 기준: 0=Shoulder Pan,1=Shoulder Lift,2=Elbow,
        // 3=Wrist1,4=Wrist2,5=Wrist3.
        const int WristJointStart = 3;
        const int WristJointCount = 3;

        Transform _ikTarget;
        Vector3 _basePosition;
        float _heightOffset;
        Vector2 _horizontalOffset; // (right, forward)
        Vector2 _joystickInput;    // -1..1 per axis, read in FixedUpdate
        bool _dragging;
        bool _active;

        ArticulationBody[] _wristBodies;
        bool _wristBodiesResolved;

        void Awake()
        {
            if (agent == null) agent = GetComponent<Dg5fGraspAgent>();
            if (armIK == null) armIK = GetComponent<ArmTargetIK>();
            if (armSliderUI == null) armSliderUI = GetComponent<HandSliderUI>();

            // ArmTargetIK/HandSliderUI는 원래 사람이 직접 조작하려고 만든 컴포넌트라 자체 OnGUI
            // 패널이 있다 — 이 데모에서는 이 스크립트가 대신 조작하므로 그 패널들은 항상 숨긴다.
            if (armIK != null) armIK.showUI = false;
            if (armSliderUI != null) armSliderUI.showUI = false;

            _ikTarget = new GameObject("ArmTeleopNudgeTarget").transform;
        }

        void FixedUpdate()
        {
            bool shouldBeActive = agent != null && agent.IsArmLocked;
            if (shouldBeActive != _active) SetActive(shouldBeActive);
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

            if (flattenPalm) ApplyPalmFlattenCorrection(up);
        }

        void SetActive(bool active)
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
                    // Awake의 슬라이더 초기값은 씬 시작 시점 포즈 — RL이 그 뒤로 팔을 옮겼으므로
                    // 지금 켜기 직전에 실제 자세로 재동기화해야 팔이 튀지 않는다.
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
                    // 손목(Wrist1~3)은 ApplyPalmFlattenCorrection이 오리엔테이션 전용으로 쓴다 —
                    // 위치 IK가 같은 관절을 동시에 건드리면 둘이 서로 되돌리며 흔들린다.
                    armIK.positionJointCount = WristJointStart;
                    armIK.enabled = true;
                }
                if (agent != null) agent.ExternalArmControl = true;

                ResolveWristBodiesIfNeeded();
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
                if (agent != null) agent.ExternalArmControl = false;
            }
        }

        void ResolveWristBodiesIfNeeded()
        {
            if (_wristBodiesResolved || armSliderUI == null || armSliderUI.armJoints == null) return;
            _wristBodiesResolved = true;

            var all = GetComponentsInChildren<ArticulationBody>(true);
            _wristBodies = new ArticulationBody[WristJointCount];
            for (int w = 0; w < WristJointCount; w++)
            {
                int jointIndex = WristJointStart + w;
                if (jointIndex >= armSliderUI.armJoints.Length) continue;
                string link = armSliderUI.armJoints[jointIndex].link;
                foreach (var b in all)
                    if (b.name == link) { _wristBodies[w] = b; break; }
            }
        }

        /// 손목 3관절(Wrist1~3)만 살짝 돌려서 손바닥(graspPoint.forward)이 바닥(-up)을 향하도록
        /// 매 틱 조금씩 보정한다. ArmTargetIK의 위치 CCD와 같은 오차 계산(SignedAngle)·같은
        /// signs[] 부호 보정을 그대로 재사용해 방향 규약을 통일한다. 위치 CCD와 같은 관절을
        /// 만지지만, 이 스크립트가 HandSliderUI보다 나중에 붙어 실행 순서가 뒤라 실제 xDrive
        /// 반영은 다음 틱으로 넘어갈 수 있다 — 사람이 조작하는 느린 텔레옵에는 지장 없는 지연이다.
        void ApplyPalmFlattenCorrection(Vector3 up)
        {
            if (_wristBodies == null || agent == null || agent.graspPoint == null) return;

            Vector3 desiredForward = -up;
            Vector3 currentForward = agent.graspPoint.forward;

            for (int w = _wristBodies.Length - 1; w >= 0; w--)
            {
                var b = _wristBodies[w];
                if (b == null) continue;
                int jointIndex = WristJointStart + w;

                Vector3 axis = (b.transform.rotation * b.anchorRotation) * Vector3.right;
                float err = Vector3.SignedAngle(currentForward, desiredForward, axis);
                if (Mathf.Abs(err) <= flattenDeadbandDeg) continue;

                float worldStep = Mathf.Clamp(err, -maxFlattenStepDeg, maxFlattenStepDeg);
                float sign = armIK != null && jointIndex < armIK.signs.Length ? armIK.signs[jointIndex] : 1f;
                var joint = armSliderUI.armJoints[jointIndex];
                joint.value = Mathf.Clamp(joint.value + worldStep * sign, joint.minDeg, joint.maxDeg);
                armSliderUI.armJoints[jointIndex] = joint;

                currentForward = Quaternion.AngleAxis(worldStep, axis) * currentForward;
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
