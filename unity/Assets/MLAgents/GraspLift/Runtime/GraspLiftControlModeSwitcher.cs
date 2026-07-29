using UnityEngine;

namespace KDT.GraspLiftTraining
{
    /// <summary>
    /// Pipeline_Demo_GraspLift의 자동/수동 전환 토글. 자동은 Dg5fGraspLiftAgent 정책이 그대로
    /// 팔+손을 움직여 블록을 잡고 들어올리고, 수동은 사람이 GraspLiftTeleopNudge(팔)와
    /// 실제 손 트래킹(Dg5fReceiver/Dg5fHandDriver, vision/dg5f/dg5f_teleop_gui.py가 UDP로 쏨)으로
    /// 직접 조종한다. GraspTargetSwitcher/DemoCameraSwitcher와 같은 OnGUI 버튼 컨벤션을 따른다.
    /// </summary>
    public sealed class GraspLiftControlModeSwitcher : MonoBehaviour
    {
        public Dg5fGraspLiftAgent agent;
        public GraspLiftTeleopNudge armNudge;
        public Dg5fReceiver handReceiver;
        public Dg5fHandDriver handDriver;

        [Tooltip("Game 뷰에 자동/수동 전환 UI(OnGUI)를 그릴지 여부")]
        public bool showModeUI = true;

        bool _isManual;

        public bool IsManual => _isManual;

        public void SetManualMode(bool manual)
        {
            if (manual == _isManual) return;
            _isManual = manual;

            if (manual)
            {
                if (agent != null) agent.PauseForManualControl();
                if (armNudge != null) armNudge.SetActive(true);
                if (handReceiver != null) handReceiver.enabled = true;
                if (handDriver != null) handDriver.enabled = true;
            }
            else
            {
                if (armNudge != null) armNudge.SetActive(false);
                if (handReceiver != null) handReceiver.enabled = false;
                if (handDriver != null) handDriver.enabled = false;
                // 사람이 임의로 옮겨놓은 자세에서 정책을 재개시키는 건 학습 분포 밖이라 위험하다 —
                // 항상 새 시도(에피소드 리셋)로 자동 모드에 복귀한다.
                if (agent != null) agent.EndEpisode();
            }
        }

        void OnGUI()
        {
            if (!showModeUI) return;

            GUILayout.BeginArea(new Rect(10, 10, 200, 70), GUI.skin.box);
            GUILayout.Label("제어 모드");
            GUILayout.BeginHorizontal();
            if (GUILayout.Button(!_isManual ? "[자동]" : "자동")) SetManualMode(false);
            if (GUILayout.Button(_isManual ? "[수동]" : "수동")) SetManualMode(true);
            GUILayout.EndHorizontal();
            GUILayout.EndArea();
        }
    }
}
