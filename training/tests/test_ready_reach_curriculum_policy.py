from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
POLICY = ROOT / "training/policy/ready_reach/ReadyReachCurriculum.cs"
CONFIG = ROOT / "training/config/dg5f_grasp_ready_reach_curriculum.yaml"


class ReadyReachCurriculumPolicyTests(unittest.TestCase):
    def test_ready_reach_starts_at_5cm_and_finishes_at_3cm(self):
        source = POLICY.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")

        self.assertIn("_stage = FirstStage", source)
        self.assertIn("? 0.05f", source)
        self.assertIn("? 0.03f", source)
        self.assertIn("? 0.02f", source)
        self.assertIn("? 0.10f", source)
        self.assertIn("name: learn_5cm_lock", config)
        self.assertIn("value: 1.0", config)
        self.assertIn("name: finish_3cm_stable", config)
        self.assertIn("value: 2.0", config)
        self.assertNotIn("value: 3.0", config)

    def test_curriculum_does_not_change_policy_tensor_shape_contract(self):
        patcher = (
            ROOT / "training/policy/ready_reach/PatchReadyReach.cs"
        ).read_text(encoding="utf-8")
        self.assertNotIn("CollectObservations", patcher)
        self.assertNotIn("OnActionReceived", patcher)
        self.assertIn("MeetsLockState", patcher)
        self.assertIn("HasCompletedLockHold", patcher)


if __name__ == "__main__":
    unittest.main()
