import re
import unittest
from pathlib import Path

import yaml


TRAINING_ROOT = Path(__file__).parents[1]
CONFIG = TRAINING_ROOT / "config" / "dg5f_grasp_v3.yaml"
LAUNCHER = TRAINING_ROOT / "scripts" / "dg5f.sh"
EVALUATION_LAUNCHER = (
    TRAINING_ROOT / "scripts" / "run_dg5f_v2_evaluation.sh"
)

HAND_FIRST_RUN = "dg5f_v2_stablegrasp_v3_lr5e5_gpu_fixed"
FAILED_RUNS = {
    "dg5f_v2_closure_failed_343k",
    "dg5f_v2_gpu_fixed",
    "dg5f_v2_joint26_gpu_fixed",
    "dg5f_v2_joint26_lr5e5_gpu_fixed",
    # Ran on the stale DG5FGraspJoint26 build without the hand-first logic.
    "dg5f_v2_joint26_handfirst_lr5e5_gpu_fixed",
    # Free-falling ball made stage-1 grasp success physically unreachable;
    # stopped at the 100k stage-1 gate.
    "dg5f_v2_joint26_handfirst2_lr5e5_gpu_fixed",
    # Interrupted at 839959 while the old result/player directories were moved.
    "dg5f_v2_joint26_handfirst3_lr5e5_gpu_fixed",
}


class HandFirstTrainingContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        cls.launcher = LAUNCHER.read_text(encoding="utf-8")

    def test_policy_shape_preserving_training_settings(self):
        behavior = self.config["behaviors"]["DG5FStableGrasp"]
        self.assertEqual(
            behavior["hyperparameters"]["learning_rate"],
            5e-5,
        )
        self.assertEqual(behavior["checkpoint_interval"], 100_000)
        self.assertEqual(behavior["max_steps"], 3_000_000)
        self.assertFalse(behavior["threaded"])

    def test_curriculum_uses_unsmoothed_recent_two_hundred_episode_reward(self):
        lessons = self.config["environment_parameters"]["stable_grasp_stage"][
            "curriculum"
        ]
        self.assertEqual(
            [lesson["value"]["sampler_parameters"]["value"] for lesson in lessons],
            [1.0, 2.0, 3.0],
        )
        criteria = [lesson["completion_criteria"] for lesson in lessons[:2]]
        self.assertEqual([item["measure"] for item in criteria], ["reward", "reward"])
        self.assertEqual([item["threshold"] for item in criteria], [3.5, 5.0])
        self.assertEqual([item["min_lesson_length"] for item in criteria], [200, 200])
        self.assertEqual([item["signal_smoothing"] for item in criteria], [False, False])
        self.assertTrue(all(item["require_reset"] for item in criteria))

    def test_handfirst_run_is_active_and_all_failed_runs_are_resume_protected(self):
        self.assertRegex(
            self.launcher,
            rf'STAGE_RUN_ID="{re.escape(HAND_FIRST_RUN)}"',
        )
        self.assertRegex(
            self.launcher,
            rf'PREVIOUS_RUN_ID="{re.escape(HAND_FIRST_RUN)}"',
        )
        self.assertIn("dg5f_grasp_v3.yaml", self.launcher)
        self.assertIn("DG5FGraspV3", self.launcher)
        for run_id in FAILED_RUNS:
            self.assertIn(run_id, self.launcher)
        protected_condition = self.launcher[
            self.launcher.index('if [[ "$mode" == resume')
            : self.launcher.index(
                'echo "[ERROR] 보호된 원본/실패/bootstrap run은 resume',
            )
        ]
        for constant in (
            "FAILED_CLOSURE_V2_RUN_ID",
            "FAILED_CLOSURE_V2_ORIGINAL_RUN_ID",
            "FAILED_JOINT26_PILOT_RUN_ID",
            "FAILED_JOINT26_LR5E5_RUN_ID",
            "FAILED_HANDFIRST_STALEBUILD_RUN_ID",
            "FAILED_HANDFIRST2_GATE_RUN_ID",
            "RETIRED_HANDFIRST3_RUN_ID",
            "V1_JOINT26_BOOTSTRAP_RUN_ID",
        ):
            self.assertIn(constant, protected_condition)

        v3_source_guard = self.launcher[
            self.launcher.index('if [[ "$mode" == init && "$STAGE" == v3')
            : self.launcher.index(
                'echo "[ERROR] 실패 V2 run은 V3 전이 원본',
            )
        ]
        for constant in (
            "FAILED_CLOSURE_V2_RUN_ID",
            "FAILED_CLOSURE_V2_ORIGINAL_RUN_ID",
            "FAILED_JOINT26_PILOT_RUN_ID",
            "FAILED_JOINT26_LR5E5_RUN_ID",
            "FAILED_HANDFIRST_STALEBUILD_RUN_ID",
            "FAILED_HANDFIRST2_GATE_RUN_ID",
            "RETIRED_HANDFIRST3_RUN_ID",
        ):
            self.assertIn(constant, v3_source_guard)

    def test_evaluation_defaults_to_handfirst_run(self):
        evaluation = EVALUATION_LAUNCHER.read_text(encoding="utf-8")
        self.assertIn(HAND_FIRST_RUN, evaluation)
        self.assertIn("dg5f_grasp_v3.yaml", evaluation)
        self.assertIn("DG5FGraspV3", evaluation)


if __name__ == "__main__":
    unittest.main()
