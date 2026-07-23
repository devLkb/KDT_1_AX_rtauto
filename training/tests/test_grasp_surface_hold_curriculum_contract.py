import unittest
from pathlib import Path

import yaml


TRAINING = Path(__file__).parents[1]
CONFIG = TRAINING / "config" / "dg5f_grasp_surface_hold_curriculum.yaml"
LAUNCHER = TRAINING / "scripts" / "train_dg5f_grasp_surface_hold_curriculum.sh"


class GraspSurfaceHoldCurriculumContractTests(unittest.TestCase):
    def test_curriculum_reaches_final_three_second_stage(self):
        config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        behavior = config["behaviors"]["DG5FGrasp"]
        self.assertEqual(behavior["hyperparameters"]["learning_rate"], 0.00005)
        self.assertEqual(behavior["hyperparameters"]["beta"], 0.00001)
        self.assertEqual(behavior["max_steps"], 2_000_000)
        lessons = config["environment_parameters"]["hold_stage"]["curriculum"]
        self.assertEqual(
            [lesson["value"]["sampler_parameters"]["value"] for lesson in lessons],
            [1.0, 2.0, 3.0, 4.0, 5.0],
        )
        self.assertNotIn("completion_criteria", lessons[-1])
        self.assertTrue(
            all(
                lesson["completion_criteria"]["require_reset"]
                for lesson in lessons[:-1]
            )
        )

    def test_launcher_uses_prepared_good_checkpoint_and_isolated_build(self):
        launcher = LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("dg5f_vdi_floor_safe_hold_curriculum_init_20260723", launcher)
        self.assertIn("DG5FGraspSurfaceHoldCurriculum-20260723", launcher)
        self.assertNotIn("surface3cm_hold3s_transfer_gpu_20260722", launcher)
        self.assertIn('--initialize-from "$SOURCE_RUN_ID"', launcher)


if __name__ == "__main__":
    unittest.main()
