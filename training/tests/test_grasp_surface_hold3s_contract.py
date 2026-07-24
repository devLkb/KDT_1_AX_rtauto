import unittest
from pathlib import Path

import yaml


TRAINING = Path(__file__).parents[1]
CONFIG = TRAINING / "config" / "dg5f_grasp_surface_hold3s_finetune.yaml"
LAUNCHER = TRAINING / "scripts" / "train_dg5f_grasp_surface_hold3s.sh"


class GraspSurfaceHoldContractTests(unittest.TestCase):
    def test_transfer_shape_and_conservative_finetune_settings(self):
        config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(set(config["behaviors"]), {"DG5FGrasp"})
        behavior = config["behaviors"]["DG5FGrasp"]
        self.assertEqual(behavior["network_settings"]["hidden_units"], 256)
        self.assertEqual(behavior["network_settings"]["num_layers"], 3)
        self.assertFalse(behavior["network_settings"]["normalize"])
        self.assertEqual(behavior["hyperparameters"]["learning_rate"], 0.00002)
        self.assertEqual(behavior["hyperparameters"]["beta"], 0.001)
        self.assertEqual(behavior["checkpoint_interval"], 100_000)
        self.assertEqual(behavior["max_steps"], 1_000_000)

    def test_launcher_uses_completed_floor_safe_run_and_separate_build(self):
        launcher = LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("dg5f_vdi_floor_safe_transfer_gpu_20260722", launcher)
        self.assertIn("DG5FGraspSurfaceHold3s-20260722", launcher)
        self.assertIn('--initialize-from "$SOURCE_RUN_ID"', launcher)
        self.assertIn("start)", launcher)
        self.assertIn("resume)", launcher)


if __name__ == "__main__":
    unittest.main()
