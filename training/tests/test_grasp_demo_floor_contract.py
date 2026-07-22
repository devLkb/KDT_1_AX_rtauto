import unittest
from pathlib import Path

import yaml


TRAINING = Path(__file__).parents[1]
CONFIG = TRAINING / "config" / "dg5f_grasp_demo_floor_finetune.yaml"
LAUNCHER = TRAINING / "scripts" / "train_dg5f_grasp_demo_floor.sh"


class GraspDemoFloorContractTests(unittest.TestCase):
    def test_finetune_preserves_frozen_v1_policy_shape_contract(self):
        config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(set(config["behaviors"]), {"DG5FGrasp"})
        behavior = config["behaviors"]["DG5FGrasp"]
        self.assertEqual(behavior["trainer_type"], "ppo")
        self.assertEqual(behavior["network_settings"]["hidden_units"], 256)
        self.assertEqual(behavior["network_settings"]["num_layers"], 3)
        self.assertFalse(behavior["network_settings"]["normalize"])
        self.assertEqual(behavior["hyperparameters"]["learning_rate"], 0.00005)
        self.assertEqual(behavior["max_steps"], 1_000_000)
        self.assertFalse(behavior["threaded"])

    def test_launcher_initializes_only_from_the_verified_v1_run(self):
        launcher = LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("dg5f_v1_gpu_fixed", launcher)
        self.assertIn('--initialize-from "$SOURCE_RUN_ID"', launcher)
        self.assertIn("dg5f_grasp_demo_floor_finetune.yaml", launcher)
        self.assertIn("DG5FGrasp/checkpoint.pt", launcher)
        self.assertNotIn("DG5FGraspReadyReach", launcher)

    def test_launcher_keeps_start_and_resume_paths_separate(self):
        launcher = LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("start)", launcher)
        self.assertIn("resume)", launcher)
        self.assertIn("extra_args=(--resume)", launcher)


if __name__ == "__main__":
    unittest.main()
