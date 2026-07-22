import importlib.util
import tempfile
import unittest
from pathlib import Path

import yaml


TRAINING = Path(__file__).parents[1]
CONFIG = TRAINING / "config" / "dg5f_grasp_point_reach.yaml"
GENERATOR_SCRIPT = (
    TRAINING / "scripts" / "generate_grasp_point_reach_smoke_config.py"
)
SPEC = importlib.util.spec_from_file_location(
    "generate_grasp_point_reach_smoke_config", GENERATOR_SCRIPT
)
assert SPEC and SPEC.loader
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)


class GraspPointReachContractTests(unittest.TestCase):
    def load_config(self, path: Path = CONFIG):
        return yaml.safe_load(path.read_text(encoding="utf-8"))

    def test_canonical_config_is_fresh_single_behavior_ppo(self):
        config = self.load_config()
        self.assertEqual(set(config), {"behaviors", "torch_settings"})
        self.assertEqual(set(config["behaviors"]), {"DG5FGraspReadyReach"})

        settings = config["behaviors"]["DG5FGraspReadyReach"]
        self.assertEqual(settings["trainer_type"], "ppo")
        self.assertEqual(
            settings["hyperparameters"],
            {
                "batch_size": 256,
                "buffer_size": 2048,
                "learning_rate": 0.0003,
                "beta": 0.005,
                "epsilon": 0.2,
                "lambd": 0.95,
                "num_epoch": 3,
                "learning_rate_schedule": "linear",
            },
        )
        self.assertEqual(
            settings["network_settings"],
            {
                "normalize": False,
                "hidden_units": 256,
                "num_layers": 3,
                "vis_encode_type": "simple",
            },
        )
        self.assertEqual(
            settings["reward_signals"],
            {"extrinsic": {"gamma": 0.99, "strength": 1.0}},
        )
        self.assertEqual(settings["max_steps"], 5_000_000)
        self.assertFalse(settings["threaded"])
        self.assertEqual(config["torch_settings"]["device"], "cuda")

    def test_canonical_config_has_no_transfer_or_curriculum(self):
        text = CONFIG.read_text(encoding="utf-8")
        for forbidden in (
            "curriculum",
            "environment_parameters",
            "initialize_from",
            "DG5FGraspJoint",
            "DG5FStableGrasp",
            "DG5FGrasp:",
        ):
            self.assertNotIn(forbidden, text)

        launcher = (
            TRAINING / "scripts" / "train_dg5f_grasp_point_reach.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("must not initialize from another run", launcher)

    def test_smoke_generator_changes_only_max_steps_to_exactly_512(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "smoke.yaml"
            GENERATOR.generate(CONFIG, destination)
            canonical = self.load_config()
            smoke = self.load_config(destination)

        canonical_behavior = canonical["behaviors"]["DG5FGraspReadyReach"]
        smoke_behavior = smoke["behaviors"]["DG5FGraspReadyReach"]
        self.assertEqual(smoke_behavior["max_steps"], 512)
        canonical_behavior["max_steps"] = 512
        self.assertEqual(smoke, canonical)
        self.assertEqual(
            self.load_config()["behaviors"]["DG5FGraspReadyReach"]["max_steps"],
            5_000_000,
        )

    def test_smoke_generator_rejects_curriculum_input(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "bad.yaml"
            destination = Path(directory) / "smoke.yaml"
            source.write_text(
                CONFIG.read_text(encoding="utf-8")
                + "\nenvironment_parameters:\n  curriculum: {}\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "curriculum"):
                GENERATOR.generate(source, destination)
            self.assertFalse(destination.exists())

    def test_legacy_point_reach_transfer_surfaces_are_removed(self):
        legacy = (
            "config/dg5f_grasp_point_reach_v1.yaml",
            "config/dg5f_grasp_point_reach_curriculum_v1.yaml",
            "scripts/bootstrap_v1_to_point_reach.py",
        )
        for relative_path in legacy:
            self.assertFalse((TRAINING / relative_path).exists(), relative_path)


if __name__ == "__main__":
    unittest.main()
