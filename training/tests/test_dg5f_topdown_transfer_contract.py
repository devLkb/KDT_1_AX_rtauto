import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import onnx
import yaml

from training.scripts.generate_dg5f_topdown_smoke_config import generate
from training.scripts.prepare_dg5f_topdown_transfer import (
    BEHAVIOR,
    EXPECTED_SHA256,
    prepare_source_run,
    validate_checkpoint,
)


ROOT = Path(__file__).parents[2]
TRAINING = ROOT / "training"
CONFIG = TRAINING / "config" / "dg5f_grasp_topdown_transfer.yaml"
STAGE2_CONFIG = TRAINING / "config" / "dg5f_grasp_topdown_stage2_transfer.yaml"
LAUNCHER = TRAINING / "scripts" / "train_dg5f_grasp_topdown_transfer.sh"
STAGE2_LAUNCHER = (
    TRAINING / "scripts" / "train_dg5f_grasp_topdown_stage2_transfer.sh"
)
SMOKE = TRAINING / "scripts" / "smoke_dg5f_grasp_topdown_transfer.sh"
SOURCE = (
    TRAINING
    / "results"
    / "dg5f_vdi_surface3cm_hold3s_curriculum_best_observed_600k_20260723"
    / "DG5FGrasp-599887.pt"
)
ONNX = (
    ROOT
    / "unity"
    / "Assets"
    / "MLAgents"
    / "Grasp"
    / "Models"
    / "DG5FGrasp-599887.onnx"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Dg5fTopDownTransferContractTests(unittest.TestCase):
    def test_config_preserves_57x7_network_and_uses_planned_ppo_settings(self):
        config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        behavior = config["behaviors"][BEHAVIOR]
        hyperparameters = behavior["hyperparameters"]
        network = behavior["network_settings"]

        self.assertEqual(hyperparameters["batch_size"], 1024)
        self.assertEqual(hyperparameters["buffer_size"], 10240)
        self.assertEqual(hyperparameters["learning_rate"], 0.00002)
        self.assertEqual(hyperparameters["learning_rate_schedule"], "linear")
        self.assertEqual(hyperparameters["beta"], 0.0001)
        self.assertEqual(network["hidden_units"], 256)
        self.assertEqual(network["num_layers"], 3)
        self.assertEqual(behavior["checkpoint_interval"], 100_000)
        self.assertEqual(behavior["max_steps"], 1_000_000)

        lessons = config["environment_parameters"]["hold_stage"]["curriculum"]
        self.assertEqual(
            [lesson["value"]["sampler_parameters"]["value"] for lesson in lessons],
            [1.0, 2.0, 3.0, 4.0, 5.0],
        )
        self.assertEqual(
            [80, 60, 45, 30, 15],
            [
                int(lesson["name"].split("deg", 1)[0].rsplit("_", 1)[1])
                for lesson in lessons
            ],
        )
        self.assertNotIn("completion_criteria", lessons[-1])

    def test_launcher_prepares_an_immutable_source_and_initializes_a_new_run(self):
        launcher = LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("prepare_dg5f_topdown_transfer.py", launcher)
        self.assertIn("DG5FGrasp-599887.pt", launcher)
        self.assertIn('--initialize-from "$SOURCE_RUN_ID"', launcher)
        self.assertIn("new transfer run already exists", launcher)
        self.assertIn("the immutable source run can never be resumed", launcher)
        self.assertNotIn("prepare_hold_curriculum_init.py", launcher)
        self.assertNotIn("--force", launcher)
        self.assertIn("generate_dg5f_topdown_smoke_config.py", SMOKE.read_text())

    def test_stage2_transfer_uses_the_completed_stage1_run_and_fixed_lesson(self):
        config = yaml.safe_load(STAGE2_CONFIG.read_text(encoding="utf-8"))
        behavior = config["behaviors"][BEHAVIOR]
        hold_stage = config["environment_parameters"]["hold_stage"]
        launcher = STAGE2_LAUNCHER.read_text(encoding="utf-8")

        self.assertEqual(behavior["hyperparameters"]["learning_rate"], 0.00002)
        self.assertEqual(behavior["hyperparameters"]["learning_rate_schedule"], "linear")
        self.assertEqual(behavior["network_settings"]["hidden_units"], 256)
        self.assertEqual(behavior["network_settings"]["num_layers"], 3)
        self.assertEqual(behavior["max_steps"], 1_000_000)
        self.assertEqual(config["torch_settings"]["device"], "cpu")
        self.assertNotIn("curriculum", hold_stage)
        self.assertEqual(hold_stage["sampler_parameters"]["value"], 2.0)

        self.assertIn(
            "dg5f_grasp_topdown_curriculum_v2_cpu_1m_20260724",
            launcher,
        )
        self.assertIn('--initialize-from "$SOURCE_RUN_ID"', launcher)
        self.assertIn("source and target run IDs must differ", launcher)
        self.assertIn("refusing to overwrite", launcher)

    def test_original_checkpoint_hash_step_and_shapes_match_the_transfer_contract(self):
        contract = validate_checkpoint(SOURCE)
        self.assertEqual(contract.behavior, BEHAVIOR)
        self.assertEqual(contract.sha256, EXPECTED_SHA256)
        self.assertEqual(contract.global_step, 599_887)
        self.assertEqual(contract.observations, 57)
        self.assertEqual(contract.continuous_actions, 7)
        self.assertEqual(contract.hidden_units, 256)
        self.assertEqual(contract.hidden_layers, 3)

    def test_preparation_copies_bytes_without_modifying_observation_weights(self):
        source_hash_before = sha256(SOURCE)
        with tempfile.TemporaryDirectory() as temporary:
            results = Path(temporary)
            destination, manifest_path = prepare_source_run(
                SOURCE,
                results,
                "source-run",
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

            self.assertEqual(destination, results / "source-run" / BEHAVIOR / "checkpoint.pt")
            self.assertEqual(sha256(destination), source_hash_before)
            self.assertEqual(sha256(SOURCE), source_hash_before)
            self.assertTrue(manifest["source_unchanged"])
            self.assertFalse(manifest["observation_weights_modified"])
            self.assertEqual(manifest["contract"]["sha256"], EXPECTED_SHA256)
            self.assertEqual(manifest["contract"]["observations"], 57)

    def test_smoke_generator_changes_only_the_maximum_step_contract(self):
        with tempfile.TemporaryDirectory() as temporary:
            generated = Path(temporary) / "smoke.yaml"
            generate(CONFIG, generated)
            smoke = yaml.safe_load(generated.read_text(encoding="utf-8"))
            self.assertEqual(smoke["behaviors"][BEHAVIOR]["max_steps"], 512)
            self.assertEqual(
                smoke["behaviors"][BEHAVIOR]["network_settings"]["hidden_units"],
                256,
            )
            self.assertIn("hold_stage", smoke["environment_parameters"])

    def test_existing_onnx_keeps_the_57_observation_7_action_contract(self):
        model = onnx.load(ONNX)
        inputs = {item.name: item for item in model.graph.input}
        outputs = {item.name: item for item in model.graph.output}
        self.assertEqual(
            inputs["obs_0"].type.tensor_type.shape.dim[1].dim_value,
            57,
        )
        self.assertEqual(
            outputs["continuous_actions"].type.tensor_type.shape.dim[1].dim_value,
            7,
        )


if __name__ == "__main__":
    unittest.main()
