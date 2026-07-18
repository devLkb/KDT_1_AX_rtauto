import importlib.util
import tempfile
import unittest
from collections import OrderedDict
from pathlib import Path

import torch


SCRIPT = Path(__file__).parents[1] / "scripts" / "bootstrap_v1_to_joint26.py"
SPEC = importlib.util.spec_from_file_location("bootstrap_v1_to_joint26", SCRIPT)
assert SPEC and SPEC.loader
BOOTSTRAP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BOOTSTRAP)


def tensors_for(shapes):
    values = OrderedDict()
    offset = 1.0
    for name, shape in shapes.items():
        count = 1
        for dimension in shape:
            count *= dimension
        values[name] = torch.arange(count, dtype=torch.float32).reshape(shape) + offset
        offset += count
    return values


def source_checkpoint():
    policy = tensors_for(BOOTSTRAP.POLICY_SHAPES)
    critic = tensors_for(BOOTSTRAP.CRITIC_SHAPES)
    return {
        "Policy": policy,
        "global_step": {
            BOOTSTRAP.STEP_KEY: torch.tensor([526647], dtype=torch.int64)
        },
        "Optimizer:value_optimizer": {
            "state": {0: {"step": torch.tensor(10.0)}},
            "param_groups": [
                {
                    "lr": 3e-4,
                    "betas": (0.9, 0.999),
                    "eps": 1e-8,
                    "weight_decay": 0,
                    "amsgrad": False,
                    "maximize": False,
                    "foreach": None,
                    "capturable": False,
                    "differentiable": False,
                    "fused": None,
                    "decoupled_weight_decay": False,
                    "params": list(range(23)),
                }
            ],
        },
        "Optimizer:critic": critic,
    }


class BootstrapV1ToJoint26Tests(unittest.TestCase):
    def test_copies_encoder_critic_and_arm_action_exactly(self):
        source = source_checkpoint()
        converted = BOOTSTRAP.convert_checkpoint(source, seed=123)

        for module in ("Policy", "Optimizer:critic"):
            old = source[module][BOOTSTRAP.ENCODER_WEIGHT]
            new = converted[module][BOOTSTRAP.ENCODER_WEIGHT]
            self.assertTrue(torch.equal(new[:, :12], old[:, :12]))
            self.assertTrue(torch.equal(new[:, 72:116], old[:, 13:57]))
            self.assertEqual(tuple(new.shape), (256, 116))

        self.assertTrue(
            torch.equal(
                converted["Policy"][BOOTSTRAP.MU_WEIGHT][:6],
                source["Policy"][BOOTSTRAP.MU_WEIGHT][:6],
            )
        )
        self.assertTrue(
            torch.equal(
                converted["Policy"][BOOTSTRAP.LOG_SIGMA][:, :6],
                source["Policy"][BOOTSTRAP.LOG_SIGMA][:, :6],
            )
        )
        self.assertTrue(
            torch.equal(
                converted["Optimizer:critic"][
                    "value_heads.value_heads.extrinsic.weight"
                ],
                source["Optimizer:critic"][
                    "value_heads.value_heads.extrinsic.weight"
                ],
            )
        )

    def test_discards_closure_and_initializes_new_hand_parameters(self):
        source = source_checkpoint()
        converted = BOOTSTRAP.convert_checkpoint(source, seed=456)
        policy = converted["Policy"]

        self.assertEqual(tuple(policy[BOOTSTRAP.MU_WEIGHT].shape), (26, 256))
        self.assertFalse(
            any(
                torch.equal(row, source["Policy"][BOOTSTRAP.MU_WEIGHT][6])
                for row in policy[BOOTSTRAP.MU_WEIGHT][6:]
            )
        )
        self.assertFalse(
            torch.equal(
                policy[BOOTSTRAP.ENCODER_WEIGHT][:, 12],
                source["Policy"][BOOTSTRAP.ENCODER_WEIGHT][:, 12],
            )
        )
        self.assertTrue(
            torch.all(policy[BOOTSTRAP.LOG_SIGMA][:, 6:] == -2.0)
        )
        self.assertEqual(
            torch.count_nonzero(policy[BOOTSTRAP.MU_BIAS][6:]).item(), 20
        )
        self.assertEqual(converted["Optimizer:value_optimizer"]["state"], {})
        self.assertEqual(
            converted["global_step"][BOOTSTRAP.STEP_KEY].item(), 0
        )

    def test_rejects_any_source_tensor_shape_change(self):
        source = source_checkpoint()
        source["Policy"][BOOTSTRAP.ENCODER_WEIGHT] = torch.zeros(256, 58)
        with self.assertRaisesRegex(BOOTSTRAP.ConversionError, "expected"):
            BOOTSTRAP.convert_checkpoint(source)

    def test_writes_verified_read_only_initialize_from_run(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_path = root / "source.pt"
            output_run = root / "bootstrap"
            torch.save(source_checkpoint(), source_path)

            checkpoint_path = BOOTSTRAP.create_bootstrap_run(
                source_path, output_run, seed=789
            )
            self.assertEqual(
                checkpoint_path,
                output_run / "DG5FGraspJoint" / "checkpoint.pt",
            )
            self.assertTrue((output_run / "bootstrap_manifest.json").is_file())
            self.assertEqual(checkpoint_path.stat().st_mode & 0o222, 0)
            # A second call verifies and reuses rather than mutating the run.
            self.assertEqual(
                BOOTSTRAP.create_bootstrap_run(source_path, output_run, seed=789),
                checkpoint_path,
            )

    def test_writes_separate_stable_grasp_behavior_and_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_path = root / "source.pt"
            output_run = root / "stable-bootstrap"
            torch.save(source_checkpoint(), source_path)

            checkpoint_path = BOOTSTRAP.create_bootstrap_run(
                source_path,
                output_run,
                seed=790,
                target_behavior="DG5FStableGrasp",
                spec_version="3.0.0",
            )
            self.assertEqual(
                checkpoint_path,
                output_run / "DG5FStableGrasp" / "checkpoint.pt",
            )
            import json

            manifest = json.loads(
                (output_run / "bootstrap_manifest.json").read_text()
            )
            self.assertEqual(manifest["target_behavior"], "DG5FStableGrasp")
            self.assertEqual(manifest["spec_version"], "3.0.0")

    def test_loads_all_joint26_mlagents_modules_without_missing_keys(self):
        from mlagents.torch_utils import set_torch_config
        from mlagents.trainers.learn import parse_command_line
        from mlagents.trainers.policy.torch_policy import TorchPolicy
        from mlagents.trainers.ppo.optimizer_torch import TorchPPOOptimizer
        from mlagents.trainers.settings import TorchSettings
        from mlagents.trainers.torch_entities.networks import SimpleActor
        from mlagents_envs.base_env import (
            ActionSpec,
            BehaviorSpec,
            DimensionProperty,
            ObservationSpec,
            ObservationType,
        )

        set_torch_config(TorchSettings(device="cpu"))
        options = parse_command_line(
            [
                str(Path(__file__).parents[1] / "config" / "dg5f_grasp_v2.yaml"),
                "--run-id",
                "bootstrap-module-test",
                "--torch-device",
                "cpu",
            ]
        )
        settings = options.behaviors["DG5FGraspJoint"]
        behavior_spec = BehaviorSpec(
            [
                ObservationSpec(
                    (116,),
                    (DimensionProperty.NONE,),
                    ObservationType.DEFAULT,
                    "vector",
                )
            ],
            ActionSpec(26, ()),
        )
        policy = TorchPolicy(
            0,
            behavior_spec,
            settings.network_settings,
            SimpleActor,
            {"conditional_sigma": False, "tanh_squash": False},
        )
        optimizer = TorchPPOOptimizer(policy, settings)
        modules = {}
        modules.update(policy.get_modules())
        modules.update(optimizer.get_modules())
        converted = BOOTSTRAP.convert_checkpoint(source_checkpoint())

        self.assertEqual(set(modules), set(converted))
        for name, module in modules.items():
            if isinstance(module, torch.nn.Module):
                result = module.load_state_dict(converted[name], strict=False)
                self.assertEqual(result.missing_keys, [], name)
                self.assertEqual(result.unexpected_keys, [], name)
            else:
                module.load_state_dict(converted[name])
        self.assertEqual(policy.get_current_step(), 0)


if __name__ == "__main__":
    unittest.main()
