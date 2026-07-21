import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

import yaml


SCRIPT = Path(__file__).parents[1] / "scripts" / "validate_unity_environment.py"
SPEC = importlib.util.spec_from_file_location("validate_unity_environment", SCRIPT)
assert SPEC and SPEC.loader
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class UnityEnvironmentValidationTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.player = self.root / "DG5FGrasp.x86_64"
        self.dll = self.root / "Data" / "KDT.GraspTraining.dll"
        self.dll.parent.mkdir()
        self.player.write_bytes(b"player")
        self.dll.write_bytes(b"dll")
        self.config = self.root / "config.yaml"
        self.config.write_text(
            yaml.safe_dump(
                {
                    "behaviors": {"DG5FStableGrasp": {}},
                    "environment_parameters": {"stable_grasp_stage": {}},
                }
            ),
            encoding="utf-8",
        )
        self.manifest = self.root / "DG5F_ENVIRONMENT.json"
        self.write_manifest()

    @staticmethod
    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def write_manifest(self, **changes):
        value = {
            "format": "dg5f-unity-environment",
            "spec_version": "3.0.0",
            "behavior": "DG5FStableGrasp",
            "observation_size": 116,
            "continuous_actions": 26,
            "curriculum_parameter": "stable_grasp_stage",
            "player_sha256": self.digest(self.player),
            "grasp_dll": "Data/KDT.GraspTraining.dll",
            "grasp_dll_sha256": self.digest(self.dll),
        }
        value.update(changes)
        self.manifest.write_text(json.dumps(value), encoding="utf-8")

    def test_accepts_matching_package_and_config(self):
        result = VALIDATOR.validate(self.player, self.config, self.manifest)
        self.assertEqual(result["spec_version"], "3.0.0")

    def test_rejects_behavior_mismatch(self):
        self.config.write_text(
            yaml.safe_dump(
                {
                    "behaviors": {"DG5FGraspJoint": {}},
                    "environment_parameters": {"stable_grasp_stage": {}},
                }
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "behavior mismatch"):
            VALIDATOR.validate(self.player, self.config, self.manifest)

    def test_rejects_curriculum_parameter_mismatch(self):
        self.config.write_text(
            yaml.safe_dump(
                {
                    "behaviors": {"DG5FStableGrasp": {}},
                    "environment_parameters": {"joint26_stage": {}},
                }
            ),
            encoding="utf-8",
        )
        with self.assertRaisesRegex(
            VALIDATOR.ValidationError, "curriculum parameter mismatch"
        ):
            VALIDATOR.validate(self.player, self.config, self.manifest)

    def test_rejects_modified_binary(self):
        self.dll.write_bytes(b"modified")
        with self.assertRaisesRegex(VALIDATOR.ValidationError, "DLL SHA-256"):
            VALIDATOR.validate(self.player, self.config, self.manifest)


if __name__ == "__main__":
    unittest.main()
