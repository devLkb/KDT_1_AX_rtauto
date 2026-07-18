import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).parents[1] / "scripts" / "promote_dg5f_stable_model.py"
SPEC = importlib.util.spec_from_file_location("promote_dg5f_stable_model", SCRIPT)
assert SPEC and SPEC.loader
PROMOTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROMOTER)


class PromoteDg5fStableModelTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.model = self.root / "trained.onnx"
        self.ledger = self.root / "evaluation.csv"
        self.approval = self.root / "evaluation.csv.approved.json"
        self.models = self.root / "unity-models"
        self.model.write_bytes(b"116-observation/26-action-model")
        self.ledger.write_text("accepted 200-episode ledger\n")

    @staticmethod
    def sha256(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def write_approval(self, model_hash):
        self.approval.write_text(
            json.dumps(
                {
                    "specVersion": "3.0.0",
                    "behaviorName": "DG5FStableGrasp",
                    "episodes": 200,
                    "modelSha256": model_hash,
                    "evaluationCsvSha256": self.sha256(self.ledger),
                }
            )
        )

    def stage(self):
        evaluator = mock.Mock()
        evaluator.validate.return_value = (0.8, 0.8)
        with mock.patch.object(PROMOTER, "validate_onnx"), mock.patch.object(
            PROMOTER, "load_evaluator", return_value=evaluator
        ):
            return PROMOTER.stage(
                self.model,
                self.ledger,
                self.approval,
                self.models,
                self.root,
            )

    def test_rejects_evaluation_approval_for_another_model(self):
        self.write_approval("0" * 64)
        with self.assertRaisesRegex(ValueError, "model-bound evaluation approval mismatch"):
            self.stage()
        self.assertFalse(self.models.exists())

    def test_stages_only_the_model_bound_to_the_accepted_ledger(self):
        self.write_approval(self.sha256(self.model))
        canonical, manifest = self.stage()
        self.assertEqual(canonical.read_bytes(), self.model.read_bytes())
        staged = json.loads(manifest.read_text())
        self.assertEqual(staged["sha256"], self.sha256(self.model))
        self.assertEqual(staged["successRate"], 0.8)
        self.assertEqual(staged["reachRate"], 0.8)


if __name__ == "__main__":
    unittest.main()
