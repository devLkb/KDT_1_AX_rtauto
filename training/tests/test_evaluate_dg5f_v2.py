import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "evaluate_dg5f_v2.py"
SPEC = importlib.util.spec_from_file_location("evaluate_dg5f_v2", SCRIPT)
assert SPEC and SPEC.loader
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


class EvaluateDg5fV2Tests(unittest.TestCase):
    fields = [
        "episode_id",
        "seed",
        "area",
        "success",
        "failure_reason",
        "completion_seconds",
        "reach_success",
        "first_reach_seconds",
        "final_distance_meters",
        "best_distance_meters",
        "max_contact_hold_seconds",
    ]

    def write_ledger(self, mutate=None) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "evaluation.csv"
        rows = []
        for episode_id in range(200):
            rows.append(
                {
                    "episode_id": episode_id,
                    "seed": 10000 + episode_id,
                    "area": episode_id % 20,
                    "success": 1,
                    "failure_reason": "None",
                    "completion_seconds": 1.0,
                    "reach_success": 1,
                    "first_reach_seconds": 0.2,
                    "final_distance_meters": 0.01,
                    "best_distance_meters": 0.005,
                    "max_contact_hold_seconds": 0.5,
                }
            )
        if mutate:
            mutate(rows)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=self.fields)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def validate(self, path):
        return EVALUATOR.validate(path, 200, 20, 0.5, 0.8, 0.8)

    def test_accepts_complete_unique_ledger(self):
        self.assertEqual(self.validate(self.write_ledger()), (1.0, 1.0))

    def test_rejects_duplicate_seed(self):
        path = self.write_ledger(
            lambda rows: rows[1].update(seed=rows[0]["seed"])
        )
        with self.assertRaisesRegex(ValueError, "duplicate seed"):
            self.validate(path)

    def test_rejects_success_below_half_second_hold(self):
        path = self.write_ledger(
            lambda rows: rows[0].update(max_contact_hold_seconds=0.499)
        )
        with self.assertRaisesRegex(ValueError, "below 0.5"):
            self.validate(path)


if __name__ == "__main__":
    unittest.main()
