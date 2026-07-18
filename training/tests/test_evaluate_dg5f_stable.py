import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "evaluate_dg5f_stable.py"
SPEC = importlib.util.spec_from_file_location("evaluate_dg5f_stable", SCRIPT)
assert SPEC and SPEC.loader
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


class EvaluateDg5fStableTests(unittest.TestCase):
    fields = sorted(EVALUATOR.REQUIRED_COLUMNS)

    def write_ledger(self, mutate=None) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "evaluation.csv"
        rows = []
        for episode_id in range(200):
            rows.append(
                {
                    "episode_id": episode_id,
                    "seed": 300000 + episode_id,
                    "area": episode_id % 20,
                    "success": 1,
                    "failure_reason": "None",
                    "completion_seconds": 2.0,
                    "reach_success": 1,
                    "first_reach_seconds": 0.2,
                    "final_distance_meters": 0.01,
                    "best_distance_meters": 0.005,
                    "max_contact_hold_seconds": 0.5,
                    "max_contact_finger_count": 3,
                    "max_non_thumb_contact_count": 2,
                    "stable_grasp_acquired": 1,
                    "max_lift_height_meters": 0.05,
                    "max_valid_hold_seconds": 1.0,
                    "grip_lost_cause": "None",
                    "thumb_maintained_after_stable": 1,
                    "minimum_non_thumb_contacts_after_stable": 2,
                }
            )
        if mutate:
            mutate(rows)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=self.fields)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_accepts_complete_stable_grasp_ledger(self):
        self.assertEqual(EVALUATOR.validate(self.write_ledger()), (1.0, 1.0))

    def test_rejects_thumb_plus_one_success(self):
        path = self.write_ledger(
            lambda rows: rows[0].update(
                max_contact_finger_count=2,
                max_non_thumb_contact_count=1,
                minimum_non_thumb_contacts_after_stable=1,
            )
        )
        with self.assertRaisesRegex(ValueError, "max_contact_finger_count"):
            EVALUATOR.validate(path)

    def test_rejects_success_without_five_centimeter_lift(self):
        path = self.write_ledger(
            lambda rows: rows[0].update(max_lift_height_meters=0.049)
        )
        with self.assertRaisesRegex(ValueError, "max_lift_height_meters"):
            EVALUATOR.validate(path)

    def test_rejects_success_without_one_second_hold(self):
        path = self.write_ledger(
            lambda rows: rows[0].update(max_valid_hold_seconds=0.999)
        )
        with self.assertRaisesRegex(ValueError, "max_valid_hold_seconds"):
            EVALUATOR.validate(path)


if __name__ == "__main__":
    unittest.main()
