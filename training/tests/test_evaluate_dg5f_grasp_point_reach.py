import csv
import importlib.util
import tempfile
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "evaluate_dg5f_grasp_point_reach.py"
)
SPEC = importlib.util.spec_from_file_location(
    "evaluate_dg5f_grasp_point_reach", SCRIPT
)
assert SPEC and SPEC.loader
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


class EvaluateGraspPointReachTests(unittest.TestCase):
    def write_ledger(self, mutate=None) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "evaluation.csv"
        rows = []
        for episode in range(500):
            success = episode < 450
            rows.append(
                {
                    "episode": episode,
                    "seed": 500_000 + episode,
                    "success": int(success),
                    "final_distance_meters": 0.01 if success else 0.02,
                    "grasp_point_speed_mps": 0.05 if success else 0.10,
                    "palm_alignment": 0.965925826 if success else 0.0,
                    "upper_cone_alignment": 0.707106781 if success else 0.0,
                    "success_hold_seconds": 0.25 if success else 0.0,
                    "elapsed_seconds": 1.0 if success else 20.0,
                    "minimum_transit_clearance_meters": 0.10,
                    "unsafe_surface_contact": 0,
                    "premature_descent": 0,
                    "workspace_safe": 1,
                    "finite_physics": 1,
                    "termination_reason": "Success" if success else "Timeout",
                }
            )
        if mutate:
            mutate(rows)
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=EVALUATOR.REQUIRED_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_accepts_exact_contract_at_ninety_percent_boundary(self):
        summary = EVALUATOR.validate(self.write_ledger())
        self.assertEqual(summary.success_rate, 0.9)
        self.assertEqual(summary.median_success_seconds, 1.0)
        self.assertEqual(summary.p95_success_seconds, 1.0)

    def test_rejects_below_ninety_percent_success(self):
        path = self.write_ledger(
            lambda rows: rows[449].update(
                success=0,
                success_hold_seconds=0.0,
                termination_reason="Timeout",
            )
        )
        with self.assertRaisesRegex(ValueError, "success rate"):
            EVALUATOR.validate(path)

    def test_rejects_duplicate_or_non_sequential_held_out_seed(self):
        path = self.write_ledger(lambda rows: rows[1].update(seed=rows[0]["seed"]))
        with self.assertRaisesRegex(ValueError, "duplicate seed"):
            EVALUATOR.validate(path)

        path = self.write_ledger(lambda rows: rows[1].update(seed=999_999))
        with self.assertRaisesRegex(ValueError, "base seed \\+ episode"):
            EVALUATOR.validate(path)

    def test_rejects_seed_permutation_between_episodes(self):
        def swap_seeds(rows):
            rows[0]["seed"], rows[1]["seed"] = rows[1]["seed"], rows[0]["seed"]

        with self.assertRaisesRegex(ValueError, "base seed \\+ episode"):
            EVALUATOR.validate(self.write_ledger(swap_seeds))

    def test_rejects_success_outside_distance_speed_or_hold_contract(self):
        cases = (
            ("final_distance_meters", 0.01001),
            ("grasp_point_speed_mps", 0.05001),
            ("success_hold_seconds", 0.24999),
            ("palm_alignment", 0.965),
            ("upper_cone_alignment", 0.707),
        )
        for column, value in cases:
            with self.subTest(column=column):
                path = self.write_ledger(
                    lambda rows, column=column, value=value: rows[0].update(
                        {column: value}
                    )
                )
                with self.assertRaisesRegex(ValueError, column):
                    EVALUATOR.validate(path)

    def test_rejects_timeout_workspace_and_physics_contract_violations(self):
        cases = (
            ("elapsed_seconds", 20.001, "elapsed_seconds"),
            ("workspace_safe", 0, "workspace safety"),
            ("finite_physics", 0, "non-finite physics"),
        )
        for column, value, message in cases:
            with self.subTest(column=column):
                path = self.write_ledger(
                    lambda rows, column=column, value=value: rows[499].update(
                        {column: value}
                    )
                )
                with self.assertRaisesRegex(ValueError, message):
                    EVALUATOR.validate(path)

    def test_rejects_surface_contact_premature_descent_and_low_clearance(self):
        cases = (
            ("unsafe_surface_contact", 1, "unsafe surface contact"),
            ("premature_descent", 1, "premature descent"),
            (
                "minimum_transit_clearance_meters",
                0.099,
                "minimum transit clearance",
            ),
        )
        for column, value, message in cases:
            with self.subTest(column=column):
                path = self.write_ledger(
                    lambda rows, column=column, value=value: rows[0].update(
                        {column: value}
                    )
                )
                with self.assertRaisesRegex(ValueError, message):
                    EVALUATOR.validate(path)

    def test_rejects_non_finite_measurement_and_mismatched_success_reason(self):
        path = self.write_ledger(
            lambda rows: rows[0].update(final_distance_meters="nan")
        )
        with self.assertRaisesRegex(ValueError, "not finite"):
            EVALUATOR.validate(path)

        path = self.write_ledger(
            lambda rows: rows[0].update(termination_reason="Timeout")
        )
        with self.assertRaisesRegex(ValueError, "termination_reason"):
            EVALUATOR.validate(path)

        path = self.write_ledger(
            lambda rows: rows[499].update(termination_reason="WorkspaceViolation")
        )
        with self.assertRaisesRegex(ValueError, "must terminate by Timeout"):
            EVALUATOR.validate(path)

    def test_requires_exactly_500_unique_episode_rows(self):
        path = self.write_ledger(lambda rows: rows.pop())
        with self.assertRaisesRegex(ValueError, "expected 500 rows"):
            EVALUATOR.validate(path)

        path = self.write_ledger(lambda rows: rows[1].update(episode=0))
        with self.assertRaisesRegex(ValueError, "duplicate episode"):
            EVALUATOR.validate(path)


if __name__ == "__main__":
    unittest.main()
