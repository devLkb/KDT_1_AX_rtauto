import csv
import json
import tempfile
import unittest
from pathlib import Path

from training.scripts.evaluate_dg5f_topdown import (
    DIRECTIONS,
    EXPECTED_BASE_SEED,
    REQUIRED_COLUMNS,
    freeze_first_passing_candidate,
    validate,
)


def write_ledger(
    path: Path,
    *,
    aligned_successes: dict[str, int],
    distance_successes: dict[str, int] | None = None,
    hold_successes: dict[str, int] | None = None,
    successful_angle: float = 10.0,
) -> None:
    distance_successes = distance_successes or {
        direction: 125 for direction in DIRECTIONS
    }
    hold_successes = hold_successes or {
        direction: 125 for direction in DIRECTIONS
    }
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=REQUIRED_COLUMNS)
        writer.writeheader()
        for episode in range(500):
            direction = DIRECTIONS[episode // 125]
            index = episode % 125
            aligned = index < aligned_successes[direction]
            distance = index < distance_successes[direction]
            hold = index < hold_successes[direction]
            writer.writerow(
                {
                    "episode": episode,
                    "seed": EXPECTED_BASE_SEED + episode,
                    "direction": direction,
                    "aligned_success": int(aligned),
                    "distance_success": int(distance),
                    "hold_success": int(hold),
                    "final_palm_angle_degrees": (
                        successful_angle if aligned else 90.0
                    ),
                    "final_surface_clearance_meters": 0.02 if aligned else 0.2,
                    "hold_seconds": 3.0 if aligned else 0.0,
                    "hold_position_error_meters": 0.01 if aligned else 0.1,
                    "unsafe_surface_contact": 0,
                    "misaligned_descent": 0,
                    "premature_descent": 0,
                    "termination_reason": "Success" if aligned else "Timeout",
                }
            )


class EvaluateDg5fTopDownTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.baseline = self.root / "baseline.csv"
        self.candidate = self.root / "candidate.csv"
        write_ledger(
            self.baseline,
            aligned_successes={direction: 100 for direction in DIRECTIONS},
        )
        write_ledger(
            self.candidate,
            aligned_successes={
                "front": 113,
                "right": 113,
                "back": 112,
                "left": 112,
            },
        )

    def tearDown(self):
        self.temporary.cleanup()

    def test_matched_500_seed_gate_accepts_exact_overall_threshold(self):
        _, candidate = validate(self.baseline, self.candidate)
        self.assertEqual(candidate.overall_aligned_success_rate, 0.90)
        self.assertGreaterEqual(
            candidate.directions["front"].aligned_success_rate,
            0.90,
        )
        self.assertGreaterEqual(
            candidate.directions["right"].aligned_success_rate,
            0.90,
        )

    def test_gate_rejects_a_direction_below_85_percent(self):
        write_ledger(
            self.candidate,
            aligned_successes={
                "front": 125,
                "right": 125,
                "back": 124,
                "left": 106,
            },
        )
        with self.assertRaisesRegex(ValueError, "left.*below 85%"):
            validate(self.baseline, self.candidate)

    def test_gate_rejects_success_above_fifteen_degrees(self):
        write_ledger(
            self.candidate,
            aligned_successes={
                "front": 125,
                "right": 125,
                "back": 118,
                "left": 125,
            },
            successful_angle=15.01,
        )
        with self.assertRaisesRegex(ValueError, "final palm angle"):
            validate(self.baseline, self.candidate)

    def test_gate_rejects_left_or_back_distance_hold_regression_over_5pp(self):
        write_ledger(
            self.candidate,
            aligned_successes={
                "front": 125,
                "right": 125,
                "back": 118,
                "left": 125,
            },
            hold_successes={
                "front": 125,
                "right": 125,
                "back": 118,
                "left": 125,
            },
        )
        with self.assertRaisesRegex(ValueError, "back hold_success_rate"):
            validate(self.baseline, self.candidate)

    def test_first_passing_checkpoint_is_frozen_and_cannot_be_replaced(self):
        baseline, candidate = validate(self.baseline, self.candidate)
        checkpoint = self.root / "candidate.pt"
        checkpoint.write_bytes(b"candidate")
        approved = self.root / "approved"

        manifest_path = freeze_first_passing_candidate(
            checkpoint,
            self.candidate,
            approved,
            baseline,
            candidate,
        )
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.assertEqual((approved / "checkpoint.pt").read_bytes(), b"candidate")
        self.assertEqual(
            manifest["selection_policy"],
            "first_checkpoint_passing_matched_500_seed_gate",
        )
        with self.assertRaises(FileExistsError):
            freeze_first_passing_candidate(
                checkpoint,
                self.candidate,
                approved,
                baseline,
                candidate,
            )


if __name__ == "__main__":
    unittest.main()
