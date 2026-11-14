#!/usr/bin/env python3
"""Read six finished trials and independently reproduce the supplied metrics.

This script never runs a simulation; it writes comparison reports only.
Without --watch it reports missing inputs and exits immediately. With --watch
it checks for the six finalized MAT files at a configurable, slow interval.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import time

from independent_metrics import calculate_metrics, compare_expected, read_expected

ROOT = Path(__file__).resolve().parents[1]


CONDITIONS = ((50, 0.5, 0.0), (70, 0.6, 2.5), (90, 0.6, 2.5))
VARIANTS = ("baseline", "revised")
NUMERIC_FIELDS = (
    "Baseline_RMSE", "Revised_RMSE", "RMSE_Reduction_pct",
    "Baseline_MaxAbsError", "Revised_MaxAbsError", "Baseline_RMSE_0_2",
    "Revised_RMSE_0_2", "Baseline_RMSE_2_10", "Revised_RMSE_2_10",
)


def validate_trials(result_dir: Path, reference_dir: Path, expected: Path,
                    output_dir: Path, atol: float, rtol: float) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    all_comparisons, flat_rows, computed_rows = [], [], []
    csv_header = list(read_expected(expected)[0])
    for speed, mu, steer in CONDITIONS:
        condition = {"speed_kmh": float(speed), "mu": mu, "steer_command_deg": steer}
        metrics = {}
        for variant in VARIANTS:
            actual = result_dir / f"reference_{speed}_{variant}.mat"
            reference = reference_dir / f"CarSim_{speed}.mat"
            metrics[variant] = calculate_metrics(actual, reference, condition, variant)
            target = output_dir / f"metrics_{speed}_{variant}.json"
            target.write_text(json.dumps(metrics[variant], indent=2, allow_nan=False) + "\n")
        report = compare_expected(metrics["baseline"], metrics["revised"], expected, atol, rtol)
        (output_dir / f"comparison_{speed}.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        all_comparisons.append(report)
        for signal_report in report["signals"]:
            signal = signal_report["signal"]
            checks = signal_report["checks"]
            computed_row = {
                "Speed_kmh": speed, "RoadMu": mu, "SteerCommand_deg": steer,
                "Signal": signal, "Assessment": signal_report["expected_assessment"],
            }
            for field in csv_header:
                if field in checks:
                    computed_row[field] = checks[field]["actual"]
            # Assessment is an annotation from the supplied CSV, not a new
            # numerical finding; the JSON report retains that distinction.
            computed_rows.append(computed_row)
            for field, check in checks.items():
                flat_rows.append({"Speed_kmh": speed, "RoadMu": mu, "SteerCommand_deg": steer,
                                  "Signal": signal, "Metric": field, **check})
    with (output_dir / "computed_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=csv_header)
        writer.writeheader()
        writer.writerows(computed_rows)
    detail_fields = ("Speed_kmh", "RoadMu", "SteerCommand_deg", "Signal", "Metric",
                     "actual", "expected", "absolute_error", "relative_error", "tolerance_limit", "passed")
    with (output_dir / "detailed_comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=detail_fields)
        writer.writeheader()
        writer.writerows(flat_rows)
    numeric_rows = [row for row in flat_rows if row["Metric"] in NUMERIC_FIELDS]
    worst_by_metric = {}
    for metric in NUMERIC_FIELDS:
        rows = [row for row in numeric_rows if row["Metric"] == metric]
        worst = max(rows, key=lambda row: row["absolute_error"] if row["absolute_error"] is not None else float("inf"))
        worst_by_metric[metric] = {key: worst[key] for key in (
            "Speed_kmh", "Signal", "actual", "expected", "absolute_error", "relative_error", "tolerance_limit", "passed")}
    summary = {
        "schema": "HMC_THREE_CONDITION_EXPECTED_METRICS_AUDIT_V1",
        "overall_match": all(r["overall_match"] for r in all_comparisons),
        "condition_count": len(all_comparisons), "signal_row_count": len(computed_rows),
        "numeric_check_count": len(numeric_rows),
        "numeric_checks_passed": sum(bool(r["passed"]) for r in numeric_rows),
        "signal_rows_passed": sum(s["passed"] for r in all_comparisons for s in r["signals"]),
        "tolerances": {"atol": atol, "rtol": rtol},
        "conditions": [{"condition": r["condition"], "reference_condition_status": r["reference_condition_status"],
                        "numeric_match": r["numeric_match"], "overall_match": r["overall_match"]} for r in all_comparisons],
        "worst_absolute_error_by_metric": worst_by_metric,
        "failed_checks": [row for row in flat_rows if not row["passed"]],
        "assessment_column_note": "Assessment strings in computed_metrics.csv are preserved annotations from the supplied CSV, not recomputed numeric metrics",
        "method": "Actual Result.response arrays only; EXPORT unit conversion; recorded-start subtraction; linear interpolation; all samples retained",
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-dir", type=Path, default=ROOT / "result/validation/three_cases")
    parser.add_argument("--reference-dir", type=Path, default=ROOT / "validation/reference_flat")
    parser.add_argument("--expected", type=Path, default=ROOT / "validation/reference_input/HMC_Comparison_Metrics.csv")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "result/validation/recomputed")
    parser.add_argument("--atol", type=float, default=1e-9)
    parser.add_argument("--rtol", type=float, default=1e-5)
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--poll-seconds", type=float, default=30)
    args = parser.parse_args()
    if args.poll_seconds < 10:
        parser.error("Use a polling interval of at least 10 seconds")
    required = [args.result_dir / f"reference_{speed}_{variant}.mat" for speed, _, _ in CONDITIONS for variant in VARIANTS]
    if args.watch:
        # The trial runner writes this after all MAT saves have finished.
        # Waiting for the marker avoids reading a partially written last MAT.
        required.append(args.result_dir / "trial_manifest.json")
    previous_missing = None
    while True:
        missing = [str(path) for path in required if not path.is_file() or path.stat().st_size == 0]
        if not missing:
            break
        if missing != previous_missing:
            print(json.dumps({"state": "waiting_for_finished_trials", "missing": missing}), flush=True)
            previous_missing = missing
        if not args.watch:
            return 3
        time.sleep(args.poll_seconds)
    report = validate_trials(args.result_dir, args.reference_dir, args.expected, args.output_dir, args.atol, args.rtol)
    print(json.dumps({"output_dir": str(args.output_dir), "overall_match": report["overall_match"],
                      "signal_rows_passed": report["signal_rows_passed"],
                      "numeric_checks_passed": report["numeric_checks_passed"],
                      "numeric_check_count": report["numeric_check_count"]}), flush=True)
    return 0 if report["overall_match"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
