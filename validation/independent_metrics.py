#!/usr/bin/env python3
"""Independent, unadjusted six-signal CarSim comparison for HMC results.

Inputs are a v7-compatible Result MAT and a lossless flat reference MAT.
Conditions come from explicit arguments and embedded metadata, never filenames.
No fitting, smoothing, amplitude adjustment, or fitted time shift is performed.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.io import loadmat


# MATLAB EXPORT column numbers are one-based; indices below are zero-based.
SIGNALS = {
    "vy_mps": (3, 1.0 / 3.6, "m/s"),
    "rollrate_degps": (11, 1.0, "deg/s"),
    "r_radps": (13, math.pi / 180.0, "rad/s"),
    "vx_mps": (2, 1.0 / 3.6, "m/s"),
    "pitchrate_degps": (12, 1.0, "deg/s"),
    "roll_deg": (8, 1.0, "deg"),
}
CONDITION_KEYS = ("speed_kmh", "mu", "steer_command_deg")
SPLITS = ("early_le_late_gt", "early_lt_late_ge", "inclusive_both")


def sha256(path: str | Path) -> str:
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _vector(value: Any, label: str) -> np.ndarray:
    data = np.asarray(value)
    if np.iscomplexobj(data) or data.dtype.kind not in "biuf":
        raise ValueError(f"{label} must be a real numeric array")
    if data.ndim > 2 or (data.ndim == 2 and min(data.shape) != 1):
        raise ValueError(f"{label} must be a vector")
    data = np.asarray(data, dtype=np.float64).reshape(-1)
    if not np.all(np.isfinite(data)):
        raise ValueError(f"{label} contains nonfinite values")
    return data


def _times(value: Any, label: str) -> np.ndarray:
    data = _vector(value, label)
    if len(data) < 2 or not np.all(np.diff(data) > 0):
        raise ValueError(f"{label} must contain increasing timestamps")
    return data


def _condition_check(metadata: dict, expected: dict) -> dict:
    found = {}
    missing = []
    mismatches = []
    for key in CONDITION_KEYS:
        if key not in metadata:
            missing.append(key)
            continue
        value = np.asarray(metadata[key], dtype=float)
        if value.size != 1 or not np.isfinite(value.item()):
            raise ValueError(f"Invalid embedded condition {key}")
        found[key] = float(value.item())
        if not math.isclose(found[key], expected[key], rel_tol=0, abs_tol=1e-9):
            mismatches.append(key)
    return {
        "status": "mismatched" if mismatches else ("unverified" if missing else "verified"),
        "embedded": found,
        "missing_fields": missing,
        "mismatched_fields": mismatches,
    }


def _load_struct(path: str | Path, variable: str) -> dict:
    loaded = loadmat(path, simplify_cells=True)
    if variable not in loaded or not isinstance(loaded[variable], dict):
        raise ValueError(f"{path}: expected scalar struct {variable!r}")
    return loaded[variable]


def _split_masks(time: np.ndarray) -> dict:
    # Exact elapsed-time comparisons; no boundary epsilon or sample adjustment.
    return {
        "early_le_late_gt": ((time >= 0) & (time <= 2), (time > 2) & (time <= 10)),
        "early_lt_late_ge": ((time >= 0) & (time < 2), (time >= 2) & (time <= 10)),
        "inclusive_both": ((time >= 0) & (time <= 2), (time >= 2) & (time <= 10)),
    }


def _rmse(error: np.ndarray) -> float:
    if error.size == 0:
        raise ValueError("Cannot calculate an RMSE over an empty interval")
    return float(np.sqrt(np.mean(np.square(error), dtype=np.float64)))


def calculate_metrics(
    actual_result_mat: str | Path,
    flat_reference_mat: str | Path,
    condition: dict,
    variant: str,
    result_variable: str = "Result",
) -> dict:
    """Compare one actual trajectory to every recorded reference timestamp."""
    if set(condition) != set(CONDITION_KEYS) or not all(np.isfinite(list(condition.values()))):
        raise ValueError(f"Explicit condition must supply exactly {CONDITION_KEYS}")
    if variant not in ("baseline", "revised"):
        raise ValueError("variant must be baseline or revised")
    actual = _load_struct(actual_result_mat, result_variable)
    flat = _load_struct(flat_reference_mat, "ref")
    if not isinstance(actual.get("conditions"), dict):
        raise ValueError("Result lacks embedded conditions")
    actual_check = _condition_check(actual["conditions"], condition)
    if actual_check["status"] != "verified":
        raise ValueError(f"Actual result condition is not verified: {actual_check}")
    if "variant" in actual and str(actual["variant"]) != variant:
        raise ValueError("Actual result variant differs from the explicit variant")
    info = flat.get("info", {})
    if not isinstance(info, dict):
        raise ValueError("Flat ref.info must be a scalar structure")
    reference_check = _condition_check(info, condition)
    model_time = _times(actual["time_s"], "Result.time_s")
    absolute_time = _times(flat["time_absolute_s"], "ref.time_absolute_s")
    reference_time = absolute_time - absolute_time[0]
    for key, observed in (("start_s", absolute_time[0]), ("stop_s", absolute_time[-1])):
        if key in info and not math.isclose(float(info[key]), float(observed), rel_tol=0, abs_tol=1e-6):
            raise ValueError(f"Reference {key} disagrees with its timestamps")
    raw = np.asarray(flat["raw_export_data"])
    if np.iscomplexobj(raw) or raw.ndim != 2 or raw.dtype.kind not in "biuf":
        raise ValueError("ref.raw_export_data must be a real numeric matrix")
    raw = np.asarray(raw, dtype=np.float64)
    if raw.shape == (72, len(reference_time)) and raw.shape[0] != len(reference_time):
        raw = raw.T
    if raw.shape != (len(reference_time), 72) or not np.all(np.isfinite(raw)):
        raise ValueError("Reference must contain finite N-by-72 EXPORT channels")
    # np.interp would clamp outside the model range, so reject it beforehand.
    if reference_time[0] < model_time[0] or reference_time[-1] > model_time[-1]:
        raise ValueError("Actual trajectory does not cover all reference timestamps; extrapolation forbidden")
    response = actual.get("response")
    if not isinstance(response, dict):
        raise ValueError("Result.response must be a structure")
    masks = _split_masks(reference_time)
    metrics = {}
    for signal, (column, scale, unit) in SIGNALS.items():
        prediction = _vector(response[signal], f"Result.response.{signal}")
        if len(prediction) != len(model_time):
            raise ValueError(f"Result.response.{signal} has an incorrect length")
        truth = raw[:, column] * scale
        interpolated = np.interp(reference_time, model_time, prediction)
        error = interpolated - truth
        maximum_index = int(np.argmax(np.abs(error)))
        metric = {
            "unit": unit,
            "reference_export_column": column + 1,
            "reference_scale": scale,
            "sample_count": len(error),
            "rmse": _rmse(error),
            "max_abs_error": float(abs(error[maximum_index])),
            "time_of_max_error_s": float(reference_time[maximum_index]),
            "reference_final": float(truth[-1]),
            "model_final": float(interpolated[-1]),
            "splits": {},
        }
        for name, (early, late) in masks.items():
            metric["splits"][name] = {
                "early_sample_count": int(np.sum(early)),
                "late_sample_count": int(np.sum(late)),
                "rmse_0_2": _rmse(error[early]),
                "rmse_2_10": _rmse(error[late]),
            }
        metrics[signal] = metric
    # Retain calibration provenance only. Cached historical metric values in
    # the profile are never used or copied into independently computed data.
    calibration = actual.get("calibration", {})
    calibration_metadata = {k: calibration[k] for k in (
        "enabled", "training_speeds_kmh", "source_conditions", "training_tire_wear",
        "profile_file", "profile_schema", "independent_validation_of_updated_profile",
    ) if k in calibration} if isinstance(calibration, dict) else {}
    return _jsonable({
        "schema": "HMC_INDEPENDENT_METRICS_V1",
        "condition": condition,
        "variant": variant,
        "actual_result": {"path": str(Path(actual_result_mat).resolve()), "sha256": sha256(actual_result_mat)},
        "reference": {"path": str(Path(flat_reference_mat).resolve()), "sha256": sha256(flat_reference_mat),
                      "embedded_condition_check": reference_check, "info": info,
                      "provenance": flat.get("provenance", {})},
        "actual_condition_check": actual_check,
        "execution_environment": actual.get("execution_environment", {}),
        "execution_runtime": actual.get("execution_runtime", actual.get("execution_environment", {}).get("runtime", "not recorded in Result")),
        "actual_engine": actual.get("engine", "not recorded in Result"),
        "actual_conditions": actual["conditions"],
        "calibration": calibration_metadata,
        "time": {"sample_count": len(reference_time), "absolute_first_s": absolute_time[0],
                 "absolute_last_s": absolute_time[-1], "elapsed_first_s": reference_time[0],
                 "elapsed_last_s": reference_time[-1], "model_sample_count": len(model_time),
                 "minimum_dt_s": np.min(np.diff(reference_time)), "maximum_dt_s": np.max(np.diff(reference_time)),
                 "operation": "Subtract recorded first timestamp only; no fitted time shift"},
        "method": {"interpolation": "linear at every reference timestamp; no extrapolation",
                   "rmse": "sqrt(mean((prediction-reference)^2)); unweighted samples",
                   "maximum": "max(abs(prediction-reference)); first maximum timestamp",
                   "amplitude_operation": "EXPORT unit conversion only", "removed_samples": 0,
                   "default_split": "early_le_late_gt", "alternate_splits_recorded": list(SPLITS)},
        "signals": metrics,
    })


def read_expected(path: str | Path) -> list[dict]:
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("Expected metrics CSV is empty")
    return rows


def _csv_condition(row: dict) -> dict:
    return {"speed_kmh": float(row["Speed_kmh"]), "mu": float(row["RoadMu"]),
            "steer_command_deg": float(row["SteerCommand_deg"])}


def audit_expected_csv(path: str | Path) -> dict:
    rows = read_expected(path)
    reduction_errors, split_residuals, alternative_residuals = [], [], []
    flag_mismatches = []
    for index, row in enumerate(rows):
        b, r = float(row["Baseline_RMSE"]), float(row["Revised_RMSE"])
        reduction_errors.append(abs(100 * (b - r) / b - float(row["RMSE_Reduction_pct"])))
        for flag, actual in (("RMSE_Improved", r < b), ("MaxAbs_Improved", float(row["Revised_MaxAbsError"]) < float(row["Baseline_MaxAbsError"]))):
            if actual != (row[flag].lower() == "true"):
                flag_mismatches.append({"row": index + 2, "field": flag})
        count = int(row["SampleCount"])
        for variant in ("Baseline", "Revised"):
            total = float(row[f"{variant}_RMSE"]) ** 2 * count
            early = float(row[f"{variant}_RMSE_0_2"]) ** 2
            late = float(row[f"{variant}_RMSE_2_10"]) ** 2
            split_residuals.append(abs(2001 * early + 8000 * late - total) / max(abs(total), 1e-300))
            alternative_residuals.append(abs(2000 * early + 8001 * late - total) / max(abs(total), 1e-300))
    return {
        "expected_csv_sha256": sha256(path), "row_count": len(rows),
        "conditions": [dict(zip(CONDITION_KEYS, values)) for values in sorted({tuple(_csv_condition(r).values()) for r in rows})],
        "signals": sorted({r["Signal"] for r in rows}),
        "sample_counts": sorted({int(r["SampleCount"]) for r in rows}),
        "maximum_reduction_percentage_residual": max(reduction_errors),
        "boolean_mismatches": flag_mismatches,
        "improved_counts": {name: sum(r[name].lower() == "true" for r in rows)
                            for name in ("RMSE_Improved", "MaxAbs_Improved")},
        "assessment_labels": sorted({r["Assessment"] for r in rows}),
        "split_audit": {
            "assumption": "Uniform elapsed 0:0.001:10 grid, 10001 samples",
            "early_le_2_late_gt_2_maximum_relative_sse_residual": max(split_residuals),
            "early_lt_2_late_ge_2_maximum_relative_sse_residual": max(alternative_residuals),
            "interpretation": "CSV is consistent with 2001 early and 8000 late samples; t=2 belongs to early only",
        },
        "limits": ["CSV does not identify source/model hashes, runtime, wear, initialization, CDC settings, or full calibration configuration",
                   "File names cannot establish CarSim conditions; embedded info is checked separately"],
    }


def compare_expected(baseline: dict, revised: dict, expected_csv: str | Path,
                     atol: float = 1e-9, rtol: float = 1e-5,
                     split: str = "early_le_late_gt") -> dict:
    """Compare computed baseline/revised metrics to CSV; never fit to CSV."""
    if baseline["variant"] != "baseline" or revised["variant"] != "revised":
        raise ValueError("Provide baseline and revised results in their respective arguments")
    if baseline["condition"] != revised["condition"]:
        raise ValueError("Variants have different explicit conditions")
    if baseline["reference"]["sha256"] != revised["reference"]["sha256"]:
        raise ValueError("Variants used different reference files")
    if split not in SPLITS or atol < 0 or rtol < 0:
        raise ValueError("Invalid split convention or tolerances")
    condition = baseline["condition"]
    matched = [r for r in read_expected(expected_csv)
               if _condition_check(_csv_condition(r), condition)["status"] == "verified"]
    expected_by_signal = {r["Signal"]: r for r in matched}
    if len(matched) != 6 or set(expected_by_signal) != set(SIGNALS):
        raise ValueError("Expected CSV must contain exactly six signal rows for the explicit condition")
    signal_reports = []
    all_checks = []
    for signal in SIGNALS:
        b, r, row = baseline["signals"][signal], revised["signals"][signal], expected_by_signal[signal]
        reduction = 100 * (b["rmse"] - r["rmse"]) / b["rmse"] if b["rmse"] != 0 else None
        computed = {
            "Baseline_RMSE": b["rmse"], "Revised_RMSE": r["rmse"],
            "RMSE_Reduction_pct": reduction,
            "Baseline_MaxAbsError": b["max_abs_error"], "Revised_MaxAbsError": r["max_abs_error"],
            "Baseline_RMSE_0_2": b["splits"][split]["rmse_0_2"],
            "Revised_RMSE_0_2": r["splits"][split]["rmse_0_2"],
            "Baseline_RMSE_2_10": b["splits"][split]["rmse_2_10"],
            "Revised_RMSE_2_10": r["splits"][split]["rmse_2_10"],
        }
        checks = {}
        for field, actual in computed.items():
            expected = float(row[field])
            difference = None if actual is None else abs(actual - expected)
            limit = atol + rtol * abs(expected)
            passed = actual is not None and difference <= limit
            checks[field] = {"actual": actual, "expected": expected, "absolute_error": difference,
                             "relative_error": None if difference is None or expected == 0 else difference / abs(expected),
                             "tolerance_limit": limit, "passed": passed}
            all_checks.append(passed)
        flags = {"RMSE_Improved": r["rmse"] < b["rmse"],
                 "MaxAbs_Improved": r["max_abs_error"] < b["max_abs_error"],
                 "ReferenceAvailable": True}
        for field, actual in flags.items():
            expected = row[field].lower() == "true"
            checks[field] = {"actual": actual, "expected": expected, "passed": actual == expected}
            all_checks.append(actual == expected)
        count_pass = b["sample_count"] == r["sample_count"] == int(row["SampleCount"])
        unit_pass = b["unit"] == r["unit"] == row["Unit"]
        checks["SampleCount"] = {"actual": b["sample_count"], "expected": int(row["SampleCount"]), "passed": count_pass}
        checks["Unit"] = {"actual": b["unit"], "expected": row["Unit"], "passed": unit_pass}
        all_checks.extend((count_pass, unit_pass))
        signal_reports.append({"signal": signal, "checks": checks, "expected_assessment": row["Assessment"],
                               "passed": all(c["passed"] for c in checks.values())})
    condition_status = revised["reference"]["embedded_condition_check"]["status"]
    numeric_match = all(all_checks)
    return {
        "schema": "HMC_EXPECTED_METRICS_COMPARISON_V1", "condition": condition,
        "reference_condition_status": condition_status,
        "numeric_match": numeric_match, "overall_match": numeric_match and condition_status == "verified",
        "tolerances": {"atol": atol, "rtol": rtol, "criterion": "abs(actual-expected) <= atol + rtol*abs(expected)",
                       "status": "Initial cross-runtime comparison thresholds; not fitted to outcomes"},
        "split_convention": split, "signals": signal_reports,
        "expected_csv_sha256": sha256(expected_csv),
        "baseline_result_sha256": baseline["actual_result"]["sha256"],
        "revised_result_sha256": revised["actual_result"]["sha256"],
        "reference_sha256": revised["reference"]["sha256"],
        "interpretation": "Implementation/result comparison at declared calibration conditions; no independent held-out accuracy claim",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    audit = sub.add_parser("audit-expected")
    audit.add_argument("--expected", required=True)
    audit.add_argument("--output", required=True)
    compute = sub.add_parser("compute")
    compute.add_argument("--actual-result", required=True)
    compute.add_argument("--flat-reference", required=True)
    compute.add_argument("--result-variable", default="Result")
    compute.add_argument("--speed", required=True, type=float)
    compute.add_argument("--mu", required=True, type=float)
    compute.add_argument("--steer", required=True, type=float)
    compute.add_argument("--variant", required=True, choices=("baseline", "revised"))
    compute.add_argument("--output", required=True)
    compare = sub.add_parser("compare")
    compare.add_argument("--baseline", required=True)
    compare.add_argument("--revised", required=True)
    compare.add_argument("--expected", required=True)
    compare.add_argument("--atol", type=float, default=1e-9)
    compare.add_argument("--rtol", type=float, default=1e-5)
    compare.add_argument("--split", choices=SPLITS, default="early_le_late_gt")
    compare.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.command == "audit-expected":
        report = audit_expected_csv(args.expected)
    elif args.command == "compute":
        condition = {"speed_kmh": args.speed, "mu": args.mu, "steer_command_deg": args.steer}
        report = calculate_metrics(args.actual_result, args.flat_reference, condition, args.variant, args.result_variable)
    else:
        baseline = json.loads(Path(args.baseline).read_text())
        revised = json.loads(Path(args.revised).read_text())
        report = compare_expected(baseline, revised, args.expected, args.atol, args.rtol, args.split)
    Path(args.output).write_text(json.dumps(_jsonable(report), indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(Path(args.output).resolve()),
                      "condition": report.get("condition"), "overall_match": report.get("overall_match")}, allow_nan=False))


if __name__ == "__main__":
    main()
