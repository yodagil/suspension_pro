#!/usr/bin/env python3
"""Audit, plot and assemble the 150 appended steering-3.5-degree regions.

This module reads completed atomic MAT files; it never runs vehicle dynamics.
The extension aggregate is refused until all IDs1051..1200 pass.
It does not finalize or claim completion of the combined 1,200-case study.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
from datetime import datetime, timezone

import numpy as np
from scipy.io import loadmat, savemat

TOTAL = 150
PER_WEAR = 30
JOB_START = 1051
JOB_END = 1200
JOB_IDS = range(JOB_START, JOB_END + 1)
COMBINED_TOTAL = 1200
BASE = Path(__file__).resolve().parents[1] / "carsim_fullcar_steer35_150"
WEARS = np.array([[0, 0, 0, 0], [25, 25, 0, 0], [50, 50, 25, 25],
                  [0, 0, 25, 25], [25, 25, 50, 50]], dtype=float)
_BINDINGS_CACHE = {}


def _now():
    return datetime.now(timezone.utc).isoformat()


def _sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def _atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, default=_json_value,
                                   allow_nan=False) + "\n")
    os.replace(temporary, path)


def expected_case(job_id):
    """Appended IDs: wear, speed, friction, with steering fixed at3.5deg.

    Original IDs1..1050 remain untouched. condition_id is the extension-local
    index1..30 for a wear pattern, not a renumbered original condition.
    """
    raw_job = job_id
    job_id = int(job_id)
    if job_id != raw_job or not JOB_START <= job_id <= JOB_END:
        raise ValueError("job_id must be an integer in1051..1200")
    w, condition = divmod(job_id - JOB_START, PER_WEAR)
    speed, friction = divmod(condition, 5)
    return {"job_id": job_id, "condition_id": condition + 1,
            "wear_case_id": w + 1, "speed_kmh": 50.0 + 10 * speed,
            "mu": [0.9, 0.8, 0.7, 0.6, 0.5][friction],
            "steer_target_deg": 3.5, "wear": WEARS[w].tolist()}


def _expected_bindings(base, job_id, require=False):
    """Bind artifacts to the published context index and frozen supervisor run."""
    base = Path(base)
    index_file, run_file = base / "case_index.json", base / "run_binding.json"
    fingerprints = tuple((str(p), p.stat().st_size, p.stat().st_mtime_ns)
                         if p.exists() else (str(p), None, None)
                         for p in (index_file, run_file))
    cached = _BINDINGS_CACHE.get(str(base))
    if cached is None or cached[0] != fingerprints:
        index = json.loads(index_file.read_text()) if index_file.exists() else {}
        run = json.loads(run_file.read_text()) if run_file.exists() else {}
        rows = {int(row["job_id"]): row for row in index.get("cases", [])}
        cached = (fingerprints, index, run, rows)
        _BINDINGS_CACHE[str(base)] = cached
    _, index, run, rows = cached
    if require and (not index or not run or set(rows) != set(JOB_IDS)):
        raise ValueError("Finalization requires the complete context index and frozen run binding")
    expected = expected_case(job_id)
    if index:
        row = rows.get(job_id)
        if not row or row.get("status") != "prepared" or not row.get("context_sha256"):
            raise ValueError(f"Job {job_id} lacks a prepared indexed context")
        for key, row_key in (("condition_id", "condition_id"), ("wear_case_id", "wear_case_id"),
                             ("speed_kmh", "speed_kmh"), ("mu", "mu"),
                             ("steer_target_deg", "command_steer_deg")):
            if not np.isclose(float(row[row_key]), expected[key], rtol=0, atol=1e-12):
                raise ValueError(f"Indexed context differs from appended mapping: {key}")
        if not np.array_equal(np.asarray(row["wear"]).ravel(), expected["wear"]):
            raise ValueError("Indexed context wear differs from appended mapping")
        if index["reference_mode"] != "model_actual_front_trim":
            raise ValueError("The full batch must use uniform model_actual_front_trim references")
        expected.update({"context_sha256": row["context_sha256"],
                         "batch_signature": index["batch_signature"],
                         "reference_mode": index["reference_mode"]})
    if run:
        if index and (run["batch_signature"] != index["batch_signature"] or
                      run["reference_mode"] != index["reference_mode"]):
            raise ValueError("Context index and frozen run binding disagree")
        files = run.get("files_sha256", run.get("files", {}))
        native = [value for name, value in files.items() if name.endswith("/hmc_region_mex.mex")]
        if len(native) != 1:
            raise ValueError("Frozen run binding lacks one unambiguous region engine")
        expected.update({"native_binary_sha256": native[0],
                         "batch_signature": run["batch_signature"],
                         "reference_mode": run["reference_mode"]})
    return expected


def _case(data):
    case = data["CASE"]
    if not isinstance(case, dict):
        flat = np.asarray(case, dtype=object).ravel()
        if len(flat) != 1 or not isinstance(flat[0], dict):
            raise ValueError("Expected exactly one RESEARCH2D.CASE")
        case = flat[0]
    return case


def _logical(value, shape, name):
    value = np.asarray(value)
    if value.shape != shape or not np.all(np.isfinite(value)) or not np.all(np.isin(value, [0, 1])):
        raise ValueError(f"Invalid {name}: expected finite logical array {shape}, got {value.shape}")
    return value.astype(bool)


def _scalar(value, name):
    if np.asarray(value).size != 1 or not np.isfinite(float(value)):
        raise ValueError(f"Invalid scalar {name}")
    return float(value)


def _first(objects, names, default=None):
    for obj in objects:
        for name in names:
            if name in obj:
                return obj[name]
    return default


def _touches(mask):
    return bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any())


def _vertices_to_cells(points):
    return (points[:-1, :-1] & points[1:, :-1] &
            points[:-1, 1:] & points[1:, 1:])


def _trim(case):
    trim = case["trim"]
    if "T" in trim:
        trim = trim["T"]
    state = np.asarray(trim["x"], dtype=float).ravel()
    if state.shape != (34,) or not np.all(np.isfinite(state)):
        raise ValueError("Expected finite original 34-state own trim")
    if not bool(trim.get("root_found", False)) or not bool(trim.get("usable", False)):
        raise ValueError("A failed own trim cannot be counted as a completed region")
    return trim, state


def audit_result(path, expected_spec=None):
    """Validate a completed region and return its small JSON-compatible summary.

    expected_spec may use canonical names or the original CaseSpec names.
    Extra expected input/context/binary signatures, when supplied, are checked.
    """
    path = Path(path)
    loaded = loadmat(path, simplify_cells=True, verify_compressed_data_integrity=True)
    data = loaded["RESEARCH2D"]
    case, meta = _case(data), data["meta"]
    objects = [case, meta, loaded.get("CaseSpec", {})]
    job = _first(objects, ["job_id", "JobID", "source_result_job_id"])
    if job is None:
        raise ValueError("Result lacks its original job ID")
    job = int(_scalar(job, "job_id"))
    spec = expected_case(job)
    automatic_expected = _expected_bindings(path.parent.parent, job)
    if expected_spec:
        for key in ("input_signature", "batch_signature", "context_sha256", "native_binary_sha256", "reference_mode"):
            if key in automatic_expected and key in expected_spec and automatic_expected[key] != expected_spec[key]:
                raise ValueError(f"Caller and stored run binding disagree: {key}")
        automatic_expected.update(expected_spec)
    expected_spec = automatic_expected
    filename = re.fullmatch(r"case_(\d{4})\.mat", path.name)
    if filename and int(filename[1]) != job:
        raise ValueError("Filename and saved job ID differ")
    aliases = {"job_id": ["job_id", "JobID", "source_result_job_id"],
               "condition_id": ["condition_id", "ConditionID"],
               "wear_case_id": ["wear_case_id", "WearCaseID"],
               "speed_kmh": ["speed_kmh", "Speed_kmh"],
               "mu": ["mu", "RoadMu"],
               "steer_target_deg": ["steer_target_deg", "command_steer_deg", "SteerCommand_deg"]}
    for key, names in aliases.items():
        actual = _first(objects, names)
        if actual is None and key in ("condition_id", "wear_case_id"):
            # Older three-condition comparison files predate batch metadata.
            actual = spec[key]
        if actual is None or not np.isclose(_scalar(actual, key), spec[key], rtol=0, atol=1e-12):
            raise ValueError(f"Appended case mapping mismatch for {key}")
    wear = np.asarray(_first(objects, ["wear"]), dtype=float).ravel()
    if wear.shape != (4,) or not np.array_equal(wear, spec["wear"]):
        raise ValueError("Appended case mapping mismatch for wear")
    if meta.get("model") != "carsim_fullcar":
        raise ValueError("Unexpected vehicle model label")
    if expected_spec:
        for key, names in aliases.items():
            expected = _first([expected_spec], names)
            if expected is not None and not np.isclose(float(expected), spec[key], rtol=0, atol=1e-12):
                raise ValueError(f"Expected CaseSpec differs: {key}")
        expected_wear = expected_spec.get("wear")
        if expected_wear is not None and not np.array_equal(wear, np.asarray(expected_wear).ravel()):
            raise ValueError("Expected wear differs")
        for key in ("input_signature", "batch_signature", "context_sha256", "native_binary_sha256", "reference_mode"):
            if key in expected_spec and _first(objects, [key]) != expected_spec[key]:
                raise ValueError(f"Expected execution binding differs: {key}")

    vc = np.asarray(case["grid"]["vc"], dtype=float).ravel()
    r = np.asarray(case["grid"]["r"], dtype=float).ravel()
    if (vc.shape != (61,) or r.shape != (121,) or
            not np.allclose(vc, np.linspace(-3, 3, 61), rtol=0, atol=2e-12) or
            not np.allclose(r, np.linspace(-1.5, 1.5, 121), rtol=0, atol=2e-12)):
        raise ValueError("Grid differs from the validated 61 by 121 grid")
    masks = case["masks"]
    reference = _logical(masks.get("reference", case["handling"]["mask"]), (61, 121), "reference")
    handling = _logical(case["handling"]["mask"], (61, 121), "handling.mask")
    if not np.array_equal(reference, handling):
        raise ValueError("Saved reference masks disagree")
    final = _logical(masks["final"], (61, 121, 2), "final")
    if np.any(final & ~reference[:, :, None]):
        raise ValueError("An untested outside-H node was marked as passing")
    if "guarded_final" in masks:
        guarded = _logical(masks["guarded_final"], final.shape, "guarded_final")
        if np.any(guarded & ~final):
            raise ValueError("Guarded pass is not a subset of main pass")
    horizons = np.asarray(meta.get("horizons_s", data["cfg"]["horizons"]), dtype=float).ravel()
    if not np.array_equal(horizons, [5, 10]):
        raise ValueError("Expected saved 5 and 10 second horizons")
    if not np.isclose(meta["integration_dt_s"], 0.0005, rtol=0, atol=1e-15) or not np.isclose(meta["observation_dt_s"], 0.001, rtol=0, atol=1e-15):
        raise ValueError("Integration or observation interval changed")

    region = case["region"]
    cells = _logical(region["cell_mask"], (60, 120), "cell_mask")
    candidates = _vertices_to_cells(final[:, :, -1])
    if np.any(cells & ~candidates):
        raise ValueError("Accepted cell has a failed vertex")
    ids = np.flatnonzero(candidates.ravel(order="F"))
    vi, ri = np.unravel_index(ids, candidates.shape, order="F")
    expected_centers = np.column_stack(((vc[vi] + vc[vi + 1]) / 2,
                                        (r[ri] + r[ri + 1]) / 2))
    centers = np.asarray(region.get("center_points", []), dtype=float).reshape(-1, 2)
    passes = np.asarray(region.get("center_pass", [])).ravel()
    if (centers.shape != expected_centers.shape or
            not np.allclose(centers, expected_centers, rtol=0, atol=2e-12) or
            passes.shape != (len(ids),) or not np.all(np.isin(passes, [0, 1]))):
        raise ValueError("Candidate cells lack their actual matching center tests")
    expected_cells = np.zeros_like(cells)
    expected_cells[vi, ri] = passes.astype(bool)
    if not np.array_equal(cells, expected_cells):
        raise ValueError("Cell mask differs from four-vertex plus actual-center policy")
    center_raw = region.get("center_raw", {})
    if len(ids) and isinstance(center_raw, dict) and "final" in center_raw:
        center_final = np.asarray(center_raw["final"]).reshape(len(ids), 2)
        if not np.array_equal(center_final[:, -1], passes.astype(bool)):
            raise ValueError("Center pass flags differ from independently integrated center results")
    weights = np.diff(vc)[:, None] * np.diff(r)[None, :]
    area = float(weights[cells].sum())
    if not np.isclose(_scalar(region["cell_area"], "cell_area"), area, rtol=2e-12, atol=1e-13):
        raise ValueError("Saved region area disagrees with the exact cell mask")
    for key, count in (("accepted_cells", cells.sum()), ("candidate_cells", len(ids))):
        if key in region and int(region[key]) != int(count):
            raise ValueError(f"Saved {key} disagrees with masks")
    _, trim_state = _trim(case)
    trim_inside = bool(case.get("trim_in_reference", case["classification_configuration"].get("trim_inside_reference", True)))
    if not trim_inside and (final.any() or cells.any()):
        raise ValueError("Outside-H own trim cannot have a nonempty main region")
    completion_kind = meta.get("completion_kind", "legacy_integrated")
    if completion_kind in ("trim_outside_reference", "empty_reference_grid"):
        if ((completion_kind == "trim_outside_reference" and trim_inside) or
                (completion_kind == "empty_reference_grid" and reference.any()) or
                final.any() or cells.any() or int(meta["fresh_vertex_trajectories"]) != 0 or
                int(meta["fresh_center_trajectories"]) != 0):
            raise ValueError("Unintegrated main-region rejection lacks its necessary-condition proof")
    elif completion_kind == "integrated":
        if (int(meta["fresh_vertex_trajectories"]) != int(reference.sum()) or
                int(meta["fresh_center_trajectories"]) != len(ids)):
            raise ValueError("Fresh trajectory counts do not match tested vertices and centers")
    elif completion_kind != "legacy_integrated":
        raise ValueError(f"Unexpected completion kind: {completion_kind}")
    for obj, key, expected in ((meta, "reference_grid_truncated", _touches(reference)),
                               (meta, "stable_grid_truncated", _touches(cells)),
                               (region, "grid_truncated", _touches(cells)),
                               (case["handling"], "grid_truncated", _touches(reference))):
        if key in obj and bool(obj[key]) != expected:
            raise ValueError(f"Saved grid truncation diagnostic differs: {key}")
    raw = case.get("raw", {})
    if isinstance(raw, dict) and "initial_grid_linear_index_1based" in raw:
        initial_ids = np.asarray(raw["initial_grid_linear_index_1based"], dtype=int).ravel()
        expected_ids = np.flatnonzero(reference.ravel(order="F")) + 1
        if not np.array_equal(initial_ids, expected_ids):
            raise ValueError("Native evaluated-node indices differ from H")
        point_final = np.asarray(raw["final"]).reshape(len(initial_ids), 2)
        if not np.array_equal(point_final, final.reshape(-1, 2, order="F")[initial_ids - 1]):
            raise ValueError("Native point results differ from mapped final masks")
        if completion_kind in ("trim_outside_reference", "empty_reference_grid"):
            if np.asarray(raw.get("trajectory_evaluated", [])).any():
                raise ValueError("Skipped trajectories were incorrectly marked as integrated")
            for key in ("recoverable", "model_safe", "yaw_ok", "ay_ok", "terminal_error"):
                values = np.asarray(raw.get(key, []), dtype=float)
                if values.size and not np.all(np.isnan(values)):
                    raise ValueError(f"Skipped auxiliary {key} must remain unknown/NaN")
    summary = dict(spec)
    summary.update({"source_file": str(path), "source_sha256": _sha(path),
                    "reference_nodes": int(reference.sum()),
                    "final_nodes_5s": int(final[:, :, 0].sum()),
                    "final_nodes_10s": int(final[:, :, 1].sum()),
                    "candidate_cells": len(ids), "accepted_cells": int(cells.sum()),
                    "cell_area": area, "trim_in_reference": trim_inside,
                    "main_region_known_empty": completion_kind in ("trim_outside_reference", "empty_reference_grid") or not trim_inside,
                    "completion_kind": str(completion_kind),
                    "reference_grid_truncated": _touches(reference),
                    "grid_truncated": _touches(cells),
                    "trim_vy_mps": float(trim_state[0]), "trim_r_radps": float(trim_state[1]),
                    "wall_seconds": float(meta.get("wall_seconds", 0)),
                    "reference_mode": str(_first(objects, ["reference_mode"], "legacy_common_CarSim_H")),
                    "native_binary_sha256": str(_first(objects, ["native_binary_sha256"], "")),
                    "input_signature": str(_first(objects, ["input_signature"], "")),
                    "batch_signature": str(_first(objects, ["batch_signature"], "")),
                    "context_sha256": str(_first(objects, ["context_sha256"], "")),
                    "outside_reference_semantics": "untested, not demonstrated unstable"})
    return summary


def render_case(path, output_path=None):
    """Render exact accepted-cell boundaries on the fixed original grid."""
    from compare_regions import Region, boundary_paths
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection, PolyCollection
    from matplotlib.lines import Line2D

    path = Path(path)
    summary = audit_result(path)
    case = _case(loadmat(path, simplify_cells=True)["RESEARCH2D"])
    vc, r = np.asarray(case["grid"]["vc"]).ravel(), np.asarray(case["grid"]["r"]).ravel()
    cells = np.asarray(case["region"]["cell_mask"], dtype=bool)
    reference = np.asarray(case["handling"]["mask"], dtype=bool)
    hcells = _vertices_to_cells(reference)
    _, trim = _trim(case)
    dummy = np.zeros_like(reference)
    accepted = Region("carsim_fullcar", path, vc, r, cells, dummy, reference)
    hregion = Region("reference", path, vc, r, hcells, dummy, reference)
    fig, ax = plt.subplots(figsize=(9, 7))
    polygons = [np.array([[vc[i], r[j]], [vc[i + 1], r[j]],
                          [vc[i + 1], r[j + 1]], [vc[i], r[j + 1]]])
                for i, j in np.argwhere(cells)]
    if polygons:
        ax.add_collection(PolyCollection(polygons, facecolor="#0875cf", alpha=0.16,
                                         edgecolor="none"))
    if hcells.any():
        ax.add_collection(LineCollection(boundary_paths(hregion), colors="#7b8490",
                                         linewidths=1.15, linestyles="dashed"))
    if cells.any():
        ax.add_collection(LineCollection(boundary_paths(accepted), colors="#0875cf",
                                         linewidths=1.8))
    ax.plot(trim[0], trim[1], "o", color="#be2626", markersize=5)
    ax.set(xlim=(vc[0], vc[-1]), ylim=(r[0], r[-1]),
           xlabel=r"Initial lateral-velocity command $v_{y,0}$ (m/s)",
           ylabel=r"Initial yaw-rate command $r_0$ (rad/s)")
    ax.grid(alpha=0.22)
    ax.axhline(0, color="#aeb6c2", linewidth=0.7, zorder=0)
    ax.axvline(0, color="#aeb6c2", linewidth=0.7, zorder=0)
    wear = "/".join(f"{v:g}" for v in summary["wear"])
    ax.set_title(f"carsim_fullcar | case ID {summary['job_id']}/1200 | steering3.5 extension {summary['job_id'] - JOB_START + 1}/150\n"
                 f"{summary['speed_kmh']:g} km/h | mu={summary['mu']:g} | steering={summary['steer_target_deg']:g} deg | wear {wear}\n"
                 f"wear case {summary['wear_case_id']}/5, extension condition index {summary['condition_id']}/30 | horizon 10 s", fontsize=11)
    ax.legend(handles=[Line2D([], [], color="#0875cf", label="Accepted-cell region"),
                       Line2D([], [], color="#7b8490", linestyle="--", label="Reference H grid cells"),
                       Line2D([], [], marker="o", linestyle="none", color="#be2626", label="Own numerical trim")], loc="upper right", fontsize=9)
    note = f"{summary['accepted_cells']} cells; area {summary['cell_area']:.6g} (m/s)·(rad/s)"
    if summary["main_region_known_empty"]:
        note += ("\nEmpty: no grid nodes lie inside H." if summary["completion_kind"] == "empty_reference_grid"
                 else "\nEmpty by policy: own trim is outside H.")
    if summary["grid_truncated"]:
        note += "\nRegion reaches the grid boundary."
    ax.text(0.02, 0.03, note, transform=ax.transAxes, va="bottom", fontsize=9,
            bbox={"facecolor": "white", "alpha": 0.88, "edgecolor": "#d5dbe3"})
    fig.text(0.5, 0.025, "Exact cell union; no hull or shifting. Outside-H nodes are untested.\n"
             "Finite-time sampled recovery, not an invariant-set certificate. Fixed original grid extent.",
             ha="center", fontsize=8, color="#475569")
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    output_path = Path(output_path) if output_path else path.parent.parent / "images" / f"case_{summary['job_id']:04d}.png"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(output_path.name + ".tmp")
    fig.savefig(temporary, format="png", dpi=150)
    plt.close(fig)
    os.replace(temporary, output_path)
    _atomic_json(output_path.with_suffix(".json"), {"job_id": summary["job_id"],
                 "source_sha256": summary["source_sha256"], "image_sha256": _sha(output_path),
                 "created_utc": _now(), "fixed_axes": [-3, 3, -1.5, 1.5],
                 "geometry": "exact accepted-cell union and four-vertex H grid-cell boundary"})
    return output_path


def _failure_ids(base):
    failed = set()
    for directory in ("errors", "failures", "preparation_errors"):
        for path in (base / directory).glob("*.json"):
            match = re.search(r"case_(\d{4})", path.name)
            if match and JOB_START <= int(match[1]) <= JOB_END:
                failed.add(int(match[1]))
    return failed


def refresh_progress(base=BASE):
    """Refresh progress, re-auditing only new/changed atomic region files.

    Cache identity uses file size, nanosecond mtime and this audit source SHA.
    Finalization deliberately ignores the cache and reads every file again.
    """
    base = Path(base)
    base.mkdir(parents=True, exist_ok=True)
    cache_path = base / ".artifact_audit_cache.json"
    code_sha = _sha(__file__)
    try:
        cache = json.loads(cache_path.read_text())
    except (OSError, ValueError):
        cache = {}
    if cache.get("code_sha256") != code_sha:
        cache = {"code_sha256": code_sha, "cases": {}}
    found, invalid = {}, {}
    for path in sorted((base / "regions").glob("case_[0-9][0-9][0-9][0-9].mat")):
        job = int(path.stem.split("_")[-1])
        stat = path.stat()
        try:
            expected = _expected_bindings(base, job)
            expected_sha = hashlib.sha256(json.dumps(expected, sort_keys=True).encode()).hexdigest()
        except Exception as exc:
            invalid[job] = f"{type(exc).__name__}: {exc}"
            continue
        fingerprint = [stat.st_size, stat.st_mtime_ns, expected_sha]
        old = cache["cases"].get(str(job), {})
        if old.get("fingerprint") != fingerprint:
            try:
                old = {"fingerprint": fingerprint, "summary": audit_result(path, expected)}
            except Exception as exc:
                old = {"fingerprint": fingerprint, "error": f"{type(exc).__name__}: {exc}"}
            cache["cases"][str(job)] = old
        if "summary" in old:
            found[job] = old["summary"]
        else:
            invalid[job] = old["error"]
    failed = ((_failure_ids(base) | set(invalid)) - set(found)) & set(JOB_IDS)
    wear_progress = []
    for wear_id in range(1, 6):
        ids = set(range(JOB_START + (wear_id - 1) * PER_WEAR, JOB_START + wear_id * PER_WEAR))
        n = len(ids & set(found))
        wear_progress.append({"wear_case_id": wear_id, "wear": WEARS[wear_id - 1].tolist(),
                              "completed": n, "total": PER_WEAR, "progress": f"{n}/{PER_WEAR}",
                              "failed": len(ids & failed)})
    path = base / "progress.json"
    try:
        progress = json.loads(path.read_text())
    except (OSError, ValueError):
        progress = {}
    try:
        finalization = json.loads((base / "finalization.json").read_text())
    except (OSError, ValueError):
        finalization = {}
    region_digest = hashlib.sha256(json.dumps({str(k): found[k]["source_sha256"] for k in sorted(found)}, sort_keys=True).encode()).hexdigest()
    aggregate_complete = (len(found) == TOTAL and finalization.get("status") == "complete" and
                          finalization.get("region_digest_sha256") == region_digest and
                          (base / finalization.get("aggregate_file", "__missing__")).is_file() and
                          (base / finalization.get("qc_csv", "__missing__")).is_file())
    progress.update({"updated_utc": _now(), "total": TOTAL, "completed": len(found),
                     "failed": len(failed), "pending": TOTAL - len(found) - len(failed),
                     "progress": f"{len(found)}/{TOTAL}", "wear_progress": wear_progress,
                     "scope": "steering_3_5_degree_extension_only",
                     "start_id": JOB_START, "end_id": JOB_END,
                     "combined_total": COMBINED_TOTAL, "global_1200_complete_claimed": False,
                     "status": "complete" if aggregate_complete else ("regions_complete" if len(found) == TOTAL else "in_progress"),
                     "images": len(list((base / "images").glob("case_[0-9][0-9][0-9][0-9].png"))),
                     "known_empty_by_trim_policy": sum(s["main_region_known_empty"] for s in found.values()),
                     "invalid_results": {str(k): v for k, v in invalid.items()},
                     "unexpected_result_ids": sorted(set(invalid) - set(JOB_IDS)),
                     "aggregate_complete": aggregate_complete})
    _atomic_json(cache_path, cache)
    _atomic_json(path, progress)
    return progress


def finalize(base=BASE, *, make_images=False):
    """Re-audit all 150 extension cases and write their own aggregate LUT/CSV.

    Image generation is controlled by make_images. It has no
    effect on numerical acceptance or the completed-region count.
    """
    base = Path(base)
    paths = {int(p.stem.split("_")[-1]): p for p in (base / "regions").glob("case_[0-9][0-9][0-9][0-9].mat")}
    missing = sorted(set(JOB_IDS) - set(paths))
    unexpected = sorted(set(paths) - set(JOB_IDS))
    if missing or unexpected:
        raise ValueError(f"Cannot finalize: {len(missing)} missing cases and {len(unexpected)} unexpected IDs")
    summaries = []
    reference = np.empty((61, 121, TOTAL), dtype=bool)
    final = np.empty((61, 121, 2, TOTAL), dtype=bool)
    guarded = np.empty_like(final)
    cells = np.empty((60, 120, TOTAL), dtype=bool)
    trim_states = np.empty((34, TOTAL), dtype=float)
    options, policy, meta_records = [], [], []
    for job in JOB_IDS:
        summary = audit_result(paths[job], _expected_bindings(base, job, require=True))
        summaries.append(summary)
        if make_images:
            image_path = base / "images" / f"case_{job:04d}.png"
            try:
                image_binding = json.loads(image_path.with_suffix(".json").read_text())
            except (OSError, ValueError):
                image_binding = {}
            if (not image_path.exists() or image_binding.get("source_sha256") != summary["source_sha256"] or
                    image_binding.get("image_sha256") != _sha(image_path)):
                render_case(paths[job], image_path)
        data = loadmat(paths[job], simplify_cells=True)["RESEARCH2D"]
        case = _case(data)
        reference[:, :, job - JOB_START] = case["handling"]["mask"]
        final[:, :, :, job - JOB_START] = case["masks"]["final"]
        guarded[:, :, :, job - JOB_START] = case["masks"]["guarded_final"]
        cells[:, :, job - JOB_START] = case["region"]["cell_mask"]
        trim_states[:, job - JOB_START] = _trim(case)[1]
        options.append(json.dumps(case["model_configuration"], default=_json_value, allow_nan=True))
        policy.append(json.dumps(case["classification_configuration"], default=_json_value, allow_nan=True))
        meta_records.append(json.dumps(data["meta"], default=_json_value, allow_nan=True))
    modes = {s["reference_mode"] for s in summaries}
    engines = {s["native_binary_sha256"] for s in summaries}
    batches = {s["batch_signature"] for s in summaries}
    if len(modes) != 1 or modes == {"legacy_common_CarSim_H"}:
        raise ValueError(f"Nonuniform or legacy comparison reference modes: {modes}")
    if len(engines) != 1 or not next(iter(engines)) or len(batches) != 1 or not next(iter(batches)):
        raise ValueError("Aggregate must use one nonempty frozen engine and batch signature")
    columns = ["job_id", "condition_id", "wear_case_id", "speed_kmh", "mu", "steer_target_deg"]
    lut = {"schema": "carsim_fullcar_steer35_finite_time_regions_150_v1", "model": "carsim_fullcar",
           "scope": "steering_3_5_degree_extension_only", "total": TOTAL,
           "start_id": JOB_START, "end_id": JOB_END, "combined_total": COMBINED_TOTAL,
           "global_1200_complete_claimed": False,
           "created_utc": _now(), "grid": {"vc": np.linspace(-3, 3, 61), "r": np.linspace(-1.5, 1.5, 121)},
           "horizons_s": np.array([5.0, 10.0]), "reference_mode": next(iter(modes)),
           "native_binary_sha256": next(iter(engines)), "batch_signature": next(iter(batches)),
           "case_columns": np.array(columns, dtype=object),
           "case_spec": np.array([[s[c] for c in columns] for s in summaries]),
           "wear": np.array([s["wear"] for s in summaries]).T,
           "reference_mask": reference, "final": final, "guarded_final": guarded,
           "cell_mask": cells, "trim_state": trim_states,
           "cell_area": np.array([s["cell_area"] for s in summaries]),
           "accepted_cells": np.array([s["accepted_cells"] for s in summaries]),
           "grid_truncated": np.array([s["grid_truncated"] for s in summaries]),
           "reference_grid_truncated": np.array([s["reference_grid_truncated"] for s in summaries]),
           "source_sha256": np.array([s["source_sha256"] for s in summaries], dtype=object),
           "model_options_json": np.array(options, dtype=object),
           "classification_config_json": np.array(policy, dtype=object),
           "case_metadata_json": np.array(meta_records, dtype=object),
           "outside_reference_semantics": "Untested rather than proven unstable",
           "certified_invariant": False,
           "definition": "Exact accepted-cell unions: all four vertices and actual center pass the 10 s sampled policy"}
    output = base / "carsim_fullcar_steer35_stability_LUT_150.mat"
    temporary = output.with_name(output.name + ".tmp")
    savemat(temporary, {"LUT": lut}, appendmat=False, do_compression=True, long_field_names=True)
    os.replace(temporary, output)
    csv_path = base / "carsim_fullcar_steer35_stability_QC_150.csv"
    temporary = csv_path.with_name(csv_path.name + ".tmp")
    fields = list(summaries[0])
    with temporary.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for summary in summaries:
            row = dict(summary)
            row["wear"] = "/".join(f"{w:g}" for w in row["wear"])
            writer.writerow(row)
    os.replace(temporary, csv_path)
    _atomic_json(base / "finalization.json", {"status": "complete", "completed": TOTAL,
                 "total": TOTAL, "progress": f"{TOTAL}/{TOTAL}", "created_utc": _now(),
                 "scope": "steering_3_5_degree_extension_only",
                 "start_id": JOB_START, "end_id": JOB_END,
                 "combined_total": COMBINED_TOTAL, "global_1200_complete_claimed": False,
                 "aggregate_file": output.name, "aggregate_sha256": _sha(output),
                 "qc_csv": csv_path.name, "qc_csv_sha256": _sha(csv_path),
                 "reference_mode": next(iter(modes)), "native_binary_sha256": next(iter(engines)),
                 "batch_signature": next(iter(batches)), "all_cases_freshly_reaudited": True,
                 "images": len(list((base / "images").glob("case_[0-9][0-9][0-9][0-9].png"))),
                 "images_required": bool(make_images),
                 "image_generation": "enabled_explicitly" if make_images else "not_requested",
                 "all_images_bound_to_source": bool(make_images),
                 "numerically_verified_cases": TOTAL,
                 "region_digest_sha256": hashlib.sha256(json.dumps({str(s['job_id']): s['source_sha256'] for s in summaries}, sort_keys=True).encode()).hexdigest()})
    refresh_progress(base)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=BASE)
    parser.add_argument("--job-id", type=int)
    parser.add_argument("--refresh", action="store_true")
    parser.add_argument("--finalize", action="store_true")
    parser.add_argument("--make-images", action="store_true",
                        help="Explicitly render/check all PNGs during finalization; disabled by default")
    args = parser.parse_args()
    if args.job_id is not None:
        path = args.base / "regions" / f"case_{args.job_id:04d}.mat"
        print(json.dumps({"image": str(render_case(path))}))
    if args.finalize:
        print(json.dumps({"aggregate": str(finalize(args.base, make_images=args.make_images))}))
    if args.refresh or (args.job_id is None and not args.finalize):
        print(json.dumps(refresh_progress(args.base), default=_json_value))


if __name__ == "__main__":
    main()
