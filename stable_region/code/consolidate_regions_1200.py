#!/usr/bin/env python3
"""Consolidate the original 1,050 and appended 150 audited region results.

No dynamics, interpolation, new images, or source-batch edits are performed.
Original IDs and files are retained; complete LUT/QC outputs require all 1,200
numerical results and their source-bound PNGs. --collect-available only records
and copies available verified inputs, with an explicit incomplete manifest.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

import numpy as np
from scipy.io import loadmat, savemat

sys.dont_write_bytecode = True
TOTAL = 1200
WEARS = [[0, 0, 0, 0], [25, 25, 0, 0], [50, 50, 25, 25],
         [0, 0, 25, 25], [25, 25, 50, 50]]
PHYSICS = ("HMC_FullCar_Revised.m", "hmc_native.cpp", "hmc_mex.cpp",
           "hmc_native_run.m", "hmc_apply_calibration.m", "calibration_active.json",
           "data/hmc_data.mat", "tire_fields.inc")
BATCHES = (
    {"name": "original_1050", "folder": "carsim_fullcar_1050", "start": 1, "end": 1050,
     "module": "code/batch_region_artifacts.py", "code": "code",
     "signature": "5d495d2a27cfd1876356b210b5949dbccfc562ecdf43fc06cf0211c7e652a396"},
    {"name": "steer35_150", "folder": "carsim_fullcar_steer35_150", "start": 1051, "end": 1200,
     "module": "code_steer35/steer35_artifacts.py", "code": "code_steer35",
     "signature": "2252921c11106ccbfeb57892d1f82396021e2f2b41e788b839d0fa8762cacf00"},
)
POLICY = {"dt": 0.0005, "observe_dt": 0.001, "horizons": [5, 10],
          "terminal_window": 1, "policy_g": 9.81, "yaw_factor": 0.85,
          "ay_factor": 1, "LTR_limit": 1, "roll_limit_rad": 40 * np.pi / 180,
          "early_reject_final": 1, "window_start_inclusive": 1, "main_requires_ay": 0,
          "terminal_tolerance": [0.1, 0.005, 0.2, 0.2, 0.5, 0.5, 0.5]}
MFY = np.array([
    [1.5662, 1.0278, -.17125, -1.1357, -1.0544, -25.85, 2.1377, 2, 8.675e-4, 7.046e-5, -.007239, -7.442e-5],
    [1.5457, 1.0286, -.16688, -1.439, -1.6063, -26, 1.9164, 2, 1.82e-3, -7.193e-4, 1.802e-3, -1.1497e-2],
    [1.5057, 1.0067, -.2019, -2, -2, -27.27, 1.5963, 2, 2.3735e-3, 2.38e-4, .01487, -.011025]])
REFERENCE_VARIABLES = {"ux", "wear", "mu", "mu_scale_x", "mu_scale_y", "MFy",
                       "delta", "cdelta", "sdelta", "reference_mode", "reference_definition"}
OPTION_VARIABLES = {"speed_kmh", "mu", "steer_deg", "wear", "data_file", "calibration_file",
                    "native_executable", "output_dir", "reference_file", "native_threads",
                    "batch_size", "verbose", "progress_stride", "figure_visible", "make_plot", "save_results"}


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def plain(value):
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in sorted(value.items())}
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


def canonical(value):
    return json.dumps(plain(value), sort_keys=True, separators=(",", ":"), allow_nan=True)


def signature(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def write_text(path, text, *, immutable=False):
    path = Path(path)
    if path.exists():
        if path.read_text() == text:
            return
        if immutable:
            raise ValueError(f"Existing output conflicts with verified inputs: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text)
    os.replace(temporary, path)


def write_json(path, data, *, immutable=False):
    write_text(path, json.dumps(plain(data), indent=2, sort_keys=True, allow_nan=False) + "\n",
               immutable=immutable)


def copy_verified(source, destination, digest):
    """Resume identical copies; refuse conflicts rather than overwrite files."""
    source, destination = Path(source), Path(destination)
    if destination.exists():
        if sha(destination) != digest:
            raise ValueError(f"Conflicting destination: {destination}")
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    shutil.copy2(source, temporary)
    if sha(temporary) != digest:
        temporary.unlink()
        raise ValueError(f"Source changed while copying: {source}")
    os.replace(temporary, destination)


def expected_case(job_id):
    if isinstance(job_id, bool) or int(job_id) != job_id or not 1 <= int(job_id) <= TOTAL:
        raise ValueError("Expected an integer job ID in 1..1200")
    job_id = int(job_id)
    if job_id <= 1050:
        wear, condition = divmod(job_id - 1, 210)
        speed, remainder = divmod(condition, 35)
        friction, steering = divmod(remainder, 7)
        angle, combined = steering * .5, condition + 1
    else:
        wear, condition = divmod(job_id - 1051, 30)
        speed, friction = divmod(condition, 5)
        angle, combined = 3.5, 211 + condition
    return {"job_id": job_id, "condition_id": condition + 1,
            "source_condition_id": condition + 1, "combined_wear_condition_id": combined,
            "wear_case_id": wear + 1, "wear": WEARS[wear], "speed_kmh": 50. + speed * 10,
            "mu": [.9, .8, .7, .6, .5][friction], "steer_target_deg": angle}


def register_case(records, job, record):
    if job in records:
        if records[job] != record:
            raise ValueError(f"Conflicting duplicate original job ID: {job}")
        return
    records[job] = record


def require_complete(ids):
    missing = sorted(set(range(1, TOTAL + 1)) - set(ids))
    unexpected = sorted(set(ids) - set(range(1, TOTAL + 1)))
    if missing or unexpected:
        raise ValueError(f"Cannot finalize 1200: missing={len(missing)}, unexpected={unexpected}; missing IDs={missing}")


def equal_numeric(actual, expected, label, atol=1e-14):
    actual, expected = np.asarray(actual), np.asarray(expected)
    if actual.shape != expected.shape or not np.allclose(actual, expected, rtol=0, atol=atol, equal_nan=False):
        raise ValueError(f"Scientific invariant differs: {label}")


def check_policy(policy, label):
    for key, expected in POLICY.items():
        if key not in policy:
            raise ValueError(f"Missing policy setting: {label}.{key}")
        equal_numeric(policy[key], expected, f"{label}.{key}")


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def prepare_sources(root):
    stable = root / "stable_region"
    physics = json.loads((stable / "physics_binding.json").read_text())["physics_sha256"]
    for name in PHYSICS:
        if sha(root / name) != physics[name]:
            raise ValueError(f"Original physical source changed: {name}")
    original = loadmat(root / "result/run_configuration.mat", simplify_cells=True)["Signature"]
    if original["model_sha256"] != physics["HMC_FullCar_Revised.m"] or original["data_sha256"] != physics["data/hmc_data.mat"]:
        raise ValueError("Original result/ run signature and physical sources differ")
    dependencies = {v["name"]: v["sha256"] for v in original["dependencies"]}
    for name in PHYSICS[1:]:
        if name == "data/hmc_data.mat":
            continue
        key = "active_calibration_file" if name == "calibration_active.json" else name
        if dependencies[key] != physics[name]:
            raise ValueError(f"Original run dependency differs: {name}")
    seed_file = stable / "comparison/reference_core/Qref_0.mat"
    seed = loadmat(seed_file, simplify_cells=True)["Qref"]
    common = None
    sources = []
    for definition in BATCHES:
        source = dict(definition)
        base = stable / source["folder"]
        index = json.loads((base / "case_index.json").read_text())
        binding = json.loads((base / "run_binding.json").read_text())
        batch = loadmat(base / "batch_configuration.mat", simplify_cells=True)["Batch"]
        expected_ids = set(range(source["start"], source["end"] + 1))
        rows = {int(row["job_id"]): row for row in index["cases"]}
        if len(rows) != len(index["cases"]) or set(rows) != expected_ids:
            raise ValueError(f"Source case index is incomplete/duplicated: {source['name']}")
        if any(doc["batch_signature"] != source["signature"] for doc in (index, binding, batch)):
            raise ValueError(f"Unexpected source-batch signature: {source['name']}")
        if any(doc["reference_mode"] != "model_actual_front_trim" for doc in (index, binding, batch)):
            raise ValueError("Inconsistent reference-H construction")
        check_policy(batch["policy"], source["name"])
        files = binding["files_sha256"]
        shared = {name: files[name] for name in PHYSICS}
        shared.update({"root_mex": files["hmc_mex.mex"],
                       "region_mex": files[f"stable_region/{source['code']}/hmc_region_mex.mex"],
                       "kernel": files[f"stable_region/{source['code']}/hmc_region_mex.cpp"],
                       "reference_helper": files[f"stable_region/{source['code']}/reference_handling.m"],
                       "reference_base": files["stable_region/comparison/reference_core/Qref_0.mat"]})
        if any(shared[name] != physics[name] for name in PHYSICS) or shared["reference_base"] != sha(seed_file):
            raise ValueError("Source physical/reference binding differs")
        if common is not None and shared != common:
            raise ValueError("Original and extension physical engines/references differ")
        common = shared
        for relative in (source["module"], f"{source['code']}/hmc_region_mex.cpp", f"{source['code']}/reference_handling.m"):
            key = f"stable_region/{relative}"
            if sha(stable / relative) != files[key]:
                raise ValueError(f"Frozen source module changed: {relative}")
        for key in ("hmc_mex.mex", f"stable_region/{source['code']}/hmc_region_mex.mex"):
            if (root / key).exists() and sha(root / key) != files[key]:
                raise ValueError(f"Present native binary differs from its frozen binding: {key}")
        source.update({"base": base, "index": index, "rows": rows, "binding": binding,
                       "batch": batch, "module_object": load_module(stable / source["module"], "consolidate_" + source["name"])})
        sources.append(source)
    if sources[1]["batch"].get("legacy_batch_signature") != sources[0]["signature"]:
        raise ValueError("Extension is not bound to the original batch")
    if sources[1]["batch"]["InputSignature"]["legacy_batch_configuration_sha256"] != sha(sources[0]["base"] / "batch_configuration.mat"):
        raise ValueError("Extension references a different original batch configuration")
    return sources, common, seed


def check_science(data, spec, common, seed, invariant_options):
    case, meta = data["CASE"], data["meta"]
    if not isinstance(case, dict):
        case = np.asarray(case, dtype=object).ravel()[0]
    cfg, options = case["classification_configuration"], case["model_configuration"]
    original = data["original_result_signature"]
    if (original["model_sha256"] != common["HMC_FullCar_Revised.m"] or
            original["data_sha256"] != common["data/hmc_data.mat"]):
        raise ValueError("Per-result original physical signature differs")
    dependencies = {item["name"]: item["sha256"] for item in original["dependencies"]}
    for name in PHYSICS[1:]:
        if name != "data/hmc_data.mat":
            key = "active_calibration_file" if name == "calibration_active.json" else name
            if dependencies[key] != common[name]:
                raise ValueError(f"Per-result original physical dependency differs: {name}")
    check_policy(cfg, f"case {spec['job_id']}")
    equal_numeric(cfg["mu"], spec["mu"], "policy friction")
    for field, key in (("speed_kmh", "speed_kmh"), ("mu", "mu"), ("steer_deg", "steer_target_deg"), ("wear", "wear")):
        equal_numeric(options[field], spec[key], f"vehicle options {field}")
    invariant = {k: v for k, v in options.items() if k not in OPTION_VARIABLES}
    if invariant_options is not None and canonical(invariant) != invariant_options:
        raise ValueError(f"Vehicle numerical options differ at original job {spec['job_id']}")
    q = cfg["reference"]
    actual_static = {k: v for k, v in q.items() if k not in REFERENCE_VARIABLES}
    expected_static = {k: v for k, v in seed.items() if k not in REFERENCE_VARIABLES}
    if canonical(actual_static) != canonical(expected_static):
        raise ValueError(f"Static H geometry/load parameters differ at job {spec['job_id']}")
    for key, expected in (("ux", spec["speed_kmh"] / 3.6), ("mu", spec["mu"]),
                          ("mu_scale_x", spec["mu"] / .9), ("mu_scale_y", spec["mu"] / .9), ("wear", spec["wear"])):
        equal_numeric(q[key], expected, f"H {key}")
    wear = np.asarray(spec["wear"])
    equal_numeric(q["MFy"], MFY[(wear >= 12.5).astype(int) + (wear >= 37.5).astype(int)], "H tyre-wear coefficients")
    delta = np.deg2rad([meta["model_actual_front_steer_deg"]] * 2 + [0, 0])
    for key, expected in (("delta", delta), ("cdelta", np.cos(delta)), ("sdelta", np.sin(delta))):
        equal_numeric(q[key], expected, f"H actual steering {key}")
    if q["reference_mode"] != "model_actual_front_trim" or meta["kernel_source_sha256"] != common["kernel"]:
        raise ValueError("Unexpected H mode or original physical kernel")
    if meta["native_binary_sha256"] != common["region_mex"] or meta["runtime"] != "GNU Octave":
        raise ValueError("Unexpected physical engine or execution runtime")
    return case, canonical(invariant)


def discover(root, sources):
    paths = {}
    for source in sources:
        for path in sorted((source["base"] / "regions").glob("case_[0-9][0-9][0-9][0-9].mat")):
            job = int(path.stem.split("_")[-1])
            if not source["start"] <= job <= source["end"]:
                raise ValueError(f"Unexpected original ID in {source['name']}: {job}")
            register_case(paths, job, (source["name"], path))
    return paths


def audit_and_collect(root, output, sources, common, seed, paths):
    records, invariant_options = {}, None
    by_name = {s["name"]: s for s in sources}
    for job in sorted(paths):
        source_name, path = paths[job]
        source = by_name[source_name]
        module, spec = source["module_object"], expected_case(job)
        expected = module._expected_bindings(source["base"], job, require=True)
        summary = module.audit_result(path, expected)
        for key in ("job_id", "condition_id", "wear_case_id", "speed_kmh", "mu", "steer_target_deg", "wear"):
            equal_numeric(summary[key], spec[key], f"canonical ID {key}")
        data = loadmat(path, simplify_cells=True, verify_compressed_data_integrity=True)["RESEARCH2D"]
        _, invariant_options = check_science(data, spec, common, seed, invariant_options)
        row = source["rows"][job]
        equal_numeric(data["meta"]["model_actual_front_steer_deg"], row["actual_front_steer_deg"],
                      "actual trim steering versus prepared context index")
        equal_numeric(data["meta"]["reference_actual_front_steer_deg"], row["actual_front_steer_deg"],
                      "reference actual steering versus prepared context index")
        if int(row["reference_nodes"]) != summary["reference_nodes"] or bool(row["trim_inside_reference"]) != summary["trim_in_reference"]:
            raise ValueError(f"Prepared reference domain differs from its result at job {job}")
        context = source["base"] / row["context_file"]
        if context.exists() and sha(context) != row["context_sha256"]:
            raise ValueError(f"Raw context bytes differ at job {job}")
        image = source["base"] / "images" / f"case_{job:04d}.png"
        sidecar = image.with_suffix(".json")
        if not image.is_file() or not sidecar.is_file():
            raise ValueError(f"Missing mandatory source PNG/sidecar for job {job}")
        image_meta = json.loads(sidecar.read_text())
        image_sha = sha(image)
        if (int(image_meta["job_id"]) != job or image_meta["source_sha256"] != summary["source_sha256"] or
                image_meta["image_sha256"] != image_sha):
            raise ValueError(f"Source image binding differs at job {job}")
        result_relative = f"regions/case_{job:04d}.mat"
        image_relative = f"images/case_{job:04d}.png"
        sidecar_relative = f"images/case_{job:04d}.json"
        copy_verified(path, output / result_relative, summary["source_sha256"])
        copy_verified(image, output / image_relative, image_sha)
        sidecar_sha = sha(sidecar)
        copy_verified(sidecar, output / sidecar_relative, sidecar_sha)
        summary.update({"source_file": str(path.relative_to(root)), "source_image": str(image.relative_to(root)),
                        "source_sidecar": str(sidecar.relative_to(root)), "region_file": result_relative,
                        "image_file": image_relative, "image_sidecar": sidecar_relative,
                        "region_sha256": summary["source_sha256"], "image_sha256": image_sha,
                        "sidecar_sha256": sidecar_sha, "image_sidecar_sha256": sidecar_sha,
                        "region_bytes": path.stat().st_size, "image_bytes": image.stat().st_size,
                        "image_sidecar_bytes": sidecar.stat().st_size,
                        "source_batch": source_name, "source_batch_signature": source["signature"],
                        "source_condition_id": spec["source_condition_id"],
                        "combined_wear_condition_id": spec["combined_wear_condition_id"],
                        "context_verification": "raw_context_sha256_verified" if context.exists() else "prepared_index_and_result_metadata",
                        "runtime": data["meta"]["runtime"], "runtime_version": data["meta"]["runtime_version"]})
        register_case(records, job, summary)
    return records


def publish_manifest(root, output, sources, common, records):
    source_files = []
    for source in sources:
        for filename in ("case_index.json", "run_binding.json", "batch_configuration.mat"):
            path = source["base"] / filename
            digest = sha(path)
            # Prepared-index metadata may legitimately change when the missing
            # original cases are prepared in a new checkout. Version snapshots
            # by content; previously verified result/image copies stay intact.
            relative = f"source_batches/{source['name']}/{path.stem}.{digest[:16]}{path.suffix}"
            copy_verified(path, output / relative, digest)
            source_files.append({"source": str(path.relative_to(root)), "file": relative, "sha256": digest})
    missing = sorted(set(range(1, TOTAL + 1)) - set(records))
    ordered = [records[job] for job in sorted(records)]
    core = {"schema": "carsim_fullcar_1200_consolidation_v1", "total": TOTAL,
            "script_sha256": sha(__file__), "common_physics_and_engine_sha256": common,
            "source_batches": [{k: source[k] for k in ("name", "start", "end", "signature")} for source in sources],
            "source_files": source_files, "cases": ordered}
    digest = signature(core)
    manifest = dict(core, consolidation_signature=digest, completed=len(records), missing_ids=missing,
                    status="all_inputs_verified" if not missing else "partial",
                    complete_claimed=False, id_policy="Original IDs1..1050 plus appended IDs1051..1200; no renumbering",
                    condition_policy="source condition preserved; combined wear condition = old1..210 or210+extension1..30")
    write_json(output / "case_index.json", manifest)
    write_json(output / "input_manifest.json", manifest)
    progress = {"total": TOTAL, "completed": len(records), "failed": 0, "pending": len(missing),
                "progress": f"{len(records)}/1200", "missing_ids": missing,
                "status": "inputs_verified" if not missing else "partial", "aggregate_complete": False,
                "images": len(records), "images_copied_without_edit": True,
                "wear_progress": [{"wear_case_id": wear + 1, "wear": WEARS[wear],
                                    "completed": sum(r["wear_case_id"] == wear + 1 for r in ordered),
                                    "total": 240, "progress": f"{sum(r['wear_case_id'] == wear + 1 for r in ordered)}/240"}
                                   for wear in range(5)]}
    write_json(output / "progress.json", progress)
    return manifest, progress


def build_lut(output, records, manifest):
    require_complete(records)
    reference = np.empty((61, 121, TOTAL), dtype=bool)
    final = np.empty((61, 121, 2, TOTAL), dtype=bool)
    guarded = np.empty_like(final)
    cells = np.empty((60, 120, TOTAL), dtype=bool)
    trims = np.empty((34, TOTAL))
    options, policies, metadata = [], [], []
    grid = None
    for job in range(1, TOTAL + 1):
        record = records[job]
        path = output / record["region_file"]
        if sha(path) != record["source_sha256"]:
            raise ValueError(f"Copied original result changed before LUT assembly: {job}")
        data = loadmat(path, simplify_cells=True)["RESEARCH2D"]
        case = data["CASE"]
        if not isinstance(case, dict):
            case = np.asarray(case, dtype=object).ravel()[0]
        if grid is None:
            grid = {"vc": np.asarray(case["grid"]["vc"]).ravel(), "r": np.asarray(case["grid"]["r"]).ravel()}
        reference[:, :, job - 1] = case["handling"]["mask"]
        final[:, :, :, job - 1] = case["masks"]["final"]
        guarded[:, :, :, job - 1] = case["masks"]["guarded_final"]
        cells[:, :, job - 1] = case["region"]["cell_mask"]
        trims[:, job - 1] = np.asarray(case["trim"]["x"]).ravel()
        options.append(canonical(case["model_configuration"]))
        policies.append(canonical(case["classification_configuration"]))
        metadata.append(canonical(data["meta"]))
    ordered = [records[job] for job in range(1, TOTAL + 1)]
    columns = ["job_id", "source_condition_id", "combined_wear_condition_id", "wear_case_id",
               "speed_kmh", "mu", "steer_target_deg"]
    return {"schema": "carsim_fullcar_finite_time_regions_1200_v1", "model": "carsim_fullcar",
            "total": TOTAL, "grid": grid, "horizons_s": np.array([5., 10.]),
            "reference_mode": "model_actual_front_trim", "case_columns": np.array(columns, dtype=object),
            "case_spec": np.array([[row[key] for key in columns] for row in ordered]),
            "wear": np.array([r["wear"] for r in ordered]).T,
            "reference_mask": reference, "final": final, "guarded_final": guarded,
            "cell_mask": cells, "trim_state": trims,
            "cell_area": np.array([r["cell_area"] for r in ordered]),
            "accepted_cells": np.array([r["accepted_cells"] for r in ordered]),
            "grid_truncated": np.array([r["grid_truncated"] for r in ordered]),
            "reference_grid_truncated": np.array([r["reference_grid_truncated"] for r in ordered]),
            "source_sha256": np.array([r["source_sha256"] for r in ordered], dtype=object),
            "source_file": np.array([r["source_file"] for r in ordered], dtype=object),
            "region_file": np.array([r["region_file"] for r in ordered], dtype=object),
            "image_sha256": np.array([r["image_sha256"] for r in ordered], dtype=object),
            "context_sha256": np.array([r["context_sha256"] for r in ordered], dtype=object),
            "source_batch": np.array([r["source_batch"] for r in ordered], dtype=object),
            "source_batch_signature": np.array([r["source_batch_signature"] for r in ordered], dtype=object),
            "native_binary_sha256": ordered[0]["native_binary_sha256"],
            "consolidation_signature": manifest["consolidation_signature"],
            "source_batches_json": canonical(manifest["source_batches"]),
            "model_options_json": np.array(options, dtype=object),
            "classification_config_json": np.array(policies, dtype=object),
            "case_metadata_json": np.array(metadata, dtype=object),
            "certified_invariant": False, "images_copied_without_edit": True,
            "outside_reference_semantics": "Untested, not demonstrated dynamically unstable",
            "definition": "Original accepted-cell masks; all four vertices plus actual center; finite-time sampled10s policy"}


def verify_lut(path, expected, variable="LUT"):
    actual = loadmat(path, simplify_cells=True, verify_compressed_data_integrity=True)[variable]
    if set(actual) != set(expected):
        raise ValueError("Saved LUT field set differs")
    for key, value in expected.items():
        if isinstance(value, np.ndarray):
            read = np.asarray(actual[key])
            if read.shape != value.shape or not np.array_equal(read, value):
                raise ValueError(f"Saved LUT array or axis differs: {key}")
        elif isinstance(value, dict):
            if canonical(actual[key]) != canonical(value):
                raise ValueError(f"Saved LUT structure differs: {key}")
        elif actual[key] != value:
            raise ValueError(f"Saved LUT scalar differs: {key}")


def finalize(output, records, manifest, progress):
    lut = build_lut(output, records, manifest)
    path = output / "carsim_fullcar_stability_LUT_1200.mat"
    if path.exists():
        verify_lut(path, lut)
    else:
        temporary = path.with_name(path.name + ".tmp")
        savemat(temporary, {"LUT": lut}, appendmat=False, do_compression=True, long_field_names=True)
        verify_lut(temporary, lut)
        os.replace(temporary, path)
    ordered = [records[job] for job in range(1, TOTAL + 1)]
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=list(ordered[0]))
    writer.writeheader()
    for record in ordered:
        row = dict(record)
        row["wear"] = "/".join(f"{v:g}" for v in row["wear"])
        writer.writerow(row)
    qc = output / "carsim_fullcar_stability_QC_1200.csv"
    write_text(qc, stream.getvalue(), immutable=True)
    completed_manifest = dict(manifest, status="complete", complete_claimed=True)
    write_json(output / "input_manifest.json", completed_manifest)
    write_json(output / "case_index.json", completed_manifest)
    manifest_file = output / "input_manifest.json"
    document = {"status": "complete", "completed": TOTAL, "total": TOTAL, "progress": "1200/1200",
                "consolidation_signature": manifest["consolidation_signature"],
                "source_batches": manifest["source_batches"], "aggregate_file": path.name,
                "aggregate_sha256": sha(path), "aggregate_bytes": path.stat().st_size,
                "qc_csv": qc.name, "qc_csv_sha256": sha(qc), "qc_csv_bytes": qc.stat().st_size,
                "manifest_file": manifest_file.name, "manifest_sha256": sha(manifest_file),
                "manifest_bytes": manifest_file.stat().st_size,
                "numerically_verified_cases": TOTAL, "all_arrays_reloaded_and_verified": True,
                "images": TOTAL, "all_images_bound_to_source": True, "images_copied_without_edit": True,
                "all_cases_freshly_reaudited": True, "common_physics_and_engine_verified": True,
                "source_batch_bindings_verified": True, "all_source_batches_preserved": True,
                "physics_recomputed": False}
    write_json(output / "finalization.json", document, immutable=True)
    progress.update(status="complete", aggregate_complete=True, aggregate_file=path.name, qc_csv=qc.name)
    write_json(output / "progress.json", progress)
    return path


def consolidate(root, output=None, *, collect_available=False):
    root = Path(root).resolve()
    output = Path(output).resolve() if output else root / "stable_region/carsim_fullcar_1200"
    sources, common, seed = prepare_sources(root)
    paths = discover(root, sources)
    if not collect_available:
        require_complete(paths)
    elif (output / "finalization.json").exists():
        raise ValueError("Refusing collection-only mode on finalized output; run full verification instead")
    output.mkdir(parents=True, exist_ok=True)
    records = audit_and_collect(root, output, sources, common, seed, paths)
    manifest, progress = publish_manifest(root, output, sources, common, records)
    if collect_available:
        return {"status": progress["status"], "progress": progress["progress"],
                "missing_ids": progress["missing_ids"], "output_base": str(output), "finalized": False}
    path = finalize(output, records, manifest, progress)
    return {"status": "complete", "progress": "1200/1200", "aggregate": str(path), "output_base": str(output)}


def self_test():
    specs = [expected_case(job) for job in range(1, TOTAL + 1)]
    keys = {(tuple(s["wear"]), s["speed_kmh"], s["mu"], s["steer_target_deg"]) for s in specs}
    assert len(keys) == TOTAL
    for wear in range(1, 6):
        assert {s["combined_wear_condition_id"] for s in specs if s["wear_case_id"] == wear} == set(range(1, 241))
    missing_75 = set(range(1, TOTAL + 1)) - set(range(451, 526))
    try:
        require_complete(missing_75)
    except ValueError as exc:
        assert "missing=75" in str(exc)
    else:
        raise AssertionError("Missing75 inputs were accepted")
    registry = {}
    register_case(registry, 1, "original_sha")
    try:
        register_case(registry, 1, "conflicting_sha")
    except ValueError:
        pass
    else:
        raise AssertionError("Conflicting duplicate accepted")
    # Serialization fixture is expressly a probe, never a result LUT.
    with tempfile.TemporaryDirectory(prefix="consolidation_1200_probe_") as temporary:
        final = np.zeros((61, 121, 2, TOTAL), dtype=bool)
        final[0, 0, 0, :] = np.arange(TOTAL) % 2 == 0
        probe = {"schema": "synthetic_serialization_probe_only",
                 "job_ids": np.arange(1, TOTAL + 1), "final": final,
                 "source_batch_signature": np.array([BATCHES[0]["signature"]] * 1050 + [BATCHES[1]["signature"]] * 150, dtype=object)}
        path = Path(temporary) / "probe.mat"
        savemat(path, {"probe": probe}, do_compression=True, long_field_names=True)
        verify_lut(path, probe, "probe")
    return {"spec_bijection": TOTAL, "per_wear": 240, "missing75_refused": True,
            "duplicate_conflict_refused": True, "synthetic_1200_axis_and_two_batch_roundtrip": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path)
    parser.add_argument("--output-base", type=Path)
    parser.add_argument("--collect-available", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        print(json.dumps(self_test()))
    else:
        if not args.root:
            parser.error("--root is required")
        print(json.dumps(consolidate(args.root, args.output_base, collect_available=args.collect_available)))


if __name__ == "__main__":
    main()
