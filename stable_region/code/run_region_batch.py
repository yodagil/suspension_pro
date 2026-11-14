#!/usr/bin/env python3
"""Resume an explicitly assigned range of the exact 1,050-case region batch.

The Python process supervises one Octave case at a time. The original four-
thread native observer performs all physics. No cloud-lifetime guarantee is
made: if the machine stops, run this command again to resume valid checkpoints.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

CODE = Path(__file__).resolve().parent
STABLE = CODE.parent
ROOT = STABLE.parent
BASE = STABLE / "carsim_fullcar_1050"
TOTAL = 1050
STOP_REQUESTED = False


def utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def read_json(path: Path, fallback=None):
    if not path.exists():
        return fallback
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def atomic_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + f".{os.getpid()}.tmp")
    with temp.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stop_signal(signum, frame):
    global STOP_REQUESTED
    STOP_REQUESTED = True


def process_group_live(pgid: int) -> bool:
    """Check live descendants without treating unreaped zombies as workers."""
    proc = Path("/proc")
    if proc.is_dir():
        for stat in proc.glob("[0-9]*/stat"):
            try:
                fields = stat.read_text().rsplit(")", 1)[1].split()
                if int(fields[2]) == pgid and fields[0] not in {"Z", "X"}:
                    return True
            except (OSError, ValueError, IndexError):
                continue
        return False
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_process_group(child):
    """Terminate the worker's private session, including wrapper descendants."""
    if process_group_live(child.pid):
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 20
        while process_group_live(child.pid) and time.monotonic() < deadline:
            child.poll()
            time.sleep(0.1)
        if process_group_live(child.pid):
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            while process_group_live(child.pid):
                child.poll()
                time.sleep(0.1)
    child.wait(timeout=10)


def matlab_string(value: Path | str) -> str:
    return "'" + str(value).replace("'", "''") + "'"


def octave_command(args, expression: str) -> list[str]:
    command = [args.octave, "--quiet", "--no-init-file", "--no-site-file", "--no-history",
               "--eval", f"addpath({matlab_string(ROOT)}); addpath({matlab_string(CODE)}); {expression}"]
    if args.runtime_wrapper:
        command.insert(0, str(args.runtime_wrapper.resolve()))
    return command


def source_fingerprints() -> dict:
    """Bind actual bytes to both the archived original run and physics manifest."""
    from scipy.io import loadmat

    manifest = read_json(STABLE / "physics_binding.json")
    original = loadmat(ROOT / "result/run_configuration.mat", simplify_cells=True)["Signature"]
    expected = dict(manifest["physics_sha256"])
    assert expected["HMC_FullCar_Revised.m"] == original["model_sha256"]
    assert expected["data/hmc_data.mat"] == original["data_sha256"]
    dependencies = original["dependencies"]
    if isinstance(dependencies, dict):
        dependencies = [dependencies]
    by_name = {str(item["name"]): item for item in dependencies}
    for name in ["hmc_apply_calibration.m", "hmc_native_run.m", "hmc_mex.cpp",
                 "hmc_native.cpp", "tire_fields.inc", "calibration_active.json"]:
        key = "active_calibration_file" if name == "calibration_active.json" else name
        assert expected[name] == by_name[key]["sha256"], f"Original signature differs: {name}"
    for name, wanted in expected.items():
        assert sha256(ROOT / name) == wanted, f"Original physics changed: {name}"
    verified = read_json(STABLE / "comparison/validation/binary_binding.json")
    assert sha256(CODE / "hmc_region_mex.cpp") == verified["cpp_sha256"], (
        "Region observer source differs from the verified frozen implementation")

    paths = [CODE / name for name in (
        "prepare_region_batch.m", "run_region_batch_case.m", "hmc_region_mex.cpp",
        "reference_handling.m", "stable_region_sha256.m", "batch_region_artifacts.py",
        "compare_regions.py", "run_region_batch.py")]
    paths += [ROOT / "result/run_configuration.mat",
              STABLE / "comparison/reference_core/Qref_0.mat",
              STABLE / "comparison/validation/binary_binding.json"]
    for folder, stem in [(ROOT, "hmc_mex"), (CODE, "hmc_region_mex")]:
        # This supervisor runs GNU Octave, whose mexext is "mex". An older
        # MATLAB .mexa64 module may coexist and is not loaded by this worker.
        binary = folder / (stem + ".mex")
        assert binary.is_file(), f"Expected the compiled Octave module: {binary}"
        paths.append(binary)
    return {str(path.relative_to(ROOT)): sha256(path) for path in paths} | expected


class Supervisor:
    def __init__(self, args):
        self.args = args
        self.allowed_ids = set(range(args.start_id, args.end_id + 1))
        self.range_total = len(self.allowed_ids)
        self.control = BASE / "shards" / args.shard_name
        self.case_summaries = {}
        self.state = {
            "schema": "carsim_fullcar_1050_shard_runner_v1", "model": "carsim_fullcar",
            "pid": os.getpid(), "invocation_started_at": utc(), "status": "starting",
            "total": self.range_total, "global_total": TOTAL,
            "start_id": args.start_id, "end_id": args.end_id, "shard_name": args.shard_name,
            "scope": "assigned_inclusive_job_id_range", "active_job_id": None, "newly_completed": 0,
            "command": [sys.executable] + sys.argv,
            "reference_mode": "model_actual_front_trim", "threads": 4,
            "images_required": not args.no_images,
            "publish_branch": args.publish_branch,
            "environment_lifetime_guaranteed": False,
        }
        self.failures = read_json(self.control / "failures.json", {"cases": {}})
        self.last_published_completed = 0

    def update(self, **fields):
        self.state.update(fields)
        self.state["last_heartbeat_at"] = utc()
        progress = read_json(self.control / "progress.json", {})
        completed = progress.get("completed", self.state.get("completed", 0))
        self.state["completed"] = completed
        self.state["completed_in_range"] = completed
        self.state["progress_text"] = f"{completed}/{self.range_total}"
        atomic_json(self.control / "runner_state.json", self.state)

    def refresh_range_progress(self):
        """Count only audited assigned IDs; never mutate global progress/cache."""
        found = set(self.case_summaries)
        failed = {int(key) for key in self.failures["cases"] if int(key) in self.allowed_ids} - found
        wear_progress = []
        for wear_id in range(1, 6):
            selected = self.allowed_ids & set(range((wear_id - 1) * 210 + 1, wear_id * 210 + 1))
            if not selected:
                continue
            completed = len(selected & found)
            wear_progress.append({"wear_case_id": wear_id, "completed_in_range": completed,
                                  "total_in_range": len(selected), "full_wear_total": 210,
                                  "progress": f"{completed}/{len(selected)}",
                                  "failed": len(selected & failed)})
        progress = {"schema": "carsim_fullcar_range_progress_v1", "updated_at": utc(),
                    "scope": "assigned_inclusive_job_id_range", "shard_name": self.args.shard_name,
                    "start_id": self.args.start_id, "end_id": self.args.end_id,
                    "global_total": TOTAL, "total": self.range_total,
                    "completed": len(found), "completed_in_range": len(found),
                    "failed": len(failed), "pending": self.range_total - len(found) - len(failed),
                    "progress": f"{len(found)}/{self.range_total}",
                    "completed_job_ids": sorted(found), "wear_progress": wear_progress,
                    "images_required": not self.args.no_images,
                    "status": "range_complete" if len(found) == self.range_total else "in_progress",
                    "known_empty_by_trim_policy": sum(bool(s.get("main_region_known_empty"))
                                                       for s in self.case_summaries.values()),
                    "global_1050_complete_claimed": False}
        atomic_json(self.control / "progress.json", progress)
        return progress

    def run_octave(self, expression: str, log_name: str) -> int:
        command = octave_command(self.args, expression)
        logs = self.control / "logs"
        logs.mkdir(exist_ok=True)
        logfile = logs / log_name
        self.update(active_command=command, active_log=str(logfile.relative_to(BASE)))
        env = os.environ.copy()
        env["OMP_NUM_THREADS"] = "4"
        with logfile.open("a", encoding="utf-8") as stream:
            stream.write(f"\nSTART {utc()}\n")
            stream.flush()
            child = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, env=env, start_new_session=True)
            try:
                self.update(child_pid=child.pid)
                while True:
                    try:
                        return_code = child.wait(timeout=15)
                        break
                    except subprocess.TimeoutExpired:
                        self.update()
                        if STOP_REQUESTED:
                            raise InterruptedError("Stop requested; saved atomic checkpoints remain resumable")
            finally:
                stop_process_group(child)
            stream.write(f"END {utc()} exit={return_code}\n")
        self.update(child_pid=None)
        return return_code

    def save_failure(self, job_id: int, error: Exception | str, attempt: int):
        self.failures["cases"][str(job_id)] = {
            "job_id": job_id, "status": "failed", "attempt_this_invocation": attempt,
            "message": str(error), "updated_at": utc(),
        }
        self.failures["updated_at"] = utc()
        atomic_json(self.control / "failures.json", self.failures)

    def clear_failure(self, job_id: int):
        if self.failures["cases"].pop(str(job_id), None) is not None:
            self.failures["updated_at"] = utc()
            atomic_json(self.control / "failures.json", self.failures)

    def publish(self, completed: int, final: bool = False, global_final: bool = False):
        if not self.args.publish_every:
            return
        if not final and completed - self.last_published_completed < self.args.publish_every:
            return
        # Each prompt owns only its assigned result IDs and its runtime folder.
        # In particular, never stage the other shard or shared global progress.
        owned = []
        for folder in (BASE / "regions", BASE / "images"):
            for path in folder.glob("case_*.*"):
                match = re.fullmatch(r"case_(\d{4})\.(mat|png|json)", path.name)
                if match and int(match[1]) in self.allowed_ids:
                    owned.append(path)
        owned += list(self.control.rglob("*"))
        if global_final:
            owned += [BASE / name for name in (
                "carsim_fullcar_stability_LUT_1050.mat", "carsim_fullcar_stability_QC_1050.csv",
                "finalization.json", "run_binding.json", "progress.json")]
        excluded = {"logs", "quarantine", "__pycache__"}
        paths = sorted({str(p.relative_to(ROOT)) for p in owned
                        if p.is_file() and not p.is_symlink()
                        and not (set(p.relative_to(BASE).parts) & excluded)
                        and not any(part.startswith(".") for part in p.relative_to(BASE).parts)
                        and p.suffix not in {".tmp", ".log", ".lock", ".pid"}})
        try:
            if not paths:
                return
            checkout_branch = subprocess.check_output(
                ["git", "symbolic-ref", "--quiet", "--short", "HEAD"], cwd=ROOT, text=True).strip()
            assert checkout_branch == self.args.publish_branch, "Checkout and destination branches differ"
            subprocess.run(["git", "add", "--", *paths], cwd=ROOT, check=True,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            changed = subprocess.run(["git", "diff", "--cached", "--quiet", "--", *paths], cwd=ROOT)
            if changed.returncode == 1:
                message = (f"Complete carsim_fullcar range {self.args.start_id}-{self.args.end_id} "
                           f"({completed}/{self.range_total})" if final else
                           f"Save carsim_fullcar range {self.args.start_id}-{self.args.end_id} "
                           f"({completed}/{self.range_total})")
                subprocess.run(["git", "commit", "--only", "-m", message, "--", *paths],
                               cwd=ROOT, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True)
            elif changed.returncode != 0:
                raise RuntimeError(f"git diff failed: {changed.returncode}")
            subprocess.run(["git", "push", "origin", f"HEAD:{self.args.publish_branch}"], cwd=ROOT, check=True,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=600)
            head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
            remote = subprocess.check_output(["git", "ls-remote", "--heads", "origin", self.args.publish_branch],
                                             cwd=ROOT, text=True, timeout=60).split()[0]
            assert head == remote, "Remote branch did not match published commit"
            self.last_published_completed = completed
            self.update(last_publish_at=utc(), last_publish_commit=head, last_publish_error=None,
                        published_completed=completed)
        except Exception as error:
            # Publication trouble must not discard expensive simulation work.
            # Never expose subprocess stderr, which could contain credential URLs.
            self.update(last_publish_error={"type": type(error).__name__,
                                           "message": "Git publication failed; local computation continues",
                                           "at": utc()})

    def execute(self) -> int:
        artifacts = importlib.import_module("batch_region_artifacts")
        fingerprints = source_fingerprints()
        old_binding = read_json(self.control / "run_binding.json")
        legacy = None
        if old_binding:
            assert (old_binding.get("start_id"), old_binding.get("end_id"), old_binding.get("shard_name")) == (
                self.args.start_id, self.args.end_id, self.args.shard_name), (
                    "This shard name is already bound to a different assigned range")
            assert old_binding["files_sha256"] == fingerprints, (
                "Batch source/binary bytes differ from the existing run binding; "
                "do not mix engines or overwrite an active batch")
        else:
            legacy_path = BASE / "run_binding.json"
            legacy = read_json(legacy_path)
            if legacy:
                legacy_index = read_json(BASE / "case_index.json")
                assert legacy_index and legacy["batch_signature"] == legacy_index["batch_signature"] and legacy["reference_mode"] == legacy_index["reference_mode"], (
                    "Legacy batch/reference policy differs from the indexed contexts")
                old_files = dict(legacy["files_sha256"])
                controller = str((CODE / "run_region_batch.py").relative_to(ROOT))
                assert {k: v for k, v in old_files.items() if k != controller} == {
                    k: v for k, v in fingerprints.items() if k != controller}, (
                        "Legacy migration permits only the supervisor scope change; frozen numerical inputs differ")
                atomic_json(self.control / "legacy_migration.json", {
                    "legacy_binding_sha256": sha256(legacy_path), "verified_at": utc(),
                    "allowed_changed_file": controller,
                    "all_other_frozen_files_equal": True,
                    "old_controller_sha256": old_files.get(controller),
                    "new_controller_sha256": fingerprints[controller]})

        if not self.args.skip_prepare:
            self.update(status="preparing_contexts", active_job_id=None)
            result = self.run_octave(
                f"prepare_region_batch({self.args.start_id}:{self.args.end_id});", "prepare.log")
            assert result == 0, "Context preparation process failed; inspect logs/prepare.log"
        index = read_json(BASE / "case_index.json")
        assert index and index["total"] == TOTAL and len(index["cases"]) == TOTAL, (
            "The complete 1,050-case index is required before region execution")
        assert index["reference_mode"] == "model_actual_front_trim"
        if legacy:
            assert legacy["batch_signature"] == index["batch_signature"] and legacy["reference_mode"] == index["reference_mode"], (
                "Legacy batch/reference policy changed during context preparation")
        specs = {int(row["job_id"]): row for row in index["cases"]}
        assert set(specs) == set(range(1, TOTAL + 1))
        for job_id, spec in specs.items():
            assert int(spec["condition_id"]) == (job_id - 1) % 210 + 1
            assert int(spec["wear_case_id"]) == (job_id - 1) // 210 + 1
        binding = {"schema": "carsim_fullcar_range_binding_v1", "files_sha256": fingerprints,
                   "batch_signature": index["batch_signature"], "reference_mode": index["reference_mode"],
                   "start_id": self.args.start_id, "end_id": self.args.end_id,
                   "shard_name": self.args.shard_name, "global_total": TOTAL}
        if old_binding:
            assert old_binding == binding, "Context batch signature changed"
        else:
            atomic_json(self.control / "run_binding.json", binding)
        native_key = next(name for name in fingerprints if "/hmc_region_mex.mex" in name)
        native_sha = fingerprints[native_key]
        for spec in specs.values():
            spec["batch_signature"] = index["batch_signature"]
            spec["native_binary_sha256"] = native_sha
            spec["reference_mode"] = index["reference_mode"]

        def refresh_case_spec(job_id: int):
            assert job_id in self.allowed_ids, "Cannot prepare or mutate an unassigned case"
            current_index = read_json(BASE / "case_index.json")
            assert current_index["batch_signature"] == binding["batch_signature"], (
                "Context batch changed while the supervisor was running")
            rows = [row for row in current_index["cases"] if int(row["job_id"]) == job_id]
            assert len(rows) == 1, "Updated context index lacks its unique case"
            row = rows[0]
            row.update(batch_signature=binding["batch_signature"],
                       native_binary_sha256=native_sha,
                       reference_mode=binding["reference_mode"])
            context = BASE / "context" / f"case_{job_id:04d}.mat"
            assert row["status"] == "prepared" and context.is_file(), (
                "Case equilibrium preparation did not produce a prepared context")
            assert sha256(context) == row["context_sha256"], "Context differs from its prepared index"
            specs[job_id] = row

        def valid_case(job_id: int):
            assert job_id in self.allowed_ids, "Cannot audit/render outside the assigned range"
            path = BASE / "regions" / f"case_{job_id:04d}.mat"
            if not path.exists():
                return False
            summary = artifacts.audit_result(path, expected_spec=specs[job_id])
            self.case_summaries[job_id] = summary
            if self.args.no_images:
                return True
            image_path = BASE / "images" / f"case_{job_id:04d}.png"
            try:
                sidecar = read_json(image_path.with_suffix(".json"), {})
            except (OSError, ValueError):
                sidecar = {}
            if (not image_path.exists() or sidecar.get("source_sha256") != summary["source_sha256"]
                    or sidecar.get("image_sha256") != sha256(image_path)):
                artifacts.render_case(path, output_path=image_path)
            return True

        completed = set()
        for job_id in sorted(self.allowed_ids):
            try:
                if valid_case(job_id):
                    completed.add(job_id)
                    self.clear_failure(job_id)
            except Exception as error:
                self.case_summaries.pop(job_id, None)
                # Preserve invalid MAT bytes before recomputing from checkpoints.
                # A rendering failure alone does not invalidate its source MAT.
                path = BASE / "regions" / f"case_{job_id:04d}.mat"
                if path.exists():
                    try:
                        artifacts.audit_result(path, expected_spec=specs[job_id])
                    except Exception:
                        quarantine = self.control / "quarantine"
                        quarantine.mkdir(exist_ok=True)
                        preserved = quarantine / f"case_{job_id:04d}_{time.time_ns()}.mat"
                        os.replace(path, preserved)
                        error = RuntimeError(f"{error}; preserved at {preserved.relative_to(BASE)}")
                self.save_failure(job_id, f"Existing result failed audit: {error}", 0)
        self.refresh_range_progress()
        self.last_published_completed = len(completed)
        self.update(status="running", batch_signature=index["batch_signature"], resumed=len(completed),
                    active_job_id=None)
        priority = [int(x) for x in self.args.priority.split(",") if x.strip()]
        assert len(set(priority)) == len(priority) and all(1 <= x <= TOTAL for x in priority)
        ignored_priority = [job_id for job_id in priority if job_id not in self.allowed_ids]
        priority = [job_id for job_id in priority if job_id in self.allowed_ids]
        order = priority + [job_id for job_id in sorted(self.allowed_ids) if job_id not in priority]
        self.update(ignored_out_of_range_priority_ids=ignored_priority)
        attempted = 0
        limit_reached = False
        for attempt in range(1, self.args.attempts + 1):
            for job_id in order:
                if STOP_REQUESTED:
                    raise InterruptedError("Stop requested; batch can be resumed")
                if job_id in completed:
                    continue
                if self.args.limit is not None and attempted >= self.args.limit:
                    limit_reached = True
                    break
                assert source_fingerprints() == fingerprints, (
                    "Frozen source/binary bytes changed while this batch was running")
                attempted += 1
                self.update(active_job_id=job_id, attempt=attempt,
                            condition_id=specs[job_id]["condition_id"],
                            wear_case_id=specs[job_id]["wear_case_id"])
                print(json.dumps({"event": "case_start", "job_id": job_id,
                                  "completed_in_range": len(completed), "total_in_range": self.range_total,
                                  "start_id": self.args.start_id, "end_id": self.args.end_id,
                                  "attempt": attempt, "at": utc()}), flush=True)
                try:
                    if not (BASE / "context" / f"case_{job_id:04d}.mat").exists():
                        code = self.run_octave(f"prepare_region_batch({job_id});",
                                               f"prepare_case_{job_id:04d}.log")
                        assert code == 0, "Case equilibrium preparation failed"
                    refresh_case_spec(job_id)
                    code = self.run_octave(f"run_region_batch_case({job_id});",
                                           f"case_{job_id:04d}.log")
                    assert code == 0, f"Region worker exit {code}; see case log"
                    assert valid_case(job_id), "Region worker did not produce a valid final MAT"
                    completed.add(job_id)
                    self.clear_failure(job_id)
                    self.state["newly_completed"] += 1
                    self.refresh_range_progress()
                    self.update(last_completed_job_id=job_id)
                    self.publish(len(completed))
                    print(json.dumps({"event": "case_complete", "job_id": job_id,
                                      "progress": f"{len(completed)}/{self.range_total}", "at": utc()}), flush=True)
                except InterruptedError:
                    raise
                except Exception as error:
                    self.case_summaries.pop(job_id, None)
                    self.save_failure(job_id, error, attempt)
                    self.refresh_range_progress()
                    self.update(last_case_error={"job_id": job_id, "message": str(error)})
                    print(json.dumps({"event": "case_failed", "job_id": job_id,
                                      "attempt": attempt, "error": str(error), "at": utc()}), flush=True)
            if limit_reached or len(completed) == self.range_total:
                break
        self.update(active_job_id=None, child_pid=None)
        if len(completed) == self.range_total:
            assert source_fingerprints() == fingerprints, "Frozen inputs changed before finalization"
            completion = {"schema": "carsim_fullcar_range_completion_v1", "status": "range_complete",
                          "shard_name": self.args.shard_name, "start_id": self.args.start_id,
                          "end_id": self.args.end_id, "completed_in_range": self.range_total,
                          "total_in_range": self.range_total, "global_total": TOTAL,
                          "progress": f"{self.range_total}/{self.range_total}", "created_at": utc(),
                          "reference_mode": index["reference_mode"],
                          "batch_signature": index["batch_signature"],
                          "native_binary_sha256": native_sha, "files_sha256": fingerprints,
                          "images_required": not self.args.no_images,
                          "global_1050_complete_claimed": False,
                          "verified_cases": [{"job_id": job_id,
                              "source_file": f"regions/case_{job_id:04d}.mat",
                              "source_sha256": self.case_summaries[job_id]["source_sha256"],
                              "context_sha256": self.case_summaries[job_id]["context_sha256"]}
                              for job_id in sorted(completed)]}
            completion_name = f"range_completion_{self.args.start_id:04d}_{self.args.end_id:04d}.json"
            atomic_json(self.control / completion_name, completion)
            self.refresh_range_progress()
            self.update(status="range_complete", finished_at=utc(), range_completion=completion_name)
            global_final = False
            if self.args.finalize_global or self.range_total == TOTAL:
                all_present = all((BASE / "regions" / f"case_{job:04d}.mat").exists()
                                  for job in range(1, TOTAL + 1))
                if all_present:
                    self.update(status="finalizing_global")
                    # This shared binding is written only by explicit/global assembly,
                    # never as a side effect of ordinary partial-range execution.
                    global_binding = {"schema": "carsim_fullcar_1050_binding_v1",
                                      "files_sha256": fingerprints,
                                      "batch_signature": index["batch_signature"],
                                      "reference_mode": index["reference_mode"]}
                    atomic_json(BASE / "run_binding.json", global_binding)
                    aggregate = artifacts.finalize(BASE, make_images=not self.args.no_images)
                    self.update(status="complete", global_1050_complete_claimed=True,
                                aggregate=str(aggregate.relative_to(BASE)))
                    global_final = True
                else:
                    self.update(global_finalization="waiting_for_other_assigned_cases")
            self.publish(self.range_total, final=True, global_final=global_final)
            return 0
        status = "paused_by_limit" if limit_reached else "finished_with_failures"
        self.refresh_range_progress()
        self.update(status=status, finished_at=utc())
        return 0 if limit_reached else 1


def cli():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--octave", default="octave-cli", help="Octave executable (default: octave-cli)")
    parser.add_argument("--runtime-wrapper", type=Path, help="Optional executable prepended to Octave argv")
    parser.add_argument("--priority", default="302,305,307", help="Comma-separated case IDs before remaining 1..1050")
    parser.add_argument("--start-id", type=int, default=1, help="First assigned canonical job ID, inclusive")
    parser.add_argument("--end-id", type=int, default=TOTAL, help="Last assigned canonical job ID, inclusive")
    parser.add_argument("--shard-name", help="Isolated runtime folder name (default: range_START_END)")
    parser.add_argument("--skip-prepare", action="store_true", help="Use an already prepared complete case index")
    parser.add_argument("--no-images", action="store_true",
                        help="Generate and audit MAT/CSV results without rendering or checking case PNGs")
    parser.add_argument("--limit", type=int, help="Stop after N region-worker attempts; 0 prepares/audits only")
    parser.add_argument("--attempts", type=int, default=2, help="Maximum passes over failed cases (default: 2)")
    parser.add_argument("--publish-every", type=int, default=0,
                        help="Git commit/push completed artifacts every N new cases; 0 disables publication")
    parser.add_argument("--publish-branch", help="Destination branch (default: main for start1, stable-region-START-END otherwise)")
    parser.add_argument("--finalize-global", action="store_true",
                        help="After this range completes, assemble a global LUT only if all 1050 results exist")
    parser.add_argument("--status", action="store_true", help="Read status without modifying files or starting a worker")
    args = parser.parse_args()
    if not 1 <= args.start_id <= args.end_id <= TOTAL:
        parser.error("Use an inclusive range with 1 <= --start-id <= --end-id <= 1050")
    if args.shard_name is None:
        args.shard_name = f"range_{args.start_id:04d}_{args.end_id:04d}"
    if args.publish_branch is None:
        args.publish_branch = ("main" if args.start_id == 1 else
                               f"stable-region-{args.start_id}-{args.end_id}")
    if args.start_id >= 526 and args.publish_branch == "main":
        parser.error("Second-half shards must use a dedicated non-main publication branch")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", args.shard_name):
        parser.error("--shard-name must contain only letters, digits, underscores or hyphens")
    control = BASE / "shards" / args.shard_name
    if args.status:
        state = read_json(control / "runner_state.json", {})
        pid = state.get("pid")
        alive = False
        if pid:
            try:
                os.kill(int(pid), 0)
                command = Path(f"/proc/{pid}/cmdline")
                alive = not command.exists() or b"run_region_batch.py" in command.read_bytes()
            except (ProcessLookupError, PermissionError):
                pass
        print(json.dumps({"runner": state, "process_alive": alive,
                          "progress": read_json(control / "progress.json", {})}, indent=2))
        return 0
    parser.error("--limit must be >= 0") if args.limit is not None and args.limit < 0 else None
    parser.error("--attempts must be >= 1") if args.attempts < 1 else None
    parser.error("--publish-every must be >= 0") if args.publish_every < 0 else None
    if args.publish_every:
        valid_branch = subprocess.run(["git", "check-ref-format", "--branch", args.publish_branch],
                                      cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if valid_branch.returncode != 0:
            parser.error("--publish-branch must be a valid Git branch name")
        checkout = subprocess.run(["git", "symbolic-ref", "--quiet", "--short", "HEAD"],
                                  cwd=ROOT, capture_output=True, text=True)
        if checkout.returncode != 0 or checkout.stdout.strip() != args.publish_branch:
            parser.error("Current checkout branch must equal --publish-branch before publication")
    BASE.mkdir(parents=True, exist_ok=True)
    control.mkdir(parents=True, exist_ok=True)
    compute_lock = (BASE / ".compute.lock").open("a+")
    lock = (control / ".runner.lock").open("a+")
    try:
        # The numerical .m worker writes a shared active_case.json; independent
        # prompts must use independent checkouts, even for disjoint ranges.
        fcntl.flock(compute_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("Another shard owns this checkout's compute lock; use an independent checkout", file=sys.stderr)
        lock.close()
        compute_lock.close()
        return 2
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    supervisor = Supervisor(args)
    try:
        return supervisor.execute()
    except InterruptedError as error:
        supervisor.update(status="interrupted", child_pid=None, message=str(error), finished_at=utc())
        return 130
    except Exception as error:
        supervisor.update(status="failed", child_pid=None,
                          error={"type": type(error).__name__, "message": str(error)}, finished_at=utc())
        print(f"Batch supervisor failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)
        lock.close()
        fcntl.flock(compute_lock, fcntl.LOCK_UN)
        compute_lock.close()


if __name__ == "__main__":
    raise SystemExit(cli())
