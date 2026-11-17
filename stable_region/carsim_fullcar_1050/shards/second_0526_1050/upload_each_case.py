#!/usr/bin/env python3
"""Publish completed, audited second-half cases without controlling physics."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(os.environ.get('HMC_REPO_ROOT', '/workspace/suspension_pro_second')).resolve()
BASE = ROOT / 'stable_region/carsim_fullcar_1050'
CONTROL = BASE / 'shards/second_0526_1050'
LOGS = CONTROL / 'logs'
BRANCH = 'stable-region-526-1050'
STATUS = LOGS / 'each_case_uploader_status.json'
PUBLIC = CONTROL / 'per_case_upload.json'
GIT_ENV = dict(os.environ, HMC_EACH_CASE_UPLOADER='1')
sys.path.insert(0, str(ROOT / 'stable_region/code'))
import batch_region_artifacts as artifacts
from PIL import Image


def now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def atomic(path, value):
    temporary = path.with_name(path.name + f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def read(path, default=None):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {} if default is None else default


def git(*args, check=True):
    result = subprocess.run(['git', '-C', str(ROOT), *args], env=GIT_ENV,
                            capture_output=True, timeout=180)
    if check and result.returncode:
        # Do not copy credential-bearing subprocess stderr into public metadata.
        raise RuntimeError(f'Git {args[0]} failed, exit {result.returncode}')
    return result.stdout.decode().strip() if result.returncode == 0 else None


def allowed(name):
    prefix = 'stable_region/carsim_fullcar_1050/'
    rel = name[len(prefix):] if name.startswith(prefix) else ''
    match = re.fullmatch(r'(regions|images)/case_(\d{4})\.(mat|png|json)', rel)
    if match:
        return 526 <= int(match[2]) <= 1050 and (
            (match[1] == 'regions' and match[3] == 'mat') or
            (match[1] == 'images' and match[3] in {'png', 'json'}))
    if rel == 'shards/second_0526_1050/upload_each_case.py':
        return True
    return (rel.startswith('shards/second_0526_1050/') and
            Path(rel).suffix in {'.json', '.csv', '.md'} and
            not any(p.startswith('.') or p in {'logs', 'checkpoints', 'quarantine', '__pycache__'}
                    for p in Path(rel).parts))


def guard():
    assert git('symbolic-ref', '--short', 'HEAD') == BRANCH, 'Unexpected branch'
    staged = git('diff', '--cached', '--name-only').splitlines()
    assert all(allowed(p) for p in staged), 'Foreign staged work; publication deferred'


def ancestor(older, newer):
    return git('merge-base', '--is-ancestor', older, newer, check=False) is not None


def remote_tip():
    answer = git('ls-remote', '--heads', 'origin', BRANCH).split()
    assert answer, 'Publication branch disappeared'
    return answer[0]


def push_tip(tip):
    remote = remote_tip()
    if remote != tip and not ancestor(tip, remote):
        assert ancestor(remote, tip), 'Remote branch diverged; no merge or force push'
        changed = git('diff', '--name-only', remote + '..' + tip).splitlines()
        assert all(allowed(p) for p in changed), 'Pending commit has unapproved paths'
        git('push', 'origin', f'{tip}:{BRANCH}')
        remote = remote_tip()
    assert remote == tip or ancestor(tip, remote), 'Remote verification failed'
    return remote


def paths_for(job):
    return [BASE / 'regions' / f'case_{job:04d}.mat',
            BASE / 'images' / f'case_{job:04d}.png',
            BASE / 'images' / f'case_{job:04d}.json']


def relative(path):
    return str(path.relative_to(ROOT))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tree(tip, paths):
    raw = subprocess.check_output(['git', '-C', str(ROOT), 'ls-tree', '-r', '-z', tip,
                                   '--', *[relative(p) for p in paths]], env=GIT_ENV)
    return {name.decode(): meta.split()[2].decode() for record in raw.split(b'\0')
            if record for meta, name in [record.split(b'\t', 1)]}


def validate(job, cache):
    paths = paths_for(job)
    stamp = tuple((p.stat().st_size, p.stat().st_mtime_ns) for p in paths)
    if job not in cache or cache[job][0] != stamp:
        summary = artifacts.audit_result(paths[0])
        sidecar = read(paths[2])
        assert summary['job_id'] == job
        assert sidecar['source_sha256'] == sha(paths[0])
        assert sidecar['image_sha256'] == sha(paths[1])
        with Image.open(paths[1]) as image:
            image.verify()
        assert not any(p.is_symlink() for p in paths)
        blobs = {relative(p): git('hash-object', '--', str(p)) for p in paths}
        cache[job] = (stamp, blobs)
    return cache[job][1]


def uploaded(tip, jobs, cache):
    found = tree(tip, [p for j in jobs for p in paths_for(j)])
    return {j for j in jobs if all(found.get(name) == blob
            for name, blob in validate(j, cache).items())}


def status(phase, **fields):
    previous = read(STATUS)
    previous.update(pid=os.getpid(), phase=phase, heartbeat_utc=now(),
                    branch=BRANCH, upload_every_completed_case=1, **fields)
    atomic(STATUS, previous)


def commit_paths(paths, message):
    names = [relative(p) for p in paths]
    assert all(allowed(p) for p in names)
    assert all(p.is_file() and not p.is_symlink() for p in paths)
    guard()
    git('add', '--', *names)
    # --only ensures a legacy publisher's other owned staged files cannot
    # silently join this individual-case commit.
    if git('diff', '--cached', '--quiet', '--', *names, check=False) is None:
        git('diff', '--cached', '--check')
        git('commit', '--only', '-m', message, '--', *names)
    return push_tip(git('rev-parse', 'HEAD'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interval', type=float, default=2.)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--audit-only', action='store_true')
    args = parser.parse_args()
    LOGS.mkdir(parents=True, exist_ok=True)
    lock = (LOGS / 'each_case_uploader.lock').open('a+')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    cache = {}
    last_verified_head = last_remote = None
    status('starting', uploaded_cases=0)
    while True:
        try:
            guard()
            producer = read(CONTROL / 'runner_state.json')
            progress = read(CONTROL / 'progress.json')
            jobs = sorted(set(map(int, progress.get('completed_job_ids', []))))
            assert all(526 <= j <= 1050 for j in jobs)
            for job in jobs:
                validate(job, cache)
            if args.audit_only:
                print(json.dumps({'audited_cases': len(jobs), 'job_ids': jobs,
                                  'git_writes': False}), flush=True)
                status('audit_only', audited_cases=len(jobs))
                return 0
            head = git('rev-parse', 'HEAD')
            if head != last_verified_head:
                last_remote = push_tip(head)
                last_verified_head = git('rev-parse', 'HEAD')
                # Recheck immediately if the legacy controller added a commit
                # while the network transaction was in flight.
                if last_verified_head != head:
                    continue
            done = uploaded(last_remote, jobs, cache)
            for job in jobs:
                if job in done:
                    continue
                record = dict(schema='carsim_fullcar_each_case_upload_v1', model='carsim_fullcar',
                    start_id=526, end_id=1050, assigned_total=525,
                    upload_every_completed_case=1, computed_cases=len(jobs),
                    uploaded_cases=len(done | {job}), uploaded_job_ids=sorted(done | {job}),
                    latest_uploaded_job_id=job, updated_utc=now(),
                    numerical_calculation_stopped=False, global_1050_complete_claimed=False)
                atomic(PUBLIC, record)
                last_remote = commit_paths(paths_for(job) + [PUBLIC, CONTROL / 'progress.json'],
                                          f'Publish carsim_fullcar case {job} ({len(done)+1}/525)')
                assert uploaded(last_remote, [job], cache) == {job}
                last_verified_head = git('rev-parse', 'HEAD')
                done.add(job)
                status('watching', uploaded_cases=len(done), computed_cases=len(jobs),
                       latest_uploaded_job_id=job, remote_commit=last_remote, last_error=None)
                print(f'{now()} UPLOADED {len(done)}/525 case {job}; remote {last_remote}', flush=True)
            status('watching', uploaded_cases=len(done), computed_cases=len(jobs),
                   remote_commit=last_remote, last_error=None)
            if producer.get('status') in {'range_complete', 'complete'} and len(done) == 525:
                completion = CONTROL / 'range_completion_0526_1050.json'
                if completion.is_file():
                    last_remote = commit_paths([completion, PUBLIC, CONTROL / 'progress.json'],
                                              'Complete per-case publication for range 526-1050')
                status('range_complete', uploaded_cases=525, remote_commit=last_remote)
                return 0
            if args.once:
                return 0
        except Exception as error:
            # All failures are publication-only. Never signal a numerical worker
            # or change model inputs, checkpoints, source files, or Git history.
            status('retrying', last_error=type(error).__name__,
                   error_note='Publication retry pending; numerical calculation is unaffected')
            print(f'{now()} RETRY {type(error).__name__}: {error}', flush=True)
            if args.once or args.audit_only:
                return 1
        time.sleep(max(1., args.interval))


if __name__ == '__main__':
    raise SystemExit(main())
