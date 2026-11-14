#!/usr/bin/env python3
"""Verify the exact delivered second-half inputs without running simulations."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    root = args.root.resolve()
    manifest = json.loads((root / 'FILES_SHA256.json').read_text())
    assert manifest['assigned_job_ids'] == [526, 1050]
    seen = set()
    for entry in manifest['files']:
        name = entry['path']
        relative = PurePosixPath(name)
        assert not relative.is_absolute() and '..' not in relative.parts, name
        assert name not in seen, name
        seen.add(name)
        path = root / name
        assert path.is_file(), f'Missing: {name}'
        assert path.stat().st_size == entry['bytes'], f'Size differs: {name}'
        assert hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256'], f'SHA differs: {name}'
    expected_contexts = {f'stable_region/carsim_fullcar_1050/context/case_{i:04d}.mat'
                         for i in range(526, 1051)}
    actual_contexts = {name for name in seen if name.startswith(
        'stable_region/carsim_fullcar_1050/context/case_')}
    assert actual_contexts == expected_contexts, 'Handoff context IDs differ from526..1050'
    physical = json.loads((root / 'stable_region/physics_binding.json').read_text())
    for name, expected in physical['physics_sha256'].items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected, name
    for name, expected in manifest['native_binary_sha256'].items():
        assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected, name
    print(json.dumps({'status': 'pass', 'files': len(seen),
                      'prepared_contexts': 525, 'assigned_job_ids': [526, 1050],
                      'physical_sources_unchanged': True,
                      'simulation_started': False}))


if __name__ == '__main__':
    main()
