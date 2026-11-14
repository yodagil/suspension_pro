#!/usr/bin/env python3
"""Verify the bundled runtime installer and exact public-package lock."""
import hashlib
import json
from pathlib import Path
import sys
root = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parent)
manifest = json.loads((root / 'runtime_inputs.json').read_text())
for name, wanted in manifest['sha256'].items():
    actual = hashlib.sha256((root / name).read_bytes()).hexdigest()
    if actual != wanted:
        raise SystemExit(f'Runtime input SHA-256 mismatch: {name}')
print('Bundled runtime archive and public package lock: SHA-256 passed')
