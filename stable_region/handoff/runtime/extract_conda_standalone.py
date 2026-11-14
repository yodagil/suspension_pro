#!/usr/bin/env python3
"""Extract only the known standalone executable from a verified .conda archive."""
from pathlib import Path
import os
import sys
import tarfile
import zipfile
import zstandard

archive, output = map(Path, sys.argv[1:])
with zipfile.ZipFile(archive) as package:
    names = [name for name in package.namelist() if name.startswith('pkg-') and name.endswith('.tar.zst')]
    assert len(names) == 1
    found = False
    with package.open(names[0]) as compressed, zstandard.ZstdDecompressor().stream_reader(compressed) as decoded, tarfile.open(fileobj=decoded, mode='r|') as members:
        for member in members:
            if member.name == 'standalone_conda/conda.exe':
                assert member.isfile()
                data = members.extractfile(member)
                assert data is not None
                output.parent.mkdir(parents=True, exist_ok=True)
                temporary = output.with_name(output.name + '.tmp')
                with temporary.open('wb') as destination:
                    while block := data.read(1024 * 1024):
                        destination.write(block)
                temporary.chmod(0o755)
                os.replace(temporary, output)
                found = True
                break
    assert found, 'Expected conda executable not found'
