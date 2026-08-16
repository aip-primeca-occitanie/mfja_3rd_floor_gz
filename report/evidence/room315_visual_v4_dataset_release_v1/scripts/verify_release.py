#!/usr/bin/env python3
import hashlib
import json
import sys
from pathlib import Path

repo = Path(__file__).resolve().parents[1]
asset_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "release-assets")
manifest = json.loads(
    (repo / "manifests/release_manifest.json").read_text(encoding="utf-8")
)
for expected in manifest["assets"]:
    path = asset_dir / expected["name"]
    if not path.is_file():
        raise SystemExit(f"missing release asset: {path}")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != expected["sha256"] or path.stat().st_size != expected["bytes"]:
        raise SystemExit(f"release asset mismatch: {path}")
    print(f"verified {path.name}")
