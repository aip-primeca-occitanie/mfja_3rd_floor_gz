#!/usr/bin/env python3
import hashlib
import json
import sys
from pathlib import Path, PurePosixPath

root = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()

def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()

checksum_files = sorted((root / "checksums").glob("*_files.sha256"))
if not checksum_files:
    raise SystemExit("no extracted checksum manifests found")
for checksum_file in checksum_files:
    for line in checksum_file.read_text(encoding="utf-8").splitlines():
        expected, relative = line.split("  ", 1)
        portable = PurePosixPath(relative)
        if portable.is_absolute() or ".." in portable.parts:
            raise SystemExit(f"unsafe manifest path: {relative}")
        target = (root / Path(*portable.parts)).resolve()
        try:
            target.relative_to(root)
        except ValueError:
            raise SystemExit(f"manifest path escaped root: {relative}")
        if not target.is_file() or digest(target) != expected:
            raise SystemExit(f"file verification failed: {relative}")
    print(f"verified {checksum_file.name}")

for manifest_path in sorted((root / "manifests").glob("*_asset.json")):
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for source in manifest.get("sources", []):
        rows_path = root / source["rows"]
        labels_path = root / source["labels"]
        rows = [json.loads(line) for line in rows_path.read_text(encoding="utf-8").splitlines()]
        labels = [json.loads(line) for line in labels_path.read_text(encoding="utf-8").splitlines()]
        if len(rows) != source["records"] or len(labels) != source["records"]:
            raise SystemExit(f"record count mismatch: {source['name']}")
        if [row["sample_id"] for row in rows] != [label["sample_id"] for label in labels]:
            raise SystemExit(f"row/label mismatch: {source['name']}")
        for row in rows:
            for reference in row["model_input"]["overhead_images"].values():
                portable = PurePosixPath(reference)
                if portable.is_absolute() or ".." in portable.parts:
                    raise SystemExit(f"unsafe image path: {reference}")
                if not (root / source["dataset_root"] / Path(*portable.parts)).is_file():
                    raise SystemExit(f"missing image: {reference}")
        print(f"verified {source['name']}: {source['records']} records")
