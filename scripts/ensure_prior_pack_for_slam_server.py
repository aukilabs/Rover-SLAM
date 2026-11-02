#!/usr/bin/env python3
"""Copy prior pack to a Linux path and decompress feature npz for slam_server/cnpy."""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import zipfile
from pathlib import Path


def _needs_decompress(npz_path: Path) -> bool:
    with zipfile.ZipFile(npz_path) as zf:
        for info in zf.infolist():
            if info.compress_type != 0:
                return True
    return False


def _decompress_npz(src: Path, dst: Path) -> None:
    import numpy as np

    data = np.load(src)
    dst.parent.mkdir(parents=True, exist_ok=True)
    np.savez(dst, **{k: data[k] for k in data.files})


def ensure_pack(src: Path, dst: Path, *, force: bool = False) -> None:
    src = src.resolve()
    dst = dst.resolve()
    if not src.is_dir():
        raise SystemExit(f"source pack missing: {src}")

    marker = dst / ".slam_server_ready"
    if marker.is_file() and not force:
        meta = json.loads(marker.read_text())
        src_manifest = src / "manifest.json"
        src_mtime = src_manifest.stat().st_mtime if src_manifest.is_file() else None
        if (
            meta.get("src") == str(src)
            and src_mtime is not None
            and meta.get("src_mtime") == src_mtime
        ):
            print(f"reuse prepared pack: {dst}")
            return
        print(
            f"refresh prepared pack (src changed or mtime mismatch): {dst}",
            flush=True,
        )

    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True)

    skip_copy = {"features"}
    for item in src.iterdir():
        if item.name in skip_copy:
            continue
        dest = dst / item.name
        if item.is_dir():
            shutil.copytree(item, dest, symlinks=True)
        else:
            shutil.copy2(item, dest)

    src_features = src / "features"
    dst_features = dst / "features"
    dst_features.mkdir()
    npz_files = sorted(src_features.glob("*.npz"), key=lambda p: int(p.stem))
    print(f"decompress {len(npz_files)} feature npz -> {dst_features}")
    for i, src_npz in enumerate(npz_files):
        dst_npz = dst_features / src_npz.name
        if _needs_decompress(src_npz):
            _decompress_npz(src_npz, dst_npz)
        else:
            shutil.copy2(src_npz, dst_npz)
        if (i + 1) % 200 == 0 or i + 1 == len(npz_files):
            print(f"  features {i + 1}/{len(npz_files)}")

    src_manifest = src / "manifest.json"
    src_mtime = src_manifest.stat().st_mtime if src_manifest.is_file() else None
    marker.write_text(
        json.dumps(
            {"src": str(src), "src_mtime": src_mtime, "features": len(npz_files)}
        )
        + "\n"
    )
    print(f"ready: {dst}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", type=Path, required=True)
    parser.add_argument("--dst", type=Path, required=True)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    ensure_pack(args.src, args.dst, force=args.force)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
