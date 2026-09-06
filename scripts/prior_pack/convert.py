"""Convert reconstruction layout to prior pack v1."""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any, Dict

from .schema import (
    DEFAULT_DESCRIPTOR_DIM,
    DEFAULT_DESCRIPTOR_MODEL,
    DEFAULT_DESCRIPTOR_TYPE,
    FORMAT_VERSION,
)


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def _copy_features(features_src: Path, features_dst: Path) -> None:
    features_dst.mkdir(parents=True, exist_ok=True)
    for src in sorted(features_src.iterdir()):
        if src.suffix in (".npz", ".json") and src.stem.isdigit():
            shutil.copy2(src, features_dst / src.name)


def convert_recon_to_prior_pack(
    recon_root: Path,
    output_root: Path,
    coordinate_frame_name: str = "domain",
    coordinate_frame_notes: str = "Poses from reconstruction input (camera-to-world).",
) -> Path:
    """Read recon layout from recon_root and write a prior pack to output_root."""
    recon_root = recon_root.resolve()
    output_root = output_root.resolve()

    if not recon_root.is_dir():
        raise FileNotFoundError(f"recon input directory not found: {recon_root}")

    required = [
        "images.json",
        "covisibility.json",
        "tracks.json",
        "features",
    ]
    for name in required:
        path = recon_root / name
        if not path.exists():
            raise FileNotFoundError(f"recon input missing required {name} at {path}")

    images_doc = json.loads((recon_root / "images.json").read_text(encoding="utf-8"))
    images = images_doc.get("images", [])
    if not images:
        raise ValueError("recon images.json has no images")

    portals_src = recon_root / "portals.json"
    if portals_src.is_file():
        portals_doc = json.loads(portals_src.read_text(encoding="utf-8"))
    else:
        portals_doc = {"portals": []}

    tracks_doc = json.loads((recon_root / "tracks.json").read_text(encoding="utf-8"))
    covis_doc = json.loads((recon_root / "covisibility.json").read_text(encoding="utf-8"))

    tracks = tracks_doc.get("tracks", [])
    portals = portals_doc.get("portals", [])

    if output_root.exists():
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True)

    images_out_dir = output_root / "images"
    images_out_dir.mkdir()

    out_images: list[Dict[str, Any]] = []
    for img in images:
        rel = img["path"]
        src_blob = recon_root / rel
        if not src_blob.is_file():
            raise FileNotFoundError(f"image blob not found: {src_blob}")
        dst_rel = Path("images") / Path(rel).name
        dst_blob = output_root / dst_rel
        shutil.copy2(src_blob, dst_blob)
        out_img = dict(img)
        out_img["path"] = dst_rel.as_posix()
        out_images.append(out_img)

    _copy_features(recon_root / "features", output_root / "features")

    _write_json(output_root / "images.json", {"images": out_images})
    _write_json(output_root / "portals.json", {"portals": portals})
    _write_json(output_root / "covisibility.json", covis_doc)
    _write_json(output_root / "tracks.json", tracks_doc)

    manifest = {
        "format_version": FORMAT_VERSION,
        "coordinate_frame": {
            "name": coordinate_frame_name,
            "notes": coordinate_frame_notes,
        },
        "descriptor": {
            "type": DEFAULT_DESCRIPTOR_TYPE,
            "model": DEFAULT_DESCRIPTOR_MODEL,
            "dim": DEFAULT_DESCRIPTOR_DIM,
        },
        "files": {
            "images": "images.json",
            "portals": "portals.json",
            "covisibility": "covisibility.json",
            "tracks": "tracks.json",
            "features_dir": "features",
        },
        "counts": {
            "images": len(out_images),
            "tracks": len(tracks),
            "portals": len(portals),
        },
    }
    _write_json(output_root / "manifest.json", manifest)
    return output_root
