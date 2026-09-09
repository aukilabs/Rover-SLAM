"""Generate synthetic recon fixture and prior pack for tests."""

from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import Any, Dict, List

from .convert import convert_recon_to_prior_pack
from .schema import DEFAULT_DESCRIPTOR_DIM


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def _write_minimal_png(path: Path, width: int, height: int, rgb: tuple[int, int, int]) -> None:
    """Write an uncompressed RGB PNG without external dependencies."""
    path.parent.mkdir(parents=True, exist_ok=True)
    r, g, b = rgb
    row = bytes([0, r, g, b] * width)
    raw = row * height
    import zlib

    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xffffffff
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    idat = zlib.compress(raw, 9)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(
        b"IEND", b""
    )
    path.write_bytes(png)


def _synthetic_descriptor(seed: int) -> List[float]:
    dim = DEFAULT_DESCRIPTOR_DIM
    vals = []
    for i in range(dim):
        v = ((seed * 17 + i * 31) % 997) / 997.0 - 0.5
        vals.append(v)
    norm = sum(v * v for v in vals) ** 0.5 or 1.0
    return [v / norm for v in vals]


def write_recon_fixture(recon_root: Path) -> Path:
    """Write a tiny synthetic recon tree (3 images, 5 tracks, 1 portal)."""
    recon_root = recon_root.resolve()
    if recon_root.exists():
        import shutil

        shutil.rmtree(recon_root)
    recon_root.mkdir(parents=True)

    images_dir = recon_root / "images"
    images_dir.mkdir()

    image_specs = [
        {"name": "000.png", "color": (40, 60, 80), "t": [0.0, 0.0, 0.0]},
        {"name": "001.png", "color": (50, 70, 90), "t": [0.5, 0.0, 0.0]},
        {"name": "002.png", "color": (60, 80, 100), "t": [1.0, 0.0, 0.0]},
    ]
    fx, fy, cx, cy = 500.0, 500.0, 64.0, 48.0

    images: List[Dict[str, Any]] = []
    features_dir = recon_root / "features"
    features_dir.mkdir()

    keypoints_per_image = [
        [(32.0, 24.0), (48.0, 36.0), (64.0, 40.0), (80.0, 44.0)],
        [(30.0, 26.0), (50.0, 38.0), (70.0, 42.0), (90.0, 46.0)],
        [(34.0, 28.0), (54.0, 40.0), (74.0, 44.0)],
    ]

    for image_id, spec in enumerate(image_specs):
        rel_path = f"images/{spec['name']}"
        _write_minimal_png(images_dir / spec["name"], 128, 96, spec["color"])
        tx, ty, tz = spec["t"]
        images.append(
            {
                "id": image_id,
                "path": rel_path,
                "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
                "pose": {
                    "position": {"x": tx, "y": ty, "z": tz},
                    "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                },
            }
        )
        kpts = keypoints_per_image[image_id]
        desc = [_synthetic_descriptor(image_id * 100 + ki) for ki in range(len(kpts))]
        feat = {"keypoints": kpts, "descriptors": desc}
        _write_json(features_dir / f"{image_id}.json", feat)

    _write_json(recon_root / "images.json", {"images": images})

    portals = {
        "portals": [
            {
                "shortId": "FIXTURE01",
                "physicalSize": 0.2,
                "pose": {
                    "position": {"x": 0.5, "y": 0.0, "z": 0.0},
                    "rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                },
            }
        ]
    }
    _write_json(recon_root / "portals.json", portals)

    covis = {
        "edges": [
            {"i": 0, "j": 1, "shared_tracks": 3},
            {"i": 1, "j": 2, "shared_tracks": 2},
        ]
    }
    _write_json(recon_root / "covisibility.json", covis)

    track_colors = [
        (200, 80, 60),
        (60, 180, 90),
        (70, 120, 220),
        (220, 180, 40),
        (180, 60, 200),
    ]

    tracks = {
        "tracks": [
            {
                "id": 0,
                "position": {"x": 0.25, "y": 0.05, "z": 2.0},
                "rgb": list(track_colors[0]),
                "observations": [
                    {"image_id": 0, "keypoint_index": 0},
                    {"image_id": 1, "keypoint_index": 0},
                ],
            },
            {
                "id": 1,
                "position": {"x": 0.35, "y": 0.06, "z": 2.1},
                "rgb": list(track_colors[1]),
                "observations": [
                    {"image_id": 0, "keypoint_index": 1},
                    {"image_id": 1, "keypoint_index": 1},
                    {"image_id": 2, "keypoint_index": 0},
                ],
            },
            {
                "id": 2,
                "position": {"x": 0.45, "y": 0.07, "z": 2.2},
                "rgb": list(track_colors[2]),
                "observations": [
                    {"image_id": 0, "keypoint_index": 2},
                    {"image_id": 1, "keypoint_index": 2},
                ],
            },
            {
                "id": 3,
                "position": {"x": 0.55, "y": 0.08, "z": 2.3},
                "rgb": list(track_colors[3]),
                "observations": [
                    {"image_id": 1, "keypoint_index": 3},
                    {"image_id": 2, "keypoint_index": 1},
                ],
            },
            {
                "id": 4,
                "position": {"x": 0.65, "y": 0.09, "z": 2.4},
                "observations": [
                    {"image_id": 2, "keypoint_index": 2},
                ],
            },
        ]
    }
    _write_json(recon_root / "tracks.json", tracks)
    return recon_root


def generate_fixture_pack(
    repo_root: Path,
    recon_dir: str = "testdata/recon_fixture",
    pack_dir: str = "testdata/prior_pack_fixture",
) -> Path:
    """Generate recon fixture + prior pack under repo_root."""
    recon_root = repo_root / recon_dir
    pack_root = repo_root / pack_dir
    write_recon_fixture(recon_root)
    convert_recon_to_prior_pack(
        recon_root,
        pack_root,
        coordinate_frame_notes="Synthetic fixture pack for hybrid prior-map initiative S1.",
    )
    return pack_root
