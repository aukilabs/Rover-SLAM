"""Prior pack schema constants and shared helpers."""

from __future__ import annotations

from typing import Any, List, Sequence, Tuple

FORMAT_VERSION = 2
MIN_FORMAT_VERSION = 1
RGB_CHANNEL_COUNT = 3
DEFAULT_DESCRIPTOR_TYPE = "aliked"
DEFAULT_DESCRIPTOR_MODEL = "aliked-n16"
DEFAULT_DESCRIPTOR_DIM = 128

REQUIRED_MANIFEST_KEYS = (
    "format_version",
    "coordinate_frame",
    "descriptor",
    "files",
    "counts",
)

REQUIRED_FILE_KEYS = (
    "images",
    "portals",
    "covisibility",
    "tracks",
    "features_dir",
)

REQUIRED_COUNT_KEYS = ("images", "tracks", "portals")

REQUIRED_INTRINSIC_KEYS = ("fx", "fy", "cx", "cy")
REQUIRED_POSE_KEYS = ("position", "rotation")
REQUIRED_VEC3_KEYS = ("x", "y", "z")
REQUIRED_QUAT_KEYS = ("w", "x", "y", "z")


class PriorPackError(Exception):
    """Raised when a prior pack fails validation."""


def supported_format_version(version: int) -> bool:
    return MIN_FORMAT_VERSION <= version <= FORMAT_VERSION


def validate_track_rgb(rgb: Any, label: str) -> Tuple[int, int, int]:
    """Validate optional track rgb field; returns (r, g, b) in 0–255."""
    if not isinstance(rgb, list) or len(rgb) != RGB_CHANNEL_COUNT:
        raise PriorPackError(f"{label}.rgb must be [r, g, b]")
    out: List[int] = []
    for idx, channel in enumerate(rgb):
        if not isinstance(channel, int) or isinstance(channel, bool):
            raise PriorPackError(f"{label}.rgb[{idx}] must be int")
        if not 0 <= channel <= 255:
            raise PriorPackError(f"{label}.rgb[{idx}] must be in 0–255")
        out.append(channel)
    return out[0], out[1], out[2]


def colmap_point_rgb(point: Any) -> Sequence[int]:
    """Extract uint8 RGB from a pycolmap Point3D, or empty if unavailable."""
    color = getattr(point, "color", None)
    if color is None:
        return ()
    try:
        import numpy as np

        channels = np.asarray(color, dtype=np.uint8).reshape(-1)
    except (TypeError, ValueError):
        return ()
    if channels.size < RGB_CHANNEL_COUNT:
        return ()
    return (int(channels[0]), int(channels[1]), int(channels[2]))
