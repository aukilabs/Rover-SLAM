"""Prior pack v1 schema constants and shared helpers."""

from __future__ import annotations

FORMAT_VERSION = 1
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
