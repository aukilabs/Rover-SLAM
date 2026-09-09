"""Validate prior pack v1 on-disk layout."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .schema import (
    FORMAT_VERSION,
    MIN_FORMAT_VERSION,
    PriorPackError,
    REQUIRED_COUNT_KEYS,
    REQUIRED_FILE_KEYS,
    REQUIRED_INTRINSIC_KEYS,
    REQUIRED_MANIFEST_KEYS,
    REQUIRED_POSE_KEYS,
    REQUIRED_QUAT_KEYS,
    REQUIRED_VEC3_KEYS,
    supported_format_version,
    validate_track_rgb,
)


def _load_json(path: Path) -> Any:
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        raise PriorPackError(f"invalid JSON in {path}: {e}") from e
    except OSError as e:
        raise PriorPackError(f"cannot read {path}: {e}") from e


def _require_keys(obj: Dict[str, Any], keys: Tuple[str, ...], label: str) -> None:
    missing = [k for k in keys if k not in obj]
    if missing:
        raise PriorPackError(f"{label} missing required field(s): {', '.join(missing)}")


def _require_vec3(obj: Dict[str, Any], label: str) -> None:
    _require_keys(obj, REQUIRED_VEC3_KEYS, label)
    for k in REQUIRED_VEC3_KEYS:
        if not isinstance(obj[k], (int, float)):
            raise PriorPackError(f"{label}.{k} must be numeric")


def _require_quat(obj: Dict[str, Any], label: str) -> None:
    _require_keys(obj, REQUIRED_QUAT_KEYS, label)
    for k in REQUIRED_QUAT_KEYS:
        if not isinstance(obj[k], (int, float)):
            raise PriorPackError(f"{label}.{k} must be numeric")


def _load_features(
    features_dir: Path, image_id: int, desc_dim: int
) -> Tuple[int, List[List[float]]]:
    npz_path = features_dir / f"{image_id}.npz"
    json_path = features_dir / f"{image_id}.json"

    if npz_path.is_file():
        try:
            import numpy as np
        except ImportError as e:
            raise PriorPackError(
                f"features/{image_id}.npz requires numpy to validate: {e}"
            ) from e
        data = np.load(npz_path)
        if "keypoints" not in data or "descriptors" not in data:
            raise PriorPackError(
                f"features/{image_id}.npz must contain keypoints and descriptors arrays"
            )
        kpts = data["keypoints"]
        desc = data["descriptors"]
        if kpts.ndim != 2 or kpts.shape[1] != 2:
            raise PriorPackError(
                f"features/{image_id}.npz keypoints must be shape N×2, got {kpts.shape}"
            )
        if desc.ndim != 2 or desc.shape[1] != desc_dim:
            raise PriorPackError(
                f"features/{image_id}.npz descriptors must be N×{desc_dim}, got {desc.shape}"
            )
        if kpts.shape[0] != desc.shape[0]:
            raise PriorPackError(
                f"features/{image_id}.npz keypoints/descriptors row count mismatch"
            )
        return int(kpts.shape[0]), []

    if json_path.is_file():
        feat = _load_json(json_path)
        kpts = feat.get("keypoints")
        desc = feat.get("descriptors")
        if not isinstance(kpts, list) or not isinstance(desc, list):
            raise PriorPackError(
                f"features/{image_id}.json must contain keypoints and descriptors arrays"
            )
        if len(kpts) != len(desc):
            raise PriorPackError(
                f"features/{image_id}.json keypoints/descriptors length mismatch"
            )
        for row in kpts:
            if not isinstance(row, list) or len(row) != 2:
                raise PriorPackError(
                    f"features/{image_id}.json each keypoint must be [x, y]"
                )
        for row in desc:
            if not isinstance(row, list) or len(row) != desc_dim:
                raise PriorPackError(
                    f"features/{image_id}.json each descriptor must have dim {desc_dim}"
                )
        return len(kpts), kpts

    raise PriorPackError(
        f"missing features for image id {image_id}: expected "
        f"features/{image_id}.npz or features/{image_id}.json"
    )


def validate_prior_pack(pack_root: Path) -> Dict[str, Any]:
    """Validate pack at pack_root. Returns manifest dict on success."""
    pack_root = pack_root.resolve()
    if not pack_root.is_dir():
        raise PriorPackError(f"pack root is not a directory: {pack_root}")

    manifest_path = pack_root / "manifest.json"
    if not manifest_path.is_file():
        raise PriorPackError("missing required file: manifest.json")

    manifest = _load_json(manifest_path)
    if not isinstance(manifest, dict):
        raise PriorPackError("manifest.json must be a JSON object")

    _require_keys(manifest, REQUIRED_MANIFEST_KEYS, "manifest")

    version = manifest["format_version"]
    if not supported_format_version(version):
        raise PriorPackError(
            f"unsupported format_version {version} "
            f"(expected {MIN_FORMAT_VERSION}–{FORMAT_VERSION})"
        )

    coord = manifest["coordinate_frame"]
    if not isinstance(coord, dict):
        raise PriorPackError("manifest.coordinate_frame must be an object")
    if "name" not in coord or "notes" not in coord:
        raise PriorPackError("manifest.coordinate_frame requires name and notes")

    descriptor = manifest["descriptor"]
    if not isinstance(descriptor, dict):
        raise PriorPackError("manifest.descriptor must be an object")
    if "dim" not in descriptor:
        raise PriorPackError("manifest.descriptor missing dim")
    desc_dim = int(descriptor["dim"])
    if desc_dim <= 0:
        raise PriorPackError("manifest.descriptor.dim must be positive")

    files = manifest["files"]
    if not isinstance(files, dict):
        raise PriorPackError("manifest.files must be an object")
    _require_keys(files, REQUIRED_FILE_KEYS, "manifest.files")

    counts = manifest["counts"]
    if not isinstance(counts, dict):
        raise PriorPackError("manifest.counts must be an object")
    _require_keys(counts, REQUIRED_COUNT_KEYS, "manifest.counts")

    images_path = pack_root / files["images"]
    portals_path = pack_root / files["portals"]
    covis_path = pack_root / files["covisibility"]
    tracks_path = pack_root / files["tracks"]
    features_dir = pack_root / files["features_dir"]

    for label, path in (
        ("images", images_path),
        ("portals", portals_path),
        ("covisibility", covis_path),
        ("tracks", tracks_path),
    ):
        if not path.is_file():
            raise PriorPackError(f"missing required file: {label} ({path.relative_to(pack_root)})")

    if not features_dir.is_dir():
        raise PriorPackError(
            f"missing required directory: features_dir ({features_dir.relative_to(pack_root)})"
        )

    images_doc = _load_json(images_path)
    if not isinstance(images_doc, dict) or "images" not in images_doc:
        raise PriorPackError("images.json must be an object with an images array")
    images = images_doc["images"]
    if not isinstance(images, list):
        raise PriorPackError("images.json images must be an array")

    image_ids: List[int] = []
    for idx, img in enumerate(images):
        label = f"images[{idx}]"
        if not isinstance(img, dict):
            raise PriorPackError(f"{label} must be an object")
        if "id" not in img or "path" not in img or "intrinsics" not in img or "pose" not in img:
            raise PriorPackError(f"{label} missing id, path, intrinsics, or pose")
        image_id = img["id"]
        if not isinstance(image_id, int):
            raise PriorPackError(f"{label}.id must be int")
        image_ids.append(image_id)

        intr = img["intrinsics"]
        if not isinstance(intr, dict):
            raise PriorPackError(f"{label}.intrinsics must be an object")
        _require_keys(intr, REQUIRED_INTRINSIC_KEYS, f"{label}.intrinsics")

        pose = img["pose"]
        if not isinstance(pose, dict):
            raise PriorPackError(f"{label}.pose must be an object")
        _require_keys(pose, REQUIRED_POSE_KEYS, f"{label}.pose")
        _require_vec3(pose["position"], f"{label}.pose.position")
        _require_quat(pose["rotation"], f"{label}.pose.rotation")

        rel_path = img["path"]
        if not isinstance(rel_path, str) or not rel_path:
            raise PriorPackError(f"{label}.path must be a non-empty string")
        blob_path = pack_root / rel_path
        if not blob_path.is_file():
            raise PriorPackError(
                f"{label} image blob not found: {rel_path} (resolved {blob_path})"
            )

    if image_ids != list(range(len(image_ids))):
        raise PriorPackError(
            f"image ids must be contiguous from 0: got {image_ids}"
        )

    num_keypoints: Dict[int, int] = {}
    for image_id in image_ids:
        n_kpts, _ = _load_features(features_dir, image_id, desc_dim)
        num_keypoints[image_id] = n_kpts

    portals_doc = _load_json(portals_path)
    if not isinstance(portals_doc, dict) or "portals" not in portals_doc:
        raise PriorPackError("portals.json must be an object with portals array")
    portals = portals_doc["portals"]
    if not isinstance(portals, list):
        raise PriorPackError("portals.json portals must be an array")
    portal_ids: set[str] = set()
    for idx, portal in enumerate(portals):
        label = f"portals[{idx}]"
        if not isinstance(portal, dict):
            raise PriorPackError(f"{label} must be an object")
        if "shortId" not in portal or "physicalSize" not in portal or "pose" not in portal:
            raise PriorPackError(f"{label} missing shortId, physicalSize, or pose")
        short_id = portal["shortId"]
        if not isinstance(short_id, str) or not short_id:
            raise PriorPackError(f"{label}.shortId must be a non-empty string")
        portal_ids.add(short_id)
        size = portal["physicalSize"]
        if not isinstance(size, (int, float)) or size <= 0:
            raise PriorPackError(f"{label}.physicalSize must be positive")
        pose = portal["pose"]
        _require_keys(pose, REQUIRED_POSE_KEYS, f"{label}.pose")
        _require_vec3(pose["position"], f"{label}.pose.position")
        _require_quat(pose["rotation"], f"{label}.pose.rotation")

    covis_doc = _load_json(covis_path)
    if not isinstance(covis_doc, dict) or "edges" not in covis_doc:
        raise PriorPackError("covisibility.json must be an object with edges array")
    edges = covis_doc["edges"]
    if not isinstance(edges, list):
        raise PriorPackError("covisibility.json edges must be an array")
    image_id_set = set(image_ids)
    for idx, edge in enumerate(edges):
        label = f"covisibility.edges[{idx}]"
        if not isinstance(edge, dict):
            raise PriorPackError(f"{label} must be an object")
        if "i" not in edge or "j" not in edge or "shared_tracks" not in edge:
            raise PriorPackError(f"{label} missing i, j, or shared_tracks")
        i, j = edge["i"], edge["j"]
        if not isinstance(i, int) or not isinstance(j, int):
            raise PriorPackError(f"{label}.i and .j must be int")
        if i >= j:
            raise PriorPackError(f"{label}: require i < j")
        if i not in image_id_set or j not in image_id_set:
            raise PriorPackError(f"{label}: image id out of range")
        if not isinstance(edge["shared_tracks"], int) or edge["shared_tracks"] < 0:
            raise PriorPackError(f"{label}.shared_tracks must be non-negative int")

    tracks_doc = _load_json(tracks_path)
    if not isinstance(tracks_doc, dict) or "tracks" not in tracks_doc:
        raise PriorPackError("tracks.json must be an object with tracks array")
    tracks = tracks_doc["tracks"]
    if not isinstance(tracks, list):
        raise PriorPackError("tracks.json tracks must be an array")
    track_ids: List[int] = []
    for idx, track in enumerate(tracks):
        label = f"tracks[{idx}]"
        if not isinstance(track, dict):
            raise PriorPackError(f"{label} must be an object")
        if "id" not in track or "position" not in track or "observations" not in track:
            raise PriorPackError(f"{label} missing id, position, or observations")
        track_id = track["id"]
        if not isinstance(track_id, int):
            raise PriorPackError(f"{label}.id must be int")
        track_ids.append(track_id)
        _require_vec3(track["position"], f"{label}.position")
        if "rgb" in track:
            validate_track_rgb(track["rgb"], label)
        obs = track["observations"]
        if not isinstance(obs, list) or len(obs) == 0:
            raise PriorPackError(f"{label}.observations must be a non-empty array")
        for oidx, ob in enumerate(obs):
            ob_label = f"{label}.observations[{oidx}]"
            if not isinstance(ob, dict):
                raise PriorPackError(f"{ob_label} must be an object")
            if "image_id" not in ob or "keypoint_index" not in ob:
                raise PriorPackError(f"{ob_label} missing image_id or keypoint_index")
            img_id = ob["image_id"]
            kpt_idx = ob["keypoint_index"]
            if not isinstance(img_id, int) or img_id not in image_id_set:
                raise PriorPackError(f"{ob_label}.image_id invalid: {img_id}")
            if not isinstance(kpt_idx, int) or kpt_idx < 0:
                raise PriorPackError(f"{ob_label}.keypoint_index must be non-negative int")
            if kpt_idx >= num_keypoints[img_id]:
                raise PriorPackError(
                    f"{ob_label}.keypoint_index {kpt_idx} >= num keypoints "
                    f"{num_keypoints[img_id]} for image {img_id}"
                )

    if track_ids != list(range(len(track_ids))):
        raise PriorPackError(f"track ids must be contiguous from 0: got {track_ids}")

    expected_images = int(counts["images"])
    expected_tracks = int(counts["tracks"])
    expected_portals = int(counts["portals"])
    if len(image_ids) != expected_images:
        raise PriorPackError(
            f"manifest.counts.images={expected_images} but found {len(image_ids)} images"
        )
    if len(tracks) != expected_tracks:
        raise PriorPackError(
            f"manifest.counts.tracks={expected_tracks} but found {len(tracks)} tracks"
        )
    if len(portals) != expected_portals:
        raise PriorPackError(
            f"manifest.counts.portals={expected_portals} but found {len(portals)} portals"
        )

    return manifest


def validate_prior_pack_or_exit(pack_root: Path) -> int:
    try:
        manifest = validate_prior_pack(pack_root)
        counts = manifest["counts"]
        print(
            f"OK: prior pack v{manifest['format_version']} at {pack_root} "
            f"({counts['images']} images, {counts['tracks']} tracks, "
            f"{counts['portals']} portals)"
        )
        return 0
    except PriorPackError as e:
        print(f"INVALID prior pack: {e}", flush=True)
        return 1
