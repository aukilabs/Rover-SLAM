#!/usr/bin/env python3
"""Export refined global COLMAP + per-scan hloc features.h5 to prior-pack recon layout.

Coordinate / pose math (projection-critical pack domain — NOT recon-server viz quats):

  World remap M maps COLMAP (x,y,z) -> (y,x,-z):
      M = [[0,1,0],[1,0,0],[0,0,-1]]

  Pack images/tracks use left-multiply only (preserves OpenCV camera +Z for pinhole):
      P' = M @ P,  t' = M @ t,  R' = M @ R

  Do NOT use reconstruction-server convert_pose_colmap_to_opengl for pack images/tracks:
      that componentwise quaternion swap is conjugation R' = M @ R @ M and breaks track
      self-reproj (~500px). Recon-server still uses conjugation for viz/portal CSV export.

  Portals from refined_manifest.json are already conjugation-OpenGL (R_conj = M R M, t'=M t).
  Pack portals: position unchanged; R_pack = R_conj @ M (= M R).

  - COLMAP pycolmap image.cam_from_world is world->camera (T_cw); pack image pose is Twc
      (cam_from_world.inverse()) then M @ R_wc / M @ t_wc as above.

Image / feature naming:
  - COLMAP image.name, features.h5 group keys, and datasets/<scan>/Frames/<name> must match
    exactly (e.g. 2026-04-02_13-38-55_000000.jpg). Keypoints in h5 are pixel (x,y) on those
    JPEGs with no EXIF rotation; track keypoint_index is the hloc/COLMAP point2D index
    (same row in features.h5 keypoints array).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

try:
    import h5py
except ImportError as exc:
    raise SystemExit("h5py required: pip install h5py") from exc

try:
    import pycolmap
except ImportError as exc:
    raise SystemExit("pycolmap required: pip install pycolmap") from exc

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.prior_pack.schema import colmap_point_rgb  # noqa: E402

try:
    import cv2
except ImportError as exc:
    cv2 = None  # type: ignore
    _cv2_import_error = exc
else:
    _cv2_import_error = None

# --- pack domain: world remap M (COLMAP xyz -> y,x,-z) ---

WORLD_REMAP_M = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], dtype=np.float64)

# Self-reproj gate thresholds (OpenCV +Z pinhole on packed Twc + tracks).
SELF_REPROJ_P50_MAX_PX = 2.0
SELF_REPROJ_FRAC_LT_12PX_MIN = 0.95
SELF_REPROJ_MAX_SAMPLES = 500


def convert_point_colmap_to_opengl(xyz: np.ndarray) -> np.ndarray:
    """Point position: P' = M @ P."""
    return WORLD_REMAP_M @ np.asarray(xyz, dtype=np.float64)


def rotmat_from_wxyz(q: Dict[str, float]) -> np.ndarray:
    w, x, y, z = float(q["w"]), float(q["x"]), float(q["y"]), float(q["z"])
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def wxyz_from_rotmat(R: np.ndarray) -> Dict[str, float]:
    """Rotation matrix -> unit quaternion wxyz (Shepperd's method)."""
    m = np.asarray(R, dtype=np.float64)
    tr = float(np.trace(m))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    q /= np.linalg.norm(q) + 1e-12
    return {"w": float(q[0]), "x": float(q[1]), "y": float(q[2]), "z": float(q[3])}


def colmap_twc_to_pack_pose(cam_from_world) -> Dict[str, Any]:
    """COLMAP T_cw -> pack-domain Twc: t'=M t, R'=M R_wc (not conjugation)."""
    if callable(cam_from_world):
        cam_from_world = cam_from_world()
    twc = cam_from_world.inverse()
    t_wc = np.asarray(twc.translation, dtype=np.float64)
    R_wc = np.asarray(twc.rotation.matrix(), dtype=np.float64)
    t_gl = WORLD_REMAP_M @ t_wc
    R_gl = WORLD_REMAP_M @ R_wc
    return {
        "position": {"x": float(t_gl[0]), "y": float(t_gl[1]), "z": float(t_gl[2])},
        "rotation": wxyz_from_rotmat(R_gl),
    }


def camera_intrinsics(cam: pycolmap.Camera) -> Dict[str, float]:
    model = cam.model.name
    p = cam.params
    if model in ("SIMPLE_PINHOLE", "SIMPLE_RADIAL"):
        fx = fy = float(p[0])
        cx, cy = float(p[1]), float(p[2])
    elif model == "PINHOLE":
        fx, fy, cx, cy = float(p[0]), float(p[1]), float(p[2]), float(p[3])
    elif model == "OPENCV":
        fx, fy, cx, cy = float(p[0]), float(p[1]), float(p[2]), float(p[3])
    else:
        raise ValueError(f"unsupported COLMAP camera model {model}")
    return {"fx": fx, "fy": fy, "cx": cx, "cy": cy}


DEFAULT_JOB = (
    Path(__file__).resolve().parents[2]
    / "reconstruction-server/_local/job-artifacts/"
    "20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete"
)

HOLDOUT_SCAN = "2026-04-02_13-42-53"
HOLDOUT_TOKEN = "13-42-53"
MAP_SCANS = (
    "2026-04-02_13-38-55",
    "2026-04-02_13-40-15",
    "2026-04-02_13-41-55",
)

SCAN_FROM_NAME = re.compile(r"^(20\d{2}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})_")


def resolve_job_path(raw: Path) -> Path:
    p = raw.resolve()
    if p.exists():
        return p
    # Allow running from WSL against a Windows-style path passed from docs.
    if str(raw).startswith("D:") or str(raw).startswith("d:"):
        wsl = Path("/mnt/d") / Path(str(raw)[3:].replace("\\", "/"))
        if wsl.exists():
            return wsl
    raise FileNotFoundError(f"job artifacts not found: {raw}")


def scan_from_image_name(name: str) -> Optional[str]:
    m = SCAN_FROM_NAME.match(name)
    return m.group(1) if m else None


def is_holdout_image(name: str) -> bool:
    return HOLDOUT_SCAN in name or HOLDOUT_TOKEN in name


def is_map_image(name: str) -> bool:
    scan = scan_from_image_name(name)
    return scan in MAP_SCANS if scan else False


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def _link_or_copy_image(src: Path, dst: Path, mode: str) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        dst.unlink()
    if mode == "copy":
        shutil.copy2(src, dst)
        return
    try:
        os.symlink(src, dst)
    except OSError:
        # Fallback when symlinks are unavailable (e.g. Windows without privilege).
        shutil.copy2(src, dst)


def portal_from_refined_manifest(entry: Dict[str, Any]) -> Dict[str, Any]:
    """Convert conjugation-OpenGL manifest portal to pack OpenGL (R_pack = R_conj @ M)."""
    pose = entry["pose"]
    pos = pose["position"]
    R_conj = rotmat_from_wxyz(pose["rotation"])
    R_pack = R_conj @ WORLD_REMAP_M
    return {
        "shortId": entry["shortId"],
        "physicalSize": entry["physicalSize"],
        "pose": {
            "position": {"x": float(pos["x"]), "y": float(pos["y"]), "z": float(pos["z"])},
            "rotation": wxyz_from_rotmat(R_pack),
        },
    }


# slam_server ingest: q_domain = q_pack * wall_to_qr (−π/2 about Y, Eigen/scipy convention).
_WALL_ANGLE = -0.5 * math.pi
_WC, _WS = math.cos(_WALL_ANGLE), math.sin(_WALL_ANGLE)
PORTAL_WALL_TO_QR_R = np.array(
    [[_WC, 0.0, _WS], [0.0, 1.0, 0.0], [-_WS, 0.0, _WC]],
    dtype=np.float64,
)


def _portal_object_points(physical_size: float) -> np.ndarray:
    half = 0.5 * float(physical_size)
    return np.array(
        [[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]],
        dtype=np.float64,
    )


_PORTAL_URL_PREFIX = "https://r8.hr/"
_SEED_PORTAL_SHORT_IDS = frozenset({"HBORRXLUOD6", "MQAUVO7PXZ1"})


def _parse_portal_short_id(decoded: str) -> Optional[str]:
    """Match slam_server parsePortalShortId: https://r8.hr/<alnum> → upper shortId."""
    text = decoded.strip()
    lower = text.lower()
    if not lower.startswith(_PORTAL_URL_PREFIX):
        return None
    candidate = text[len(_PORTAL_URL_PREFIX) :].strip()
    if not candidate or not candidate.isalnum():
        return None
    return candidate.upper()


def _qr_corners(points: Any, detection_idx: int) -> Optional[np.ndarray]:
    """Extract 4×2 corner pixels for one QR detection (OpenCV detectAndDecodeMulti layout)."""
    arr = np.asarray(points, dtype=np.float64)
    if arr.ndim == 3 and arr.shape[0] > detection_idx:
        return arr[detection_idx].reshape(4, 2)
    flat = arr.reshape(-1, arr.shape[-1])
    base = detection_idx * 4
    if flat.shape[0] < base + 4 or flat.shape[1] < 2:
        return None
    corners = flat[base : base + 4, :2]
    if not np.all(np.isfinite(corners)):
        return None
    return corners


def _map_images_by_distance_to_portal(
    portal: Dict[str, Any], images_out: List[Dict[str, Any]]
) -> List[Tuple[float, Dict[str, Any]]]:
    pos = portal["pose"]["position"]
    p = np.array([float(pos["x"]), float(pos["y"]), float(pos["z"])], dtype=np.float64)
    ranked: List[Tuple[float, Dict[str, Any]]] = []
    for img in images_out:
        ip = img["pose"]["position"]
        q = np.array([float(ip["x"]), float(ip["y"]), float(ip["z"])], dtype=np.float64)
        ranked.append((float(np.linalg.norm(q - p)), img))
    ranked.sort(key=lambda t: t[0])
    return ranked


def _find_portal_qr_on_image(
    gray: Any, short_id: str
) -> Tuple[Optional[np.ndarray], Optional[str]]:
    """Return (4×2 corners, decoded_text) for QR matching portal shortId, else (None, reason)."""
    ok, decoded_info, points, _ = cv2.QRCodeDetector().detectAndDecodeMulti(gray)
    if not ok or points is None or len(points) == 0:
        return None, "qr_not_detected"
    decoded_list = list(decoded_info) if decoded_info is not None else []
    for i in range(len(points)):
        decoded = str(decoded_list[i]) if i < len(decoded_list) else ""
        parsed = _parse_portal_short_id(decoded)
        if parsed != short_id:
            continue
        corners = _qr_corners(points, i)
        if corners is not None:
            return corners, decoded
    return None, "qr_shortid_mismatch"


def calibrate_portal_rotations_to_map(
    portals_out: List[Dict[str, Any]],
    images_out: List[Dict[str, Any]],
    output_root: Path,
    *,
    skip_images: bool,
) -> List[Dict[str, str]]:
    """Rotate R_pack so QR PnP on a map image with visible portal QR recovers M@R Twc."""
    if skip_images:
        return [{"shortId": p["shortId"], "status": "skipped_no_images"} for p in portals_out]
    if cv2 is None:
        raise SystemExit(
            "opencv-python required for portal R calibration: pip install opencv-python"
        ) from _cv2_import_error

    notes: List[Dict[str, str]] = []
    calibrated_ids: set[str] = set()
    for portal in portals_out:
        short_id = str(portal["shortId"])
        ranked = _map_images_by_distance_to_portal(portal, images_out)
        if not ranked:
            notes.append({"shortId": short_id, "status": "no_map_image"})
            continue

        ip: Optional[np.ndarray] = None
        nearest: Optional[Dict[str, Any]] = None
        last_reason = "qr_not_detected"
        images_tried = 0
        for dist_m, candidate in ranked:
            images_tried += 1
            image_path = output_root / str(candidate["path"])
            if not image_path.is_file():
                last_reason = "missing_image_file"
                continue
            gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                last_reason = "unreadable_image"
                continue
            corners, reason = _find_portal_qr_on_image(gray, short_id)
            if corners is None:
                last_reason = reason or "qr_not_detected"
                continue
            ip = corners
            nearest = candidate
            break

        if ip is None or nearest is None:
            notes.append(
                {
                    "shortId": short_id,
                    "status": last_reason,
                    "images_tried": str(images_tried),
                }
            )
            continue

        intr = nearest["intrinsics"]
        K = np.array(
            [
                [float(intr["fx"]), 0.0, float(intr["cx"])],
                [0.0, float(intr["fy"]), float(intr["cy"])],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        obj = _portal_object_points(float(portal["physicalSize"]))

        R_pack = rotmat_from_wxyz(portal["pose"]["rotation"])
        t_portal = np.array(
            [
                float(portal["pose"]["position"]["x"]),
                float(portal["pose"]["position"]["y"]),
                float(portal["pose"]["position"]["z"]),
            ],
            dtype=np.float64,
        )
        R_dom = R_pack @ PORTAL_WALL_TO_QR_R

        pnp_ok, rvec, tvec = cv2.solvePnP(obj, ip.astype(np.float32), K, None, flags=cv2.SOLVEPNP_SQPNP)
        if not pnp_ok:
            notes.append({"shortId": short_id, "status": "pnp_failed"})
            continue

        R_cp, _ = cv2.Rodrigues(rvec)
        R_cam_domain = R_cp.reshape(3, 3) @ R_dom.T
        t_cam_domain = tvec.reshape(3) - R_cam_domain @ t_portal
        Twc_pnp_R = R_cam_domain.T

        Twc_map_R = rotmat_from_wxyz(nearest["pose"]["rotation"])
        R_delta = Twc_map_R @ Twc_pnp_R.T
        R_dom_cal = R_delta @ R_dom
        R_pack_cal = R_dom_cal @ PORTAL_WALL_TO_QR_R.T
        portal["pose"]["rotation"] = wxyz_from_rotmat(R_pack_cal)

        Twc_map_t = np.array(
            [
                float(nearest["pose"]["position"]["x"]),
                float(nearest["pose"]["position"]["y"]),
                float(nearest["pose"]["position"]["z"]),
            ],
            dtype=np.float64,
        )
        Twc_pnp_t = -Twc_pnp_R @ t_cam_domain
        pos_before = float(np.linalg.norm(Twc_pnp_t - Twc_map_t))

        R_cam_domain2 = R_cp.reshape(3, 3) @ R_dom_cal.T
        t_cam_domain2 = tvec.reshape(3) - R_cam_domain2 @ t_portal
        Twc_cal_t = -R_cam_domain2.T @ t_cam_domain2
        pos_after = float(np.linalg.norm(Twc_cal_t - Twc_map_t))

        calibrated_ids.add(short_id)
        notes.append(
            {
                "shortId": short_id,
                "status": "calibrated",
                "nearest_image_id": str(nearest["id"]),
                "pos_err_before_m": f"{pos_before:.4f}",
                "pos_err_after_m": f"{pos_after:.4f}",
            }
        )

    portal_ids = {str(p["shortId"]) for p in portals_out}
    missing_seed = sorted(portal_ids & _SEED_PORTAL_SHORT_IDS - calibrated_ids)
    if missing_seed:
        detail = [n for n in notes if n.get("shortId") in missing_seed]
        raise SystemExit(
            "portal R calibration FAILED: seed portals present but uncalibrated: "
            f"{missing_seed}; notes={json.dumps(detail, indent=2)}"
        )
    return notes


def export_colmap_h5_to_recon(
    job_root: Path,
    output_root: Path,
    *,
    max_images: Optional[int] = None,
    image_mode: str = "symlink",
    skip_images: bool = False,
) -> Dict[str, Any]:
    job_root = resolve_job_path(job_root)
    output_root = output_root.resolve()
    if output_root.exists():
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True)

    colmap_dir = job_root / "refined/global/refined_sfm_combined"
    manifest_path = job_root / "refined/global/refined_manifest.json"
    if not colmap_dir.is_dir():
        raise FileNotFoundError(f"missing COLMAP dir: {colmap_dir}")
    if not manifest_path.is_file():
        raise FileNotFoundError(f"missing refined manifest: {manifest_path}")

    rec = pycolmap.Reconstruction(str(colmap_dir))

    included: List[pycolmap.Image] = []
    for img in rec.images.values():
        if not is_map_image(img.name):
            continue
        if is_holdout_image(img.name):
            continue
        included.append(img)
    included.sort(key=lambda im: im.name)
    if max_images is not None:
        included = included[: max_images]

    if not included:
        raise ValueError("no map images selected after holdout exclusion")

    colmap_to_new: Dict[int, int] = {}
    images_out: List[Dict[str, Any]] = []
    features_dir = output_root / "features"
    features_dir.mkdir()
    images_dir = output_root / "images"

    h5_cache: Dict[str, h5py.File] = {}

    def features_h5_for_scan(scan: str) -> h5py.File:
        if scan not in h5_cache:
            path = job_root / f"refined/local/{scan}/sfm/features.h5"
            if not path.is_file():
                raise FileNotFoundError(f"missing features.h5 for scan {scan}: {path}")
            h5_cache[scan] = h5py.File(path, "r", libver="latest")
        return h5_cache[scan]

    missing_h5: List[str] = []
    missing_frames: List[str] = []

    for new_id, img in enumerate(included):
        colmap_to_new[img.image_id] = new_id
        scan = scan_from_image_name(img.name)
        if scan is None:
            raise ValueError(f"cannot parse scan from image name: {img.name}")

        frame_src = job_root / "datasets" / scan / "Frames" / img.name
        rel_path = f"images/{img.name}"
        if not skip_images:
            if not frame_src.is_file():
                missing_frames.append(str(frame_src))
            else:
                _link_or_copy_image(frame_src, output_root / rel_path, image_mode)

        h5 = features_h5_for_scan(scan)
        if img.name not in h5:
            missing_h5.append(img.name)
            continue
        grp = h5[img.name]
        kpts = np.asarray(grp["keypoints"], dtype=np.float32)
        desc = np.asarray(grp["descriptors"], dtype=np.float32)
        if desc.dtype == np.float16:
            desc = desc.astype(np.float32)
        if kpts.dtype == np.float16:
            kpts = kpts.astype(np.float32)
        # hloc ALIKED stores descriptors as D×N; prior-pack schema is N×D.
        if desc.ndim == 2 and desc.shape[0] != kpts.shape[0] and desc.shape[1] == kpts.shape[0]:
            desc = np.ascontiguousarray(desc.T)
        feat_path = features_dir / f"{new_id}.npz"
        out_feat: Dict[str, np.ndarray] = {"keypoints": kpts, "descriptors": desc}
        score_key = "scores" if "scores" in grp else ("keypoint_scores" if "keypoint_scores" in grp else None)
        if score_key is not None:
            scores = np.asarray(grp[score_key], dtype=np.float32)
            if scores.dtype == np.float16:
                scores = scores.astype(np.float32)
            out_feat["scores"] = scores
        np.savez_compressed(feat_path, **out_feat)

        cam = rec.cameras[img.camera_id]
        images_out.append(
            {
                "id": new_id,
                "path": rel_path,
                "intrinsics": camera_intrinsics(cam),
                "pose": colmap_twc_to_pack_pose(img.cam_from_world),
            }
        )

    for fd in h5_cache.values():
        fd.close()

    if missing_h5:
        raise FileNotFoundError(
            f"{len(missing_h5)} map images missing from local features.h5 "
            f"(first: {missing_h5[0]})"
        )
    if missing_frames and not skip_images:
        raise FileNotFoundError(
            f"{len(missing_frames)} frame JPEGs missing under datasets "
            f"(first: {missing_frames[0]})"
        )

    # Tracks + covisibility from global COLMAP, map observations only.
    tracks_out: List[Dict[str, Any]] = []
    pair_shared: Dict[Tuple[int, int], int] = defaultdict(int)
    next_track_id = 0

    for point in rec.points3D.values():
        observations: List[Dict[str, int]] = []
        seen_new_ids: List[int] = []
        for elem in point.track.elements:
            new_id = colmap_to_new.get(elem.image_id)
            if new_id is None:
                continue
            observations.append({"image_id": new_id, "keypoint_index": int(elem.point2D_idx)})
            seen_new_ids.append(new_id)
        if len(observations) < 2:
            continue

        pos_gl = convert_point_colmap_to_opengl(np.asarray(point.xyz, dtype=np.float64))
        track_entry: Dict[str, Any] = {
            "id": next_track_id,
            "position": {
                "x": float(pos_gl[0]),
                "y": float(pos_gl[1]),
                "z": float(pos_gl[2]),
            },
            "observations": observations,
        }
        rgb = colmap_point_rgb(point)
        if rgb:
            track_entry["rgb"] = list(rgb)
        tracks_out.append(track_entry)
        uniq = sorted(set(seen_new_ids))
        for a in range(len(uniq)):
            for b in range(a + 1, len(uniq)):
                pair_shared[(uniq[a], uniq[b])] += 1
        next_track_id += 1

    covis_edges = [
        {"i": i, "j": j, "shared_tracks": int(n)}
        for (i, j), n in sorted(pair_shared.items())
        if n > 0
    ]

    manifest_doc = json.loads(manifest_path.read_text(encoding="utf-8"))
    portals_out = [portal_from_refined_manifest(p) for p in manifest_doc.get("portals", [])]
    portal_calibration = calibrate_portal_rotations_to_map(
        portals_out, images_out, output_root, skip_images=skip_images
    )

    _write_json(output_root / "images.json", {"images": images_out})
    _write_json(output_root / "tracks.json", {"tracks": tracks_out})
    _write_json(output_root / "covisibility.json", {"edges": covis_edges})
    _write_json(output_root / "portals.json", {"portals": portals_out})

    meta = {
        "source_job": str(job_root),
        "colmap_dir": str(colmap_dir),
        "refined_manifest": str(manifest_path),
        "holdout_excluded": HOLDOUT_SCAN,
        "map_scans": list(MAP_SCANS),
        "counts": {
            "images": len(images_out),
            "tracks": len(tracks_out),
            "portals": len(portals_out),
            "covisibility_edges": len(covis_edges),
        },
        "conventions": {
            "image_pose": "COLMAP cam_from_world.inverse() Twc; t'=M t, R'=M R_wc (not conjugation)",
            "track_position": "COLMAP point XYZ via M @ P",
            "portal_pose": (
                "refined_manifest R_pack=R_conj@M; R_pack calibrated on nearest map image "
                "with visible portal QR (shortId match) so slam wall→QR ingest matches M@R Twc"
            ),
            "portal_calibration": portal_calibration,
            "physicalSize": "copied unchanged from refined_manifest.json",
            "keypoint_index": "COLMAP point2D index == row in per-scan features.h5 keypoints",
            "image_names": "COLMAP name == h5 group == datasets/<scan>/Frames/<name>",
        },
    }
    _write_json(output_root / "export_meta.json", meta)
    return meta


def validate_recon_shape(recon_root: Path) -> None:
    """Lightweight JSON shape check (slice B dry-run helper)."""
    recon_root = recon_root.resolve()
    images = json.loads((recon_root / "images.json").read_text(encoding="utf-8"))["images"]
    tracks = json.loads((recon_root / "tracks.json").read_text(encoding="utf-8"))["tracks"]
    portals = json.loads((recon_root / "portals.json").read_text(encoding="utf-8"))["portals"]
    edges = json.loads((recon_root / "covisibility.json").read_text(encoding="utf-8"))["edges"]
    if not images:
        raise ValueError("images.json empty")
    for i, img in enumerate(images):
        if img["id"] != i:
            raise ValueError(f"image ids not contiguous at {i}")
        for key in ("path", "intrinsics", "pose"):
            if key not in img:
                raise ValueError(f"images[{i}] missing {key}")
        feat = recon_root / "features" / f"{i}.npz"
        if not feat.is_file():
            raise ValueError(f"missing features/{i}.npz")
    if len(portals) < 1:
        raise ValueError("portals.json empty")
    for p in portals:
        if "shortId" not in p or "physicalSize" not in p or "pose" not in p:
            raise ValueError("portal entry incomplete")
    for t_idx, track in enumerate(tracks):
        if track["id"] != t_idx:
            raise ValueError("track ids not contiguous")
        if not track.get("observations"):
            raise ValueError(f"track {t_idx} has no observations")
    for edge in edges:
        if edge["i"] >= edge["j"]:
            raise ValueError("covisibility edge requires i < j")


def _project_opencv(
    point_world: np.ndarray,
    twc_pos: np.ndarray,
    twc_R: np.ndarray,
    intr: Dict[str, float],
) -> Tuple[float, float, float]:
    """OpenCV pinhole (+Z forward). Returns (u, v, z_cam) or z_cam<=0 if behind."""
    R_cw = twc_R.T
    p_cam = R_cw @ (point_world - twc_pos)
    z = float(p_cam[2])
    if z <= 1e-6:
        return 0.0, 0.0, z
    u = intr["fx"] * p_cam[0] / z + intr["cx"]
    v = intr["fy"] * p_cam[1] / z + intr["cy"]
    return float(u), float(v), z


def validate_track_self_reproj(
    recon_root: Path,
    *,
    max_samples: int = SELF_REPROJ_MAX_SAMPLES,
    p50_max_px: float = SELF_REPROJ_P50_MAX_PX,
    frac_lt_12_min: float = SELF_REPROJ_FRAC_LT_12PX_MIN,
) -> Dict[str, Any]:
    """Sample track observations vs packed features + Twc; fail if reproj is bad."""
    recon_root = recon_root.resolve()
    images = json.loads((recon_root / "images.json").read_text(encoding="utf-8"))["images"]
    tracks = json.loads((recon_root / "tracks.json").read_text(encoding="utf-8"))["tracks"]

    img_by_id = {int(im["id"]): im for im in images}
    obs_by_image: Dict[int, List[Tuple[int, int, np.ndarray]]] = defaultdict(list)
    for track in tracks:
        pos = track["position"]
        p_w = np.array([pos["x"], pos["y"], pos["z"]], dtype=np.float64)
        for obs in track["observations"]:
            obs_by_image[int(obs["image_id"])].append(
                (int(track["id"]), int(obs["keypoint_index"]), p_w)
            )

    # Prefer images with many observations (fast, stable gate).
    ranked = sorted(obs_by_image.items(), key=lambda kv: len(kv[1]), reverse=True)
    errors: List[float] = []
    samples_used = 0
    images_used: List[int] = []

    for image_id, obs_list in ranked:
        if samples_used >= max_samples:
            break
        img = img_by_id[image_id]
        feat_path = recon_root / "features" / f"{image_id}.npz"
        if not feat_path.is_file():
            continue
        kpts = np.load(feat_path)["keypoints"]
        twc_pos = np.array(
            [img["pose"]["position"]["x"], img["pose"]["position"]["y"], img["pose"]["position"]["z"]],
            dtype=np.float64,
        )
        twc_R = rotmat_from_wxyz(img["pose"]["rotation"])
        intr = img["intrinsics"]
        images_used.append(image_id)

        for _tid, kp_idx, p_w in obs_list:
            if samples_used >= max_samples:
                break
            if kp_idx < 0 or kp_idx >= len(kpts):
                continue
            u, v, z = _project_opencv(p_w, twc_pos, twc_R, intr)
            if z <= 1e-6:
                continue
            du = u - float(kpts[kp_idx, 0])
            dv = v - float(kpts[kp_idx, 1])
            errors.append(math.hypot(du, dv))
            samples_used += 1

    if len(errors) < 50:
        raise ValueError(f"self-reproj gate: too few valid samples ({len(errors)})")

    err = np.asarray(errors, dtype=np.float64)
    p50 = float(np.percentile(err, 50))
    frac_lt_12 = float(np.mean(err < 12.0))
    stats = {
        "samples": int(len(err)),
        "images_used": images_used[:5],
        "p50_px": p50,
        "p95_px": float(np.percentile(err, 95)),
        "frac_lt_12px": frac_lt_12,
        "thresholds": {
            "p50_max_px": p50_max_px,
            "frac_lt_12px_min": frac_lt_12_min,
            "max_samples": max_samples,
        },
    }

    if p50 > p50_max_px or frac_lt_12 < frac_lt_12_min:
        raise ValueError(
            "self-reproj gate FAILED: "
            f"p50={p50:.2f}px (max {p50_max_px}), "
            f"frac<12px={frac_lt_12:.3f} (min {frac_lt_12_min}); "
            f"samples={len(err)} images={images_used[:3]}"
        )
    return stats


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Export refined global COLMAP + map-scan features.h5 to recon layout."
    )
    parser.add_argument(
        "--job-artifacts",
        type=Path,
        default=DEFAULT_JOB,
        help="Job artifact root (refined/global/refined_sfm_combined + refined_manifest.json)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output recon layout directory",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=None,
        help="Limit exported map images (dry-run / smoke)",
    )
    parser.add_argument(
        "--image-mode",
        choices=("symlink", "copy"),
        default="symlink",
        help="How to populate recon images/ from dataset Frames/",
    )
    parser.add_argument(
        "--skip-images",
        action="store_true",
        help="Skip linking/copying JPEGs (features + JSON still exported)",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="After export, run recon JSON shape + track self-reproj validation",
    )
    parser.add_argument(
        "--skip-self-reproj",
        action="store_true",
        help="Skip track self-reproj gate (not recommended)",
    )
    parser.add_argument(
        "--reproj-check-only",
        type=Path,
        default=None,
        metavar="RECON",
        help="Run self-reproj gate on existing recon layout and exit",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.reproj_check_only is not None:
        stats = validate_track_self_reproj(args.reproj_check_only)
        print("track self-reproj gate: OK", json.dumps(stats, indent=2))
        return 0

    if args.output is None:
        parser.error("--output is required unless --reproj-check-only is set")

    meta = export_colmap_h5_to_recon(
        args.job_artifacts,
        args.output,
        max_images=args.max_images,
        image_mode=args.image_mode,
        skip_images=args.skip_images,
    )
    print(json.dumps(meta["counts"], indent=2))
    if args.validate_only or args.max_images is not None:
        validate_recon_shape(args.output)
        print("recon layout validation: OK")
    if not args.skip_self_reproj:
        stats = validate_track_self_reproj(args.output)
        print("track self-reproj gate: OK", json.dumps(stats, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
