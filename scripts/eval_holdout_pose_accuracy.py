#!/usr/bin/env python3
"""Holdout VO pose accuracy vs local COLMAP refinement (Umeyama + top-down plot).

1) Replay Frames/ into slam_server with capture timestamps from Frames.csv
2) Fetch /api/v1/trajectory?aligned=0&all_maps=1&dense=1
3) Load local refinement COLMAP bins, convert camera centers COLMAP->OpenGL
   using reconstruction-server convert_pose_colmap_to_opengl
4) Match by timestamp -> image index; Umeyama-align each SLAM map (Sim3 on
   positions); report translation ATE + rotation geodesic error after same R.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

import numpy as np
import requests

try:
    import pycolmap
except ImportError as e:
    raise SystemExit("pycolmap required: pip install pycolmap") from e

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as e:
    raise SystemExit("matplotlib required") from e

# --- pose convention (mirror reconstruction-server/utils/data_utils.py) ---


def convert_pose_opengl_to_colmap(position, quaternion):
    position = np.array([position[1], position[0], position[2] * -1], dtype=np.float64)
    quaternion = np.array(
        [quaternion[1], quaternion[0], quaternion[2] * -1, quaternion[3]],
        dtype=np.float64,
    )
    return position, quaternion


def convert_pose_colmap_to_opengl(position, quaternion):
    return convert_pose_opengl_to_colmap(position, quaternion)


def umeyama_alignment(src: np.ndarray, dst: np.ndarray, with_scale: bool = True):
    """src, dst: (3, N). Returns R (3x3), t (3,), s (float) mapping src -> dst."""
    assert src.shape == dst.shape and src.shape[0] == 3 and src.shape[1] >= 3
    n = src.shape[1]
    mu_s = src.mean(axis=1, keepdims=True)
    mu_d = dst.mean(axis=1, keepdims=True)
    src_c = src - mu_s
    dst_c = dst - mu_d
    cov = (dst_c @ src_c.T) / n
    U, S, Vt = np.linalg.svd(cov)
    d = np.ones(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        d[-1] = -1
    R = U @ np.diag(d) @ Vt
    var_s = (src_c**2).sum() / n
    s = float((S * d).sum() / var_s) if with_scale and var_s > 1e-12 else 1.0
    t = (mu_d - s * (R @ mu_s)).reshape(3)
    return R, t, s


def apply_sim3(pts: np.ndarray, R: np.ndarray, t: np.ndarray, s: float) -> np.ndarray:
    """pts (N,3) -> (N,3)"""
    return (s * (R @ pts.T)).T + t.reshape(1, 3)


def rotmat_from_xyzw(q) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotmat_from_wxyz(q) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    return rotmat_from_xyzw([x, y, z, w])


def rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """Geodesic angle between two rotation matrices (degrees)."""
    R_err = R_a.T @ R_b
    cos_theta = (np.trace(R_err) - 1.0) * 0.5
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    return float(math.degrees(math.acos(cos_theta)))


def average_so3(mats: List[np.ndarray]) -> np.ndarray:
    """SVD average of rotation matrices (special orthogonal projection)."""
    M = np.zeros((3, 3), dtype=np.float64)
    for R in mats:
        M += R
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def rot_stats(err_deg: np.ndarray) -> dict:
    return {
        "rot_rmse_deg": float(np.sqrt(np.mean(err_deg**2))),
        "rot_mean_deg": float(np.mean(err_deg)),
        "rot_median_deg": float(np.median(err_deg)),
        "rot_p95_deg": float(np.percentile(err_deg, 95)),
        "rot_max_deg": float(np.max(err_deg)),
    }


def topdown_axes(points: np.ndarray) -> Tuple[int, int, int]:
    """Return (h_axis, u_axis, v_axis): least-variance axis = height/up."""
    var = points.var(axis=0)
    h = int(np.argmin(var))
    horiz = [i for i in range(3) if i != h]
    return h, horiz[0], horiz[1]


def parse_image_index(name: str) -> Optional[int]:
    m = re.search(r"_(\d+)\.(jpg|jpeg|png|bmp)$", name, re.I)
    return int(m.group(1)) if m else None


def load_frames_csv(path: Path) -> List[float]:
    ts = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            ts.append(float(parts[0]))
    return ts


def load_camera_intrinsics_csv(path: Path) -> List[List[float]]:
    intrinsics = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            if len(parts) < 5:
                raise RuntimeError(
                    f"expected timestamp,fx,fy,cx,cy,... in {path}: {line!r}"
                )
            intrinsics.append(
                [float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])]
            )
    return intrinsics


def load_gt_opengl(
    colmap_dir: Path,
    name_contains: Optional[str] = None,
) -> Dict[int, Tuple[np.ndarray, np.ndarray]]:
    """image_index -> (camera center, quat xyzw) in OpenGL world.

    If name_contains is set (e.g. holdout scan id), only images whose name
    contains that substring are kept — required for global SfM where multiple
    scans share trailing frame indices.
    """
    rec = pycolmap.Reconstruction(str(colmap_dir))
    out: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    for img in rec.images.values():
        if name_contains is not None and name_contains not in img.name:
            continue
        idx = parse_image_index(img.name)
        if idx is None:
            continue
        cfw = img.cam_from_world
        if callable(cfw):
            cfw = cfw()
        c2w = cfw.inverse()
        pos_c = np.asarray(c2w.translation, dtype=np.float64)
        # pycolmap Rotation3d.quat is xyzw
        quat_c = np.asarray(c2w.rotation.quat, dtype=np.float64)
        pos_gl, quat_gl = convert_pose_colmap_to_opengl(pos_c, quat_c)
        out[idx] = (pos_gl, np.asarray(quat_gl, dtype=np.float64))
    return out


def list_images(folder: Path) -> List[Path]:
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    files = [p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in exts]
    files.sort()
    return files


def stop_port(port: int) -> None:
    try:
        out = subprocess.check_output(
            ["bash", "-lc", f"ss -ltnp | awk -v p=':{port}' '$4 ~ p {{print}}'"],
            text=True,
        )
    except Exception:
        return
    m = re.search(r"pid=(\d+)", out)
    if not m:
        return
    pid = m.group(1)
    subprocess.call(["bash", "-lc", f"kill {pid} || true"])
    time.sleep(1)


def start_server(bin_path: Path, vocab: Path, settings: Path, prior: Path, port: int, log: Path) -> int:
    log.parent.mkdir(parents=True, exist_ok=True)
    log.write_text("")
    cmd = [
        str(bin_path),
        str(vocab),
        str(settings),
        "--prior-pack",
        str(prior),
        "--port",
        str(port),
        "--max-queue",
        "100",
    ]
    # Launch via bash so LD_LIBRARY_PATH from caller env applies.
    proc = subprocess.Popen(
        cmd,
        stdout=open(log, "w"),
        stderr=subprocess.STDOUT,
        cwd=str(bin_path.parent.parent.parent),
    )
    return proc.pid


def wait_ready(base: str, timeout_s: float = 90.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        try:
            r = requests.get(f"{base}/api/v1/status", timeout=2)
            if r.status_code == 200 and "timing" in r.json():
                return True
        except Exception:
            pass
        time.sleep(1)
    return False


def replay(
    server: str,
    frames_dir: Path,
    timestamps: List[float],
    intrinsics: Union[List[float], List[List[float]]],
    fps: float,
) -> None:
    images = list_images(frames_dir)
    if len(images) != len(timestamps):
        raise RuntimeError(f"Frames count {len(images)} != Frames.csv rows {len(timestamps)}")
    per_frame = bool(intrinsics) and isinstance(intrinsics[0], (list, tuple))
    if per_frame:
        if len(intrinsics) != len(images):
            raise RuntimeError(
                f"Intrinsics count {len(intrinsics)} != Frames count {len(images)}"
            )
    interval = 1.0 / fps if fps > 0 else 0.0
    url = server.rstrip("/") + "/api/v1/track"
    for idx, img_path in enumerate(images):
        t0 = time.time()
        data = img_path.read_bytes()
        frame_intrinsics = intrinsics[idx] if per_frame else intrinsics
        headers = {
            "Content-Type": "image/jpeg" if img_path.suffix.lower() in (".jpg", ".jpeg") else "image/png",
            "X-Client-ID": "eval_holdout_pose",
            "X-Frame-Id": str(idx),
            "X-Capture-Timestamp": f"{timestamps[idx]:.9f}",
            "X-Intrinsics": ",".join(str(x) for x in frame_intrinsics),
        }
        r = requests.post(url, data=data, headers=headers, timeout=60)
        if r.status_code != 200:
            print(f"[{img_path.name}] HTTP {r.status_code}: {r.text[:200]}", file=sys.stderr)
        if interval > 0:
            sleep = interval - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)


def fetch_trajectory(server: str) -> dict:
    r = requests.get(
        server.rstrip("/") + "/api/v1/trajectory?aligned=0&dense=1&all_maps=1",
        timeout=60,
    )
    r.raise_for_status()
    return r.json()


def nearest_index(ts: float, timestamps: List[float]) -> int:
    arr = np.asarray(timestamps, dtype=np.float64)
    return int(np.argmin(np.abs(arr - ts)))


def collect_aligned_maps(
    traj: dict,
    gt: Dict[int, Tuple[np.ndarray, np.ndarray]],
    timestamps: List[float],
):
    """Per-map Umeyama alignment. Yields dicts with aligned xyz, rotations, metrics inputs."""
    poses = traj.get("trajectory") or []
    by_map: Dict[int, List[dict]] = defaultdict(list)
    for p in poses:
        by_map[int(p.get("map_id", 0))].append(p)

    for map_id, samples in sorted(by_map.items()):
        samples = sorted(samples, key=lambda s: float(s["timestamp"]))
        est_pts = []
        gt_pts = []
        est_R = []
        gt_R = []
        matched_idx = []
        for s in samples:
            ts = float(s["timestamp"])
            idx = nearest_index(ts, timestamps)
            if idx not in gt:
                continue
            if abs(timestamps[idx] - ts) > 0.08:
                continue
            ori = s.get("orientation")
            if ori is None or len(ori) != 4:
                continue
            pos = s["position"]
            gt_pos, gt_quat = gt[idx]
            est_pts.append([float(pos[0]), float(pos[1]), float(pos[2])])
            gt_pts.append(gt_pos)
            est_R.append(rotmat_from_wxyz(ori))
            gt_R.append(rotmat_from_xyzw(gt_quat))
            matched_idx.append(idx)
        if len(est_pts) < 3:
            yield {
                "map_id": map_id,
                "matched": len(est_pts),
                "slam_samples": len(samples),
                "ok": False,
            }
            continue
        est = np.asarray(est_pts, dtype=np.float64)
        gtm = np.asarray(gt_pts, dtype=np.float64)
        R, t, scale = umeyama_alignment(est.T, gtm.T, with_scale=True)
        aligned = apply_sim3(est, R, t, scale)
        yield {
            "map_id": map_id,
            "matched": len(est_pts),
            "slam_samples": len(samples),
            "ok": True,
            "aligned": aligned,
            "gtm": gtm,
            "est_R": est_R,
            "gt_R": gt_R,
            "R_u": R,
            "scale": scale,
            "matched_idx": matched_idx,
        }


def evaluate(
    traj: dict,
    gt: Dict[int, Tuple[np.ndarray, np.ndarray]],
    timestamps: List[float],
    out_dir: Path,
    baseline_traj: Optional[dict] = None,
) -> dict:
    poses = traj.get("trajectory") or []
    segments = list(collect_aligned_maps(traj, gt, timestamps))
    n_maps_ok = sum(1 for s in segments if s.get("ok"))

    gt_idxs = sorted(gt.keys())
    gt_xyz = np.stack([gt[i][0] for i in gt_idxs], axis=0)
    h, u, v = topdown_axes(gt_xyz)

    summary = {
        "gt_poses": len(gt),
        "slam_poses": len(poses),
        "maps": {},
        "topdown_axes": {"height": h, "plot_u": u, "plot_v": v, "axis_names": "xyz"[h] + " up"},
        "correspondence": "timestamp_nearest_to_Frames.csv_index",
        "coord": (
            "GT OpenGL via convert_pose_colmap_to_opengl; "
            "SLAM API position + orientation (wxyz Three/OpenGL)"
        ),
        "alignment": (
            "Per atlas map: Umeyama Sim3 with_scale=True on positions. "
            "Rotation: (1) abs after R_u only; (2) abs after R_u + per-map fixed "
            "camera-axis R_cam (Kabsch on R_aligned.T@R_gt) — absorbs API vs GT "
            "camera-frame convention (~90 deg); (3) relative frame-to-frame "
            "(gauge-free)."
        ),
    }

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(
        gt_xyz[:, u],
        gt_xyz[:, v],
        color="#444444",
        linewidth=1.5,
        label=f"GT local refine (n={len(gt_idxs)})",
        zorder=1,
    )

    baseline_segments = []
    if baseline_traj is not None:
        baseline_segments = list(collect_aligned_maps(baseline_traj, gt, timestamps))
        first_bl = True
        n_bl = sum(len(s["aligned"]) for s in baseline_segments if s.get("ok"))
        for seg in baseline_segments:
            if not seg.get("ok"):
                continue
            aligned = seg["aligned"]
            ax.plot(
                aligned[:, u],
                aligned[:, v],
                color="#d62728",
                linewidth=1.4,
                alpha=0.85,
                linestyle="--",
                label=f"Baseline sparse (n={n_bl})" if first_bl else None,
                zorder=2,
            )
            first_bl = False
        summary["baseline_slam_poses"] = len(baseline_traj.get("trajectory") or [])
        summary["baseline_source"] = baseline_traj.get("source")

    slam_color = "#1f77b4"
    all_err = []
    all_rot_raw = []
    all_rot_camfix = []
    all_rot_rel = []
    first_map = True
    rel_series = []  # (image_index mid, rot_rel_deg, series_name)
    for seg in segments:
        map_id = seg["map_id"]
        if not seg.get("ok"):
            summary["maps"][str(map_id)] = {
                "matched": seg["matched"],
                "skipped": "need >=3 matches for Umeyama",
            }
            continue
        aligned = seg["aligned"]
        gtm = seg["gtm"]
        est_R = seg["est_R"]
        gt_R = seg["gt_R"]
        R = seg["R_u"]
        matched_idx = seg["matched_idx"]
        err = np.linalg.norm(aligned - gtm, axis=1)

        R_aligned = [R @ Ri for Ri in est_R]
        rot_raw = np.asarray(
            [rotation_angle_deg(gt_R[i], R_aligned[i]) for i in range(len(est_R))],
            dtype=np.float64,
        )
        R_cam = average_so3([Ra.T @ Rg for Ra, Rg in zip(R_aligned, gt_R)])
        rot_camfix = np.asarray(
            [rotation_angle_deg(gt_R[i], R_aligned[i] @ R_cam) for i in range(len(est_R))],
            dtype=np.float64,
        )
        rot_rel = []
        for i in range(len(est_R) - 1):
            d_est = est_R[i].T @ est_R[i + 1]
            d_gt = gt_R[i].T @ gt_R[i + 1]
            ang = rotation_angle_deg(d_gt, d_est)
            rot_rel.append(ang)
            rel_series.append(((matched_idx[i] + matched_idx[i + 1]) * 0.5, ang, "dense"))
        rot_rel = np.asarray(rot_rel, dtype=np.float64)

        all_err.append(err)
        all_rot_raw.append(rot_raw)
        all_rot_camfix.append(rot_camfix)
        if len(rot_rel):
            all_rot_rel.append(rot_rel)

        metrics = {
            "matched": int(len(err)),
            "slam_samples": seg["slam_samples"],
            "scale": float(seg["scale"]),
            "cam_axis_fix_deg_from_I": float(rotation_angle_deg(np.eye(3), R_cam)),
            "ate_rmse_m": float(np.sqrt(np.mean(err**2))),
            "ate_mean_m": float(np.mean(err)),
            "ate_median_m": float(np.median(err)),
            "ate_p95_m": float(np.percentile(err, 95)),
            "ate_max_m": float(np.max(err)),
            "rot_abs_raw": rot_stats(rot_raw),
            "rot_abs_camfix": rot_stats(rot_camfix),
            "rot_rel": rot_stats(rot_rel) if len(rot_rel) else {"skipped": "need >=2 matches"},
            "image_index_span": [int(min(matched_idx)), int(max(matched_idx))],
        }
        summary["maps"][str(map_id)] = metrics

        ax.plot(
            aligned[:, u],
            aligned[:, v],
            color=slam_color,
            linewidth=1.2,
            alpha=0.9,
            label=f"Dense all-maps (n={len(poses)})" if first_map else None,
            zorder=3,
        )
        first_map = False

    # Baseline relative rotation series for comparison plot
    baseline_rel_stats = None
    if baseline_segments:
        bl_rel = []
        for seg in baseline_segments:
            if not seg.get("ok"):
                continue
            est_R = seg["est_R"]
            gt_R = seg["gt_R"]
            matched_idx = seg["matched_idx"]
            for i in range(len(est_R) - 1):
                d_est = est_R[i].T @ est_R[i + 1]
                d_gt = gt_R[i].T @ gt_R[i + 1]
                ang = rotation_angle_deg(d_gt, d_est)
                bl_rel.append(ang)
                rel_series.append(((matched_idx[i] + matched_idx[i + 1]) * 0.5, ang, "baseline"))
        if bl_rel:
            baseline_rel_stats = rot_stats(np.asarray(bl_rel, dtype=np.float64))
            summary["baseline_rot_rel"] = baseline_rel_stats

    if all_err:
        cat = np.concatenate(all_err)
        cat_raw = np.concatenate(all_rot_raw)
        cat_fix = np.concatenate(all_rot_camfix)
        cat_rel = np.concatenate(all_rot_rel) if all_rot_rel else np.asarray([])
        summary["overall"] = {
            "matched": int(len(cat)),
            "ate_rmse_m": float(np.sqrt(np.mean(cat**2))),
            "ate_mean_m": float(np.mean(cat)),
            "ate_median_m": float(np.median(cat)),
            "ate_p95_m": float(np.percentile(cat, 95)),
            "ate_max_m": float(np.max(cat)),
            "rot_abs_raw": rot_stats(cat_raw),
            "rot_abs_camfix": rot_stats(cat_fix),
            "rot_rel": rot_stats(cat_rel) if len(cat_rel) else {},
            "num_maps": n_maps_ok,
        }

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(f"axis {'xyz'[u]} (m)")
    ax.set_ylabel(f"axis {'xyz'[v]} (m)")
    title_extra = ""
    if baseline_traj is not None:
        title_extra = f" + baseline n={summary.get('baseline_slam_poses', '?')}"
    ax.set_title(
        f"Holdout VO vs local refine (top-down; up={'xyz'[h]}, least motion)\n"
        f"dense maps={n_maps_ok} poses={len(poses)} GT={len(gt)}{title_extra}"
    )
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    out_dir.mkdir(parents=True, exist_ok=True)
    plot_path = out_dir / "holdout_pose_topdown.png"
    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    # Relative rotation vs image index (dense vs baseline)
    rot_plot_path = out_dir / "holdout_pose_rot_rel.png"
    if rel_series:
        fig2, ax2 = plt.subplots(figsize=(10, 4.5))
        for name, color, marker in (
            ("baseline", "#d62728", "."),
            ("dense", "#1f77b4", "."),
        ):
            xs = [p[0] for p in rel_series if p[2] == name]
            ys = [p[1] for p in rel_series if p[2] == name]
            if not xs:
                continue
            label = name
            if name == "dense" and all_rot_rel:
                st = rot_stats(np.concatenate(all_rot_rel))
                label = f"dense (med {st['rot_median_deg']:.1f}°)"
            if name == "baseline" and baseline_rel_stats:
                label = f"baseline (med {baseline_rel_stats['rot_median_deg']:.1f}°)"
            ax2.scatter(xs, ys, s=10, alpha=0.65, c=color, marker=marker, label=label, zorder=2)
        ax2.set_xlabel("image index (midpoint of pair)")
        ax2.set_ylabel("relative rotation error (deg)")
        ax2.set_title("Frame-to-frame relative rotation error vs local refine")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="best")
        fig2.tight_layout()
        fig2.savefig(rot_plot_path, dpi=160)
        plt.close(fig2)
        summary["rot_rel_plot"] = str(rot_plot_path)

    metrics_path = out_dir / "holdout_pose_metrics.json"
    summary["plot"] = str(plot_path)
    summary["metrics_file"] = str(metrics_path)
    metrics_path.write_text(json.dumps(summary, indent=2))
    return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-replay", action="store_true", help="Only evaluate existing trajectory JSON")
    ap.add_argument("--trajectory-json", type=Path, default=None)
    ap.add_argument(
        "--baseline-trajectory-json",
        type=Path,
        default=None,
        help="Optional sparse baseline trajectory to overlay (Umeyama per map)",
    )
    ap.add_argument("--port", type=int, default=18080)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=Path.home() / "build" / "holdout_pose_eval",
    )
    ap.add_argument(
        "--gt-colmap",
        type=Path,
        default=None,
        help="COLMAP model for GT (default: local refine for scan)",
    )
    ap.add_argument(
        "--gt-name-contains",
        type=str,
        default=None,
        help="Optional substring filter for image names (needed for global SfM)",
    )
    args = ap.parse_args()

    root = Path("/mnt/d/agent-workspaces/Rover-SLAM")
    job = Path(
        "/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/"
        "20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete"
    )
    scan = "2026-04-02_13-42-53"
    frames_dir = job / "datasets" / scan / "Frames"
    frames_csv = job / "datasets" / scan / "Frames.csv"
    colmap_dir = args.gt_colmap or (job / "refined" / "local" / scan / "sfm")
    bin_path = root / "Examples" / "Monocular" / "slam_server"
    vocab = root / "Vocabulary" / "voc_binary_tartan_8u_6.yml.gz"
    settings = root / "Examples" / "Monocular" / "iphoneSLAM.yaml"
    prior = root / "testdata" / "prior_pack_fixture"
    server = f"http://127.0.0.1:{args.port}"
    # cx/cy in landscape order; the YAML historically had these two transposed.
    fallback_intrinsics = [1335.984, 1335.984, 963.1064, 716.9096]
    intrinsics_csv = job / "datasets" / scan / "CameraIntrinsics.csv"
    if intrinsics_csv.is_file():
        intrinsics = load_camera_intrinsics_csv(intrinsics_csv)
        print(f"CameraIntrinsics.csv: {len(intrinsics)} per-frame rows")
    else:
        intrinsics = fallback_intrinsics
        print(
            f"WARNING: missing {intrinsics_csv}; using fallback intrinsics "
            f"{fallback_intrinsics}",
            file=sys.stderr,
        )

    timestamps = load_frames_csv(frames_csv)
    print(f"GT Frames.csv timestamps: {len(timestamps)}")
    gt = load_gt_opengl(colmap_dir, name_contains=args.gt_name_contains)
    print(
        f"GT COLMAP OpenGL poses (pos+quat): {len(gt)} "
        f"source={colmap_dir} filter={args.gt_name_contains!r}"
    )

    traj_path = args.trajectory_json or (args.out_dir / "trajectory_all_maps.json")
    if not args.skip_replay:
        if not bin_path.is_file():
            raise SystemExit(f"missing slam_server: {bin_path}")
        stop_port(args.port)
        log = Path.home() / "build" / "rover-slam-holdout-pose.log"
        print("starting slam_server...")
        start_server(bin_path, vocab, settings, prior, args.port, log)
        if not wait_ready(server):
            raise SystemExit(f"server not ready; see {log}")
        print(f"replaying {len(list_images(frames_dir))} frames @ {args.fps} FPS with capture timestamps...")
        replay(server, frames_dir, timestamps, intrinsics, args.fps)
        time.sleep(2)
        traj = fetch_trajectory(server)
        args.out_dir.mkdir(parents=True, exist_ok=True)
        traj_path.write_text(json.dumps(traj, indent=2))
        print(f"saved trajectory -> {traj_path} count={traj.get('count')}")
        # keep server up for inspection; caller may kill
    else:
        traj = json.loads(traj_path.read_text())

    baseline_traj = None
    baseline_path = args.baseline_trajectory_json
    if baseline_path is None:
        candidate = root / "out" / "holdout_pose_eval" / "trajectory_all_maps.json"
        if candidate.is_file() and candidate.resolve() != traj_path.resolve():
            baseline_path = candidate
    if baseline_path is not None and baseline_path.is_file():
        if baseline_path.resolve() == traj_path.resolve():
            print(f"baseline path equals primary trajectory; skipping overlay ({baseline_path})")
        else:
            baseline_traj = json.loads(baseline_path.read_text())
            print(
                f"baseline overlay: {baseline_path} "
                f"count={baseline_traj.get('count')} source={baseline_traj.get('source')}"
            )

    summary = evaluate(traj, gt, timestamps, args.out_dir, baseline_traj=baseline_traj)
    print(json.dumps(summary, indent=2))
    print(f"PLOT {summary.get('plot')}")
    if summary.get("rot_rel_plot"):
        print(f"ROT_PLOT {summary.get('rot_rel_plot')}")
    print(f"METRICS {summary.get('metrics_file')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
