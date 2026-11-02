#!/usr/bin/env python3
"""Compare VO (Umeyama) vs portal-domain trajectory quality for holdout.

Supports two domain modes:
  - final pin retcon: aligned=1 trajectory (current pin × full SLAM history)
  - live/causal: ?causal=1 history of domain poses as published online
"""
from __future__ import annotations

import argparse
import json
import shutil
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

import eval_holdout_pose_accuracy as ev


def collect_identity_maps(
    traj: dict,
    gt: Dict[int, Tuple[np.ndarray, np.ndarray]],
    timestamps: List[float],
    max_dt: float = 0.08,
    pose_source: Optional[str] = "overlay",
    map_ids: Optional[set] = None,
):
    """Match poses to GT with no Sim3 — tests portal domain frame directly.

    By default only pose_source=overlay samples are used (pinned maps). Raw SLAM
    poses on unpinned maps are not in domain and must not enter this metric.
    Accept pose_source='overlay_causal' for live history export.
    """
    poses = traj.get("trajectory") or []
    by_map: Dict[int, List[dict]] = defaultdict(list)
    for p in poses:
        if pose_source is not None and p.get("pose_source") != pose_source:
            continue
        map_id = int(p.get("map_id", 0))
        if map_ids is not None and map_id not in map_ids:
            continue
        by_map[map_id].append(p)

    for map_id, samples in sorted(by_map.items()):
        samples = sorted(samples, key=lambda s: float(s["timestamp"]))
        est_pts = []
        gt_pts = []
        est_R = []
        gt_R = []
        matched_idx = []
        for s in samples:
            ts = float(s["timestamp"])
            idx = ev.nearest_index(ts, timestamps)
            if idx not in gt:
                continue
            if abs(timestamps[idx] - ts) > max_dt:
                continue
            ori = s.get("orientation")
            if ori is None or len(ori) != 4:
                continue
            pos = s["position"]
            gt_pos, gt_quat = gt[idx]
            est_pts.append([float(pos[0]), float(pos[1]), float(pos[2])])
            gt_pts.append(gt_pos)
            est_R.append(ev.rotmat_from_wxyz(ori))
            gt_R.append(ev.rotmat_from_xyzw(gt_quat))
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
        yield {
            "map_id": map_id,
            "matched": len(est_pts),
            "slam_samples": len(samples),
            "ok": True,
            "aligned": est,
            "gtm": gtm,
            "est_R": est_R,
            "gt_R": gt_R,
            "R_u": np.eye(3),
            "scale": 1.0,
            "matched_idx": matched_idx,
        }


def summarize_segments(segments) -> dict:
    errs = []
    scales = []
    matched = 0
    maps_ok = 0
    for seg in segments:
        if not seg.get("ok"):
            continue
        maps_ok += 1
        matched += int(seg["matched"])
        scales.append(float(seg["scale"]))
        err = np.linalg.norm(seg["aligned"] - seg["gtm"], axis=1)
        errs.append(err)
    if not errs:
        return {
            "matched": matched,
            "num_maps": maps_ok,
            "ate_median_m": None,
            "ate_rmse_m": None,
            "ate_mean_m": None,
            "umeyama_scale_median": None,
        }
    all_err = np.concatenate(errs)
    return {
        "matched": matched,
        "num_maps": maps_ok,
        "ate_median_m": float(np.median(all_err)),
        "ate_rmse_m": float(np.sqrt(np.mean(all_err**2))),
        "ate_mean_m": float(np.mean(all_err)),
        "ate_p95_m": float(np.percentile(all_err, 95)),
        "umeyama_scale_median": float(np.median(scales)) if scales else None,
    }


def plot_paths(
    gt: Dict[int, Tuple[np.ndarray, np.ndarray]],
    vo_segments,
    domain_segments,
    out_path: Path,
    title: str,
    causal_segments=None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    gt_idxs = sorted(gt.keys())
    gt_xyz = np.stack([gt[i][0] for i in gt_idxs], axis=0)
    _h, u, v = ev.topdown_axes(gt_xyz)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(
        gt_xyz[:, u],
        gt_xyz[:, v],
        color="#444444",
        linewidth=1.5,
        label=f"GT local refine (n={len(gt_idxs)})",
        zorder=1,
    )

    first = True
    for seg in vo_segments:
        if not seg.get("ok"):
            continue
        a = seg["aligned"]
        ax.plot(
            a[:, u],
            a[:, v],
            color="#0b57d0",
            linewidth=1.6,
            alpha=0.9,
            label="VO Umeyama" if first else None,
        )
        first = False

    first = True
    for seg in domain_segments:
        if not seg.get("ok"):
            continue
        a = seg["aligned"]
        ax.plot(
            a[:, u],
            a[:, v],
            color="#0f7b3a",
            linewidth=1.6,
            alpha=0.85,
            label="Final pin × full SLAM (retcon)" if first else None,
        )
        first = False

    if causal_segments:
        first = True
        for seg in causal_segments:
            if not seg.get("ok"):
                continue
            a = seg["aligned"]
            ax.plot(
                a[:, u],
                a[:, v],
                color="#c45c26",
                linewidth=1.8,
                alpha=0.9,
                label="Live domain history (causal)" if first else None,
            )
            first = False

    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_paths_overlay(
    gt: Dict[int, Tuple[np.ndarray, np.ndarray]],
    vo_segments,
    atlas_segments,
    overlay_segments,
    out_path: Path,
    title: str,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    gt_idxs = sorted(gt.keys())
    gt_xyz = np.stack([gt[i][0] for i in gt_idxs], axis=0)
    _h, u, v = ev.topdown_axes(gt_xyz)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(
        gt_xyz[:, u],
        gt_xyz[:, v],
        color="#444444",
        linewidth=1.5,
        label=f"GT local refine (n={len(gt_idxs)})",
        zorder=1,
    )

    first = True
    for seg in vo_segments:
        if not seg.get("ok"):
            continue
        a = seg["aligned"]
        ax.plot(
            a[:, u],
            a[:, v],
            color="#0b57d0",
            linewidth=1.4,
            alpha=0.75,
            linestyle="--",
            label="VO Umeyama" if first else None,
            zorder=2,
        )
        first = False

    first = True
    for seg in overlay_segments:
        if not seg.get("ok"):
            continue
        a = seg["aligned"]
        ax.plot(
            a[:, u],
            a[:, v],
            color="#d62728",
            linewidth=1.4,
            alpha=0.85,
            linestyle=":",
            label="Overlay Sim3 (aligned=1)" if first else None,
            zorder=3,
        )
        first = False

    first = True
    for seg in atlas_segments:
        if not seg.get("ok"):
            continue
        a = seg["aligned"]
        ax.plot(
            a[:, u],
            a[:, v],
            color="#0f7b3a",
            linewidth=1.8,
            alpha=0.9,
            label="Atlas domain (slam, identity)" if first else None,
            zorder=4,
        )
        first = False

    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_domain_error_series(
    domain_segments,
    out_path: Path,
    causal_segments=None,
) -> Optional[dict]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def series_from(segs):
        xs, ys = [], []
        for seg in segs:
            if not seg.get("ok"):
                continue
            err = np.linalg.norm(seg["aligned"] - seg["gtm"], axis=1)
            for idx, e in zip(seg["matched_idx"], err):
                xs.append(idx)
                ys.append(e)
        if not xs:
            return None, None
        xs_a = np.asarray(xs)
        ys_a = np.asarray(ys)
        order = np.argsort(xs_a)
        return xs_a[order], ys_a[order]

    xs_f, ys_f = series_from(domain_segments)
    xs_c, ys_c = series_from(causal_segments or [])
    if xs_f is None and xs_c is None:
        return None

    fig, ax = plt.subplots(figsize=(10, 4.5))
    if xs_f is not None:
        ax.plot(
            xs_f,
            ys_f * 100.0,
            ".",
            color="#0f7b3a",
            markersize=3,
            label="final-pin domain ATE",
        )
    if xs_c is not None:
        ax.plot(
            xs_c,
            ys_c * 100.0,
            ".",
            color="#c45c26",
            markersize=3,
            label="live/causal domain ATE",
        )
    ax.set_xlabel("image index")
    ax.set_ylabel("position error vs local GT (cm)")
    ax.set_title("Domain pose error: live history vs final pin (identity)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)
    out = {
        "note": "series is translation error meters plotted as cm",
    }
    if ys_f is not None:
        out["final_median_m"] = float(np.median(ys_f))
        out["rot_median_deg"] = float(np.median(ys_f))
    if ys_c is not None:
        out["causal_median_m"] = float(np.median(ys_c))
    return out


def extract_overlay_pin(status: dict) -> dict:
    ov = status.get("overlay") or {}
    pinned = [m for m in ov.get("maps", []) if m.get("pinned")]
    return {
        "active_map_id": ov.get("active_map_id"),
        "active_map_pinned": ov.get("active_map_pinned"),
        "pinned_map_ids": ov.get("pinned_map_ids"),
        "prior_soft_edges_accepted": ov.get("prior_soft_edges_accepted", 0),
        "pinned_maps": pinned,
        "causal_pose_history_count": status.get("causal_pose_history_count"),
    }


def extract_atlas_domain_status(status: dict) -> dict:
    al = status.get("alignment") or {}
    pba = status.get("portal_ba_edges") or {}
    store = status.get("portal_ba_store") or {}
    return {
        "domainized": status.get("domainized"),
        "portal_ba_edges": pba,
        "portal_ba_store": store,
        "alignment_status": al.get("status"),
        "alignment_portal_observations": al.get("portal_observations", 0),
        "coordinate_frame": status.get("coordinate_frame"),
    }


METERS_SCALE_THRESHOLD_M = 0.5


def ate_not_meters_scale(ate_m: Optional[float]) -> bool:
    """True when ATE is unknown or below the meters-scale failure threshold."""
    return ate_m is None or ate_m < METERS_SCALE_THRESHOLD_M


def build_evidence(status: dict, compare: dict, atlas_mode: bool) -> dict:
    atlas = extract_atlas_domain_status(status)
    portal_obs = int(atlas.get("alignment_portal_observations") or 0)
    pba_map = int((atlas.get("portal_ba_edges") or {}).get("map") or 0)
    store_n = int((atlas.get("portal_ba_store") or {}).get("observations") or 0)
    portal_obs_effective = max(portal_obs, pba_map, store_n)

    if atlas_mode:
        pooled = compare.get("atlas_domain_direct") or {}
        map0 = compare.get("atlas_domain_direct_map0") or {}
        active = compare.get("atlas_domain_direct_active_map") or {}
    else:
        pooled = compare.get("portal_domain_direct") or {}
        map0 = compare.get("portal_domain_direct_map0") or {}
        active = {}

    domain_pooled_ate = pooled.get("ate_median_m")
    domain_map0_ate = map0.get("ate_median_m")
    domain_active_ate = active.get("ate_median_m")

    per_map_ates = {
        "pooled_domainized_maps": domain_pooled_ate,
        "map0": domain_map0_ate,
        "active_map": domain_active_ate,
    }
    # Fail if pooled OR any reported per-map segment is meters-scale.
    domain_ate_not_meters_scale = all(
        ate_not_meters_scale(v) for v in per_map_ates.values() if v is not None
    )
    if domain_pooled_ate is None and domain_map0_ate is None and domain_active_ate is None:
        domain_ate_not_meters_scale = True

    overlay_same_run = compare.get("overlay_sim3_direct") or {}
    overlay_same_run_matched = int(overlay_same_run.get("matched") or 0)
    s9_overlay = compare.get("s9_overlay_domain_baseline") or {}
    s9_overlay_ate = s9_overlay.get("ate_median_m")
    s9_overlay_causal = compare.get("s9_overlay_causal_baseline") or {}

    domain_active_not_worse_than_s9_overlay = None
    if atlas_mode and domain_active_ate is not None and s9_overlay_ate is not None:
        # Active-map domain should beat historical S9 overlay (~3.6 cm); allow 2× margin.
        domain_active_not_worse_than_s9_overlay = domain_active_ate <= max(
            s9_overlay_ate * 2.0, s9_overlay_ate + 0.05
        )

    checks = {
        "prior_pack_loaded": (status.get("prior_pack") or {}).get("loaded") is True,
        "domainized": atlas.get("domainized") is True,
        "portal_observations_ge_1": portal_obs_effective >= 1,
        "portal_ba_edges_gt_0": pba_map > 0,
        "domain_ate_not_meters_scale": domain_ate_not_meters_scale,
    }
    if atlas_mode:
        checks["domain_active_map_not_worse_than_s9_overlay"] = (
            domain_active_not_worse_than_s9_overlay
        )
    if not atlas_mode:
        checks.pop("portal_ba_edges_gt_0", None)
        checks["domainized"] = checks["domainized"] or bool(
            (status.get("overlay") or {}).get("active_map_pinned")
        )

    overlay_baseline = {
        "same_run_overlay_sim3": overlay_same_run,
        "same_run_overlay_sim3_unavailable_reason": (
            "Post-domainize aligned=1 trajectory exports pose_source=slam only; "
            "same-run overlay Sim3 poses are absent (matched=0). "
            "Compare against historical S9 overlay baseline from --baseline-metrics."
            if overlay_same_run_matched == 0
            else None
        ),
        "s9_overlay_domain_baseline": s9_overlay or None,
        "s9_overlay_causal_baseline": s9_overlay_causal or None,
    }

    return {
        "checks": checks,
        "atlas": atlas,
        "portal_observations_effective": portal_obs_effective,
        "domain_ate_median_m": domain_pooled_ate,
        "domain_ate_per_map_median_m": per_map_ates,
        "meters_scale_threshold_m": METERS_SCALE_THRESHOLD_M,
        "vo_umeyama_ate_median_m": (compare.get("vo_umeyama") or {}).get("ate_median_m"),
        "overlay_sim3_ate_median_m": overlay_same_run.get("ate_median_m"),
        "overlay_baseline": overlay_baseline,
        "s9_class_baseline_ate_median_m": 0.17,
        "accuracy_pass": False,
        "note": (
            "accuracy_pass is always false in harness-only slice E; "
            "§5 accuracy is a separate verify gate (not claimed here)."
        ),
    }


def map_summary(segments, map_id: int) -> Optional[dict]:
    for seg in segments:
        if seg.get("ok") and int(seg.get("map_id", -1)) == map_id:
            err = np.linalg.norm(seg["aligned"] - seg["gtm"], axis=1)
            return {
                "matched": int(seg["matched"]),
                "ate_median_m": float(np.median(err)),
                "ate_rmse_m": float(np.sqrt(np.mean(err**2))),
                "ate_mean_m": float(np.mean(err)),
                "ate_p95_m": float(np.percentile(err, 95)),
            }
    return None


def map0_summary(segments) -> Optional[dict]:
    return map_summary(segments, 0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw-traj", type=Path, required=True)
    ap.add_argument("--domain-traj", type=Path, required=True)
    ap.add_argument(
        "--causal-traj",
        type=Path,
        default=None,
        help="Optional live/causal domain pose history (?causal=1)",
    )
    ap.add_argument("--status", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--baseline-metrics", type=Path, default=None)
    ap.add_argument("--baseline-topdown", type=Path, default=None)
    ap.add_argument("--baseline-rot", type=Path, default=None)
    ap.add_argument("--gt-colmap", type=Path, default=None)
    ap.add_argument("--gt-name-contains", type=str, default="2026-04-02_13-42-53")
    ap.add_argument(
        "--overlay-traj",
        type=Path,
        default=None,
        help="aligned=1 overlay Sim3 trajectory (overlay pose_source) for baseline compare",
    )
    ap.add_argument(
        "--atlas-domain",
        action="store_true",
        help="Atlas domainized: domain poses use pose_source=slam on raw traj (identity vs GT)",
    )
    args = ap.parse_args()

    job = Path(
        "/mnt/d/agent-workspaces/reconstruction-server/_local/job-artifacts/"
        "20260904-214520-job_50ca9b6a-dcc2-4bf6-a278-439b1ffea8ff-complete"
    )
    scan = "2026-04-02_13-42-53"
    frames_csv = job / "datasets" / scan / "Frames.csv"
    colmap_dir = args.gt_colmap or (job / "refined" / "global" / "refined_sfm_combined")
    name_contains = args.gt_name_contains or None

    timestamps = ev.load_frames_csv(frames_csv)
    gt = ev.load_gt_opengl(colmap_dir, name_contains=name_contains)
    print(f"GT source={colmap_dir} name_contains={name_contains!r} poses={len(gt)}")
    if len(gt) < 10:
        raise SystemExit(f"too few GT poses from {colmap_dir} filter={name_contains!r}")
    raw = json.loads(args.raw_traj.read_text())
    domain = json.loads(args.domain_traj.read_text())
    status = json.loads(args.status.read_text())
    causal = None
    if args.causal_traj and args.causal_traj.is_file():
        causal = json.loads(args.causal_traj.read_text())

    out = args.out_dir
    out.mkdir(parents=True, exist_ok=True)

    vo_summary = ev.evaluate(raw, gt, timestamps, out, baseline_traj=None)
    vo_top = out / "holdout_pose_topdown_vo.png"
    vo_rot = out / "holdout_pose_rot_rel_vo.png"
    if (out / "holdout_pose_topdown.png").is_file():
        shutil.copyfile(out / "holdout_pose_topdown.png", vo_top)
    if (out / "holdout_pose_rot_rel.png").is_file():
        shutil.copyfile(out / "holdout_pose_rot_rel.png", vo_rot)

    vo_segs = list(ev.collect_aligned_maps(raw, gt, timestamps))
    domain_pose_source = "slam" if args.atlas_domain else "overlay"
    atlas_domainized_map_ids = None
    if args.atlas_domain:
        atlas_domainized_map_ids = {
            int(m["map_id"])
            for m in (status.get("overlay") or {}).get("maps", [])
            if m.get("atlas_domainized")
        }
        if not atlas_domainized_map_ids:
            active_id = (status.get("overlay") or {}).get("active_map_id")
            if status.get("domainized") and active_id is not None:
                atlas_domainized_map_ids = {int(active_id)}
    domain_id_segs = list(
        collect_identity_maps(
            domain,
            gt,
            timestamps,
            pose_source=domain_pose_source,
            map_ids=atlas_domainized_map_ids,
        )
    )
    domain_u_segs = list(ev.collect_aligned_maps(domain, gt, timestamps))
    causal_id_segs = []
    if causal is not None:
        causal_id_segs = list(
            collect_identity_maps(causal, gt, timestamps, pose_source="overlay_causal")
        )

    overlay_sim3_segs = []
    overlay_sim3_direct = None
    if args.overlay_traj and args.overlay_traj.is_file():
        overlay_traj = json.loads(args.overlay_traj.read_text())
        overlay_sim3_segs = list(
            collect_identity_maps(overlay_traj, gt, timestamps, pose_source="overlay")
        )
        overlay_sim3_direct = summarize_segments(overlay_sim3_segs)

    vo_overall = summarize_segments(vo_segs)
    domain_direct = summarize_segments(domain_id_segs)
    domain_umeyama = summarize_segments(domain_u_segs)
    causal_direct = summarize_segments(causal_id_segs) if causal_id_segs else None
    pin = extract_overlay_pin(status)
    atlas_status = extract_atlas_domain_status(status) if args.atlas_domain else None

    plot_title = (
        "Holdout: VO Umeyama + Atlas domain (slam, identity) + overlay Sim3 baseline"
        if args.atlas_domain
        else "Holdout: VO + final-pin domain + live/causal domain (identity)"
    )
    if args.atlas_domain and overlay_sim3_segs:
        plot_paths_overlay(
            gt,
            vo_segs,
            domain_id_segs,
            overlay_sim3_segs,
            out / "holdout_pose_topdown.png",
            plot_title,
        )
    else:
        plot_paths(
            gt,
            vo_segs,
            domain_id_segs,
            out / "holdout_pose_topdown.png",
            plot_title,
            causal_segments=causal_id_segs or None,
        )
    plot_domain_error_series(
        domain_id_segs,
        out / "holdout_pose_rot_rel.png",
        causal_segments=causal_id_segs or None,
    )

    domain_dst = out / "trajectory_all_maps.json"
    if args.domain_traj.resolve() != domain_dst.resolve():
        shutil.copyfile(args.domain_traj, domain_dst)
    raw_dst = out / "trajectory_raw.json"
    if args.raw_traj.resolve() != raw_dst.resolve():
        shutil.copyfile(args.raw_traj, raw_dst)
    if args.causal_traj and args.causal_traj.is_file():
        dst = out / "trajectory_causal.json"
        if args.causal_traj.resolve() != dst.resolve():
            shutil.copyfile(args.causal_traj, dst)

    map0_direct = map0_summary(domain_id_segs)
    map0_causal = map0_summary(causal_id_segs) if causal_id_segs else None
    active_map_id = (status.get("overlay") or {}).get("active_map_id")
    active_map_direct = (
        map_summary(domain_id_segs, int(active_map_id))
        if active_map_id is not None
        else None
    )

    # Primary "overall" for the A/B page: prefer causal (device-view) when present.
    primary = causal_direct if causal_direct and causal_direct.get("ate_median_m") is not None else domain_direct
    if args.atlas_domain:
        primary = domain_direct

    mode = "atlas_domain_vs_vo" if args.atlas_domain else "portal_domain_vs_vo"
    metrics = {
        "mode": mode,
        "gt_source": str(colmap_dir),
        "gt_name_contains": name_contains,
        "slam_poses": len(raw.get("trajectory") or []),
        "domain_slam_poses": len(domain.get("trajectory") or []),
        "causal_poses": len((causal or {}).get("trajectory") or []) if causal else 0,
        "gt_poses": len(gt),
        "overall": {
            "matched": primary["matched"],
            "num_maps": primary["num_maps"],
            "ate_median_m": primary["ate_median_m"],
            "ate_rmse_m": primary["ate_rmse_m"],
            "rot_rel": {"rot_median_deg": primary.get("ate_median_m") or 0.0},
            "primary_metric": "causal" if causal_direct else "final_pin",
        },
        "vo_umeyama": vo_overall,
        "portal_domain_direct": domain_direct,
        "portal_domain_direct_map0": map0_direct,
        "portal_domain_umeyama": domain_umeyama,
        "portal_domain_causal": causal_direct,
        "portal_domain_causal_map0": map0_causal,
        "overlay_pin": pin,
        "atlas_domain": atlas_status,
        "atlas_domain_direct": domain_direct if args.atlas_domain else None,
        "atlas_domain_direct_map0": map0_direct if args.atlas_domain else None,
        "atlas_domain_direct_active_map": active_map_direct if args.atlas_domain else None,
        "overlay_sim3_direct": overlay_sim3_direct,
        "hypothesis": (
            "GT is global refined SfM. Atlas domain = raw slam poses after domainize (identity). "
            "overlay Sim3 = aligned=1 overlay pose_source (legacy pin retcon)."
            if args.atlas_domain
            else (
                "GT is global refined SfM. final_pin = current Sim3 × full SLAM traj (retcon). "
                "causal = live domain poses as a device would have seen (post-pin only)."
            )
        ),
        "vo_summary_file": str(out / "holdout_pose_metrics_vo.json"),
        "note": (
            "atlas_domain_direct is primary for fuse-prior-into-ba slice E."
            if args.atlas_domain
            else (
                "portal_domain_causal is the device-relevant metric; "
                "portal_domain_direct is final-pin retcon for comparison."
            )
        ),
    }
    (out / "holdout_pose_metrics_vo.json").write_text(json.dumps(vo_summary, indent=2))
    (out / "holdout_pose_metrics.json").write_text(json.dumps(metrics, indent=2))

    compare = {
        "baseline_label": (
            "VO Umeyama (no portal gauge)"
            if args.atlas_domain
            else "VO sync/noretry (Umeyama vs GLOBAL GT)"
        ),
        "new_label": (
            "Atlas domain (slam pose_source, identity vs GLOBAL GT)"
            if args.atlas_domain
            else "Domain: live/causal vs final-pin retcon (identity vs GLOBAL GT)"
        ),
        "gt_source": str(colmap_dir),
        "gt_name_contains": name_contains,
        "vo_umeyama": vo_overall,
        "portal_domain_direct": domain_direct,
        "portal_domain_direct_map0": map0_direct,
        "portal_domain_umeyama": domain_umeyama,
        "portal_domain_causal": causal_direct,
        "portal_domain_causal_map0": map0_causal,
        "atlas_domain_direct": domain_direct if args.atlas_domain else None,
        "atlas_domain_direct_map0": map0_direct if args.atlas_domain else None,
        "atlas_domain_direct_active_map": active_map_direct if args.atlas_domain else None,
        "overlay_sim3_direct": overlay_sim3_direct,
        "overlay_pin": pin,
        "atlas_domain": atlas_status,
    }
    if args.baseline_metrics and args.baseline_metrics.is_file():
        base = json.loads(args.baseline_metrics.read_text())
        compare["baseline_overall"] = base.get("overall")
        compare["baseline_slam_poses"] = base.get("slam_poses")
        compare["s9_overlay_domain_baseline"] = base.get("portal_domain_direct")
        compare["s9_overlay_causal_baseline"] = base.get("portal_domain_causal")
    evidence = build_evidence(status, compare, atlas_mode=args.atlas_domain)
    (out / "evidence.json").write_text(json.dumps(evidence, indent=2))
    (out / "compare_summary.json").write_text(json.dumps(compare, indent=2))

    print(json.dumps(compare, indent=2))
    print(f"PLOT {out / 'holdout_pose_topdown.png'}")
    print(f"METRICS {out / 'holdout_pose_metrics.json'}")
    print(f"EVIDENCE {out / 'evidence.json'}")
    return evidence_exit_code(evidence, atlas_mode=args.atlas_domain)


def evidence_exit_code(evidence: dict, atlas_mode: bool) -> int:
    """Non-zero when harness accuracy gates fail (plumbing vs ATE vs S9)."""
    checks = evidence.get("checks") or {}
    exit_code = 0
    if not checks.get("prior_pack_loaded"):
        exit_code = max(exit_code, 2)
    if not checks.get("domainized"):
        exit_code = max(exit_code, 2)
    if checks.get("portal_observations_ge_1") is False:
        exit_code = max(exit_code, 2)
    if atlas_mode and checks.get("portal_ba_edges_gt_0") is False:
        exit_code = max(exit_code, 2)
    if checks.get("domain_ate_not_meters_scale") is False:
        exit_code = max(exit_code, 3)
    if atlas_mode and checks.get("domain_active_map_not_worse_than_s9_overlay") is False:
        exit_code = max(exit_code, 4)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
