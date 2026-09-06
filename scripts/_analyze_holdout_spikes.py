#!/usr/bin/env python3
"""Parse [timing] spike lines + /status timing; summarize what dominates and when."""
from __future__ import annotations

import argparse
import json
import re
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


SPIKE_RE = re.compile(
    r"\[timing\] spike frame (?P<frame>\d+)\s+"
    r"total=(?P<total>[\d.]+)ms\s+"
    r"prepare=(?P<prepare>[\d.]+)\s+"
    r"extract=(?P<extract>[\d.]+)\s+"
    r"pose=(?P<pose>[\d.]+)\s+"
    r"mm=(?P<mm>[\d.]+)\s+"
    r"mm_match=(?P<mm_match>[\d.]+)\s+"
    r"mm_opt=(?P<mm_opt>[\d.]+)\s+"
    r"lg_prep=(?P<lg_prep>[\d.]+)\s+"
    r"lg_fwd=(?P<lg_fwd>[\d.]+)\s+"
    r"lg_drain=(?P<lg_drain>[\d.]+)\s+"
    r"lg_post=(?P<lg_post>[\d.]+)\s+"
    r"lg_calls=(?P<lg_calls>\d+)\s+"
    r"lg_kpts=(?P<lg_kpts0>\d+)/(?P<lg_kpts1>\d+)\s+"
    r"refkf=(?P<refkf>[\d.]+)\s+"
    r"reloc=(?P<reloc>[\d.]+)\s+"
    r"local_map=(?P<local_map>[\d.]+)\s+"
    r"new_kf=(?P<new_kf>[\d.]+)\s+"
    r"portal=(?P<portal>[\d.]+)\s+"
    r"prior=(?P<prior>[\d.]+)\s+"
    r"local_pts=(?P<local_pts>\d+)\s+"
    r"inliers=(?P<inliers>\d+)"
)

FLOAT_KEYS = [
    "total",
    "prepare",
    "extract",
    "pose",
    "mm",
    "mm_match",
    "mm_opt",
    "lg_prep",
    "lg_fwd",
    "lg_drain",
    "lg_post",
    "refkf",
    "reloc",
    "local_map",
    "new_kf",
    "portal",
    "prior",
]
INT_KEYS = ["frame", "lg_calls", "lg_kpts0", "lg_kpts1", "local_pts", "inliers"]


def parse_spikes(text: str) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for m in SPIKE_RE.finditer(text):
        row: Dict[str, Any] = {}
        for k in FLOAT_KEYS:
            row[k] = float(m.group(k))
        for k in INT_KEYS:
            row[k] = int(m.group(k))
        out.append(row)
    return out


def argmax_named(pairs: List[Tuple[str, float]]) -> str:
    return max(pairs, key=lambda x: x[1])[0]


def classify(row: Dict[str, Any]) -> Dict[str, str]:
    # Top-level VO buckets that make up almost all of total
    top = argmax_named(
        [
            ("extract", row["extract"]),
            ("pose", row["pose"]),
            ("local_map", row["local_map"]),
            ("prepare", row["prepare"]),
            ("prior", row["prior"]),
            ("portal", row["portal"]),
            ("new_kf", row["new_kf"]),
        ]
    )
    pose_sub = argmax_named(
        [
            ("mm", row["mm"]),
            ("refkf", row["refkf"]),
            ("reloc", row["reloc"]),
        ]
    )
    mm_sub = argmax_named(
        [
            ("mm_match", row["mm_match"]),
            ("mm_opt", row["mm_opt"]),
        ]
    )
    lg_sub = argmax_named(
        [
            ("lg_fwd", row["lg_fwd"]),
            ("lg_prep", row["lg_prep"]),
            ("lg_drain", row["lg_drain"]),
            ("lg_post", row["lg_post"]),
        ]
    )

    situation = []
    if row["frame"] <= 2:
        situation.append("startup_cold")
    if row["lg_calls"] >= 2:
        situation.append("mm_match_retry")
    if row["refkf"] > 5:
        situation.append("refkf_fallback")
    if row["reloc"] > 5:
        situation.append("reloc")
    if row["mm"] > 5 and row["mm_match"] > 0.8 * row["mm"]:
        situation.append("mm_match_heavy")
    if row["extract"] > 0.5 * row["total"] and row["pose"] < 1:
        situation.append("extract_only")
    if row["inliers"] > 0 and row["inliers"] < 40 and row["mm"] > 20:
        situation.append("low_inliers")
    if row["local_pts"] > 1500:
        situation.append("large_local_map")
    if not situation:
        situation.append("steady_track")

    return {
        "top_stage": top,
        "pose_subpath": pose_sub,
        "mm_subpath": mm_sub,
        "lg_subpath": lg_sub,
        "situation": "+".join(situation),
    }


def pct(vals: List[float], p: float) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    idx = p * (len(s) - 1)
    lo = int(idx)
    hi = min(lo + 1, len(s) - 1)
    frac = idx - lo
    return s[lo] * (1 - frac) + s[hi] * frac


def summarize(spikes: List[Dict[str, Any]], status: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    classified = []
    for row in spikes:
        c = classify(row)
        classified.append({**row, **c})

    by_top = Counter(r["top_stage"] for r in classified)
    by_pose = Counter(r["pose_subpath"] for r in classified)
    by_mm = Counter(r["mm_subpath"] for r in classified)
    by_lg = Counter(r["lg_subpath"] for r in classified)
    by_sit = Counter(r["situation"] for r in classified)
    by_calls = Counter(r["lg_calls"] for r in classified)
    by_kpts = Counter(f"{r['lg_kpts0']}/{r['lg_kpts1']}" for r in classified)

    # Exclude cold start for "what matters in tracking"
    tracking = [r for r in classified if r["frame"] > 2]
    track_by_top = Counter(r["top_stage"] for r in tracking)
    track_by_sit = Counter(r["situation"] for r in tracking)

    def mean(key: str, rows: List[Dict[str, Any]]) -> float:
        if not rows:
            return 0.0
        return sum(r[key] for r in rows) / len(rows)

    worst = max(classified, key=lambda r: r["total"]) if classified else None
    worst_pose = max(classified, key=lambda r: r["pose"]) if classified else None

    # Mean contribution of stages among tracking spikes
    stage_means = {
        k: round(mean(k, tracking), 2)
        for k in [
            "total",
            "extract",
            "pose",
            "mm",
            "mm_match",
            "lg_fwd",
            "lg_prep",
            "lg_drain",
            "refkf",
            "reloc",
            "local_map",
        ]
    }

    # Fraction of spike total explained by lg_fwd / extract / refkf
    def frac_of_total(key: str) -> float:
        if not tracking:
            return 0.0
        return sum(r[key] for r in tracking) / max(sum(r["total"] for r in tracking), 1e-9)

    status_summary = None
    if status:
        t = status.get("timing") or {}
        status_summary = {
            "successful_tracks": status.get("successful_tracks"),
            "failed_tracks": status.get("failed_tracks"),
            "over_100ms_count": t.get("over_100ms_count"),
            "p50_ms": t.get("p50_ms"),
            "p95_ms": t.get("p95_ms"),
            "max_pose_mm_ms": t.get("max_pose_mm_ms"),
            "max_pose_mm_match_ms": t.get("max_pose_mm_match_ms"),
            "max_lg_fwd_ms": t.get("max_lg_fwd_ms"),
            "max_pose_refkf_ms": t.get("max_pose_refkf_ms"),
            "last_frame": t.get("last_frame"),
        }

    # Cluster spikes by frame gaps (bursts)
    bursts = []
    if tracking:
        cur = [tracking[0]["frame"]]
        for r in tracking[1:]:
            if r["frame"] - cur[-1] <= 3:
                cur.append(r["frame"])
            else:
                bursts.append(cur)
                cur = [r["frame"]]
        bursts.append(cur)

    return {
        "spike_count": len(classified),
        "tracking_spike_count": len(tracking),
        "lg_kpts_hist": dict(by_kpts),
        "lg_calls_hist": {str(k): v for k, v in by_calls.items()},
        "all_spikes_top_stage": dict(by_top),
        "tracking_spikes_top_stage": dict(track_by_top),
        "pose_subpath": dict(by_pose),
        "mm_subpath": dict(by_mm),
        "lg_subpath": dict(by_lg),
        "situation_all": dict(by_sit),
        "situation_tracking": dict(track_by_sit),
        "tracking_stage_means_ms": stage_means,
        "tracking_frac_of_spike_total": {
            "extract": round(frac_of_total("extract"), 3),
            "pose": round(frac_of_total("pose"), 3),
            "mm_match": round(frac_of_total("mm_match"), 3),
            "lg_fwd": round(frac_of_total("lg_fwd"), 3),
            "refkf": round(frac_of_total("refkf"), 3),
            "reloc": round(frac_of_total("reloc"), 3),
            "local_map": round(frac_of_total("local_map"), 3),
        },
        "worst_total": worst,
        "worst_pose": worst_pose,
        "spike_bursts_frames": bursts[:30],
        "burst_count": len(bursts),
        "example_spikes": sorted(tracking, key=lambda r: -r["total"])[:8],
        "status": status_summary,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", type=Path, required=True)
    ap.add_argument("--status", type=Path, default=None)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    text = args.log.read_text(errors="replace")
    # Guard: if truncate path was active, say so
    truncated = "ROVER_LG_MM_TOPK=" in text
    spikes = parse_spikes(text)
    status = None
    if args.status and args.status.is_file():
        status = json.loads(args.status.read_text())

    summary = summarize(spikes, status)
    summary["log"] = str(args.log)
    summary["saw_ROVER_LG_MM_TOPK_log"] = truncated
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(summary, indent=2))

    print("=== spike analysis ===")
    print(f"spikes={summary['spike_count']} tracking_spikes={summary['tracking_spike_count']}")
    print(f"saw_ROVER_LG_MM_TOPK_log={truncated}")
    print(f"lg_kpts_hist={summary['lg_kpts_hist']}")
    print(f"lg_calls_hist={summary['lg_calls_hist']}")
    print(f"tracking_top_stage={summary['tracking_spikes_top_stage']}")
    print(f"pose_subpath={summary['pose_subpath']}")
    print(f"mm_subpath={summary['mm_subpath']}")
    print(f"lg_subpath={summary['lg_subpath']}")
    print(f"situation_tracking={summary['situation_tracking']}")
    print(f"tracking_stage_means_ms={json.dumps(summary['tracking_stage_means_ms'])}")
    print(f"frac_of_spike_total={json.dumps(summary['tracking_frac_of_spike_total'])}")
    if summary.get("status"):
        st = summary["status"]
        print(f"over_100ms_count={st.get('over_100ms_count')} tracks={st.get('successful_tracks')}/{st.get('failed_tracks')}")
        print(f"p50={json.dumps(st.get('p50_ms'))}")
        print(f"p95={json.dumps(st.get('p95_ms'))}")
    if summary["example_spikes"]:
        print("--- top spikes (tracking) ---")
        for r in summary["example_spikes"]:
            print(
                f"  f{r['frame']}: total={r['total']:.1f} extract={r['extract']:.1f} "
                f"pose={r['pose']:.1f} mm={r['mm']:.1f} mm_match={r['mm_match']:.1f} "
                f"lg_fwd={r['lg_fwd']:.1f} lg_calls={r['lg_calls']} "
                f"refkf={r['refkf']:.1f} reloc={r['reloc']:.1f} "
                f"inliers={r['inliers']} sit={r['situation']}"
            )
    print(f"WROTE {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
