#!/usr/bin/env python3
"""Aggregate repeated holdout A/B metrics (mean ± std)."""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def mean_std(xs: List[float]) -> Tuple[float, float]:
    if not xs:
        return float("nan"), float("nan")
    m = sum(xs) / len(xs)
    if len(xs) == 1:
        return m, 0.0
    var = sum((x - m) ** 2 for x in xs) / (len(xs) - 1)
    return m, math.sqrt(var)


def load_metrics(path: Path) -> Dict:
    m = json.loads(path.read_text())
    return m.get("overall") or m


def load_timing(path: Path) -> Dict:
    if not path.is_file():
        return {}
    d = json.loads(path.read_text())
    t = d.get("timing") or {}
    return {
        "successful_tracks": d.get("successful_tracks"),
        "failed_tracks": d.get("failed_tracks"),
        "p50": t.get("p50_ms") or {},
        "p95": t.get("p95_ms") or {},
    }


def collect(root: Path, n: int, kind: str) -> Dict[str, List[float]]:
    keys = {
        "matched": [],
        "num_maps": [],
        "ate_rmse": [],
        "ate_med": [],
        "rot_rel_rmse": [],
        "rot_rel_med": [],
        "tracks": [],
        "total_p50": [],
        "total_p95": [],
        "extract_p50": [],
        "mm_match_p50": [],
        "mm_match_p95": [],
        "lg_fwd_p50": [],
        "lg_fwd_p95": [],
        "lg_drain_p50": [],
    }
    for i in range(1, n + 1):
        d = root / f"r{i}_{kind}"
        ov = load_metrics(d / "holdout_pose_metrics.json")
        rr = ov.get("rot_rel") or {}
        tm = load_timing(d / "status.json")
        p50 = tm.get("p50") or {}
        p95 = tm.get("p95") or {}
        keys["matched"].append(float(ov.get("matched") or 0))
        keys["num_maps"].append(float(ov.get("num_maps") or 0))
        keys["ate_rmse"].append(float(ov.get("ate_rmse_m") or 0))
        keys["ate_med"].append(float(ov.get("ate_median_m") or 0))
        keys["rot_rel_rmse"].append(float(rr.get("rot_rmse_deg") or 0))
        keys["rot_rel_med"].append(float(rr.get("rot_median_deg") or 0))
        keys["tracks"].append(float(tm.get("successful_tracks") or 0))
        keys["total_p50"].append(float(p50.get("total") or 0))
        keys["total_p95"].append(float(p95.get("total") or 0))
        keys["extract_p50"].append(float(p50.get("extract") or 0))
        keys["mm_match_p50"].append(float(p50.get("pose_mm_match") or 0))
        keys["mm_match_p95"].append(float(p95.get("pose_mm_match") or 0))
        keys["lg_fwd_p50"].append(float(p50.get("lg_fwd") or 0))
        keys["lg_fwd_p95"].append(float(p95.get("lg_fwd") or 0))
        keys["lg_drain_p50"].append(float(p50.get("lg_drain") or 0))
    return keys


def fmt(xs: List[float], digits: int = 3) -> str:
    m, s = mean_std(xs)
    raw = ", ".join(f"{x:.{digits}f}" for x in xs)
    return f"{m:.{digits}f} ± {s:.{digits}f}  [{raw}]"


def main() -> None:
    root = Path(sys.argv[1])
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    for kind in ("baseline", "new"):
        print(f"=== {kind} (n={n}) ===")
        d = collect(root, n, kind)
        print("  matched       ", fmt(d["matched"], 1))
        print("  maps          ", fmt(d["num_maps"], 1))
        print("  tracks        ", fmt(d["tracks"], 1))
        print("  ate_rmse_m    ", fmt(d["ate_rmse"], 4))
        print("  ate_med_m     ", fmt(d["ate_med"], 4))
        print("  rot_rel_rmse  ", fmt(d["rot_rel_rmse"], 3))
        print("  rot_rel_med   ", fmt(d["rot_rel_med"], 3))
        print("  total p50/p95 ", fmt(d["total_p50"], 1), "/", fmt(d["total_p95"], 1))
        print("  extract p50   ", fmt(d["extract_p50"], 1))
        print("  mm_match p50  ", fmt(d["mm_match_p50"], 1))
        print("  mm_match p95  ", fmt(d["mm_match_p95"], 1))
        print("  lg_fwd p50    ", fmt(d["lg_fwd_p50"], 1))
        print("  lg_fwd p95    ", fmt(d["lg_fwd_p95"], 1))
        print("  lg_drain p50  ", fmt(d["lg_drain_p50"], 3))
        print()

    b = collect(root, n, "baseline")
    nw = collect(root, n, "new")
    print("=== delta (new - baseline), mean ===")
    for k, dig in (
        ("ate_rmse", 4),
        ("ate_med", 4),
        ("rot_rel_rmse", 3),
        ("rot_rel_med", 3),
        ("matched", 1),
        ("total_p50", 1),
        ("total_p95", 1),
        ("extract_p50", 1),
        ("mm_match_p95", 1),
        ("lg_drain_p50", 3),
    ):
        bm, _ = mean_std(b[k])
        nm, _ = mean_std(nw[k])
        print(f"  {k}: {nm - bm:+.{dig}f}")


if __name__ == "__main__":
    main()
