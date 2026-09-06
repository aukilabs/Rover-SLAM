#!/usr/bin/env python3
"""Print compact holdout metrics + optional /status timing JSON."""
from __future__ import annotations

import json
import sys
from pathlib import Path


def main() -> None:
    label = sys.argv[1] if len(sys.argv) > 1 else "run"
    metrics = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    status = Path(sys.argv[3]) if len(sys.argv) > 3 else None
    if metrics is None or not metrics.is_file():
        raise SystemExit(f"missing metrics: {metrics}")
    m = json.loads(metrics.read_text())
    ov = m.get("overall") or m
    rr = ov.get("rot_rel") or {}
    print(f"=== {label} accuracy ===")
    print(
        "matched", ov.get("matched"),
        "maps", ov.get("num_maps"),
        "ate_rmse", ov.get("ate_rmse_m"),
        "ate_med", ov.get("ate_median_m"),
        "rot_rel_rmse", rr.get("rot_rmse_deg"),
        "rot_rel_med", rr.get("rot_median_deg"),
    )
    if status is None or not status.is_file():
        print("(no status timing)")
        return
    d = json.loads(status.read_text())
    t = d.get("timing") or {}
    p50 = t.get("p50_ms") or {}
    p95 = t.get("p95_ms") or {}
    print(
        f"=== {label} timing ===",
        "successful_tracks", d.get("successful_tracks"),
        "failed_tracks", d.get("failed_tracks"),
    )
    for k in (
        "total",
        "extract",
        "pose",
        "pose_mm_match",
        "lg_fwd",
        "lg_drain",
        "lg_prep",
        "lg_post",
    ):
        if k in p50 or k in p95:
            print(f"  {k}: p50={p50.get(k)} p95={p95.get(k)}")


if __name__ == "__main__":
    main()
