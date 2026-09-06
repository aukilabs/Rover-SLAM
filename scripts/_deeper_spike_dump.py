#!/usr/bin/env python3
"""Deeper spike situation dump for 1024 holdout log."""
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

from _analyze_holdout_spikes import parse_spikes, classify

log = Path.home() / "build/rover-slam-holdout-timing-1024.log"
spikes = parse_spikes(log.read_text(errors="replace"))
rows = []
for s in spikes:
    c = classify(s)
    rows.append({**s, **c})

tracking = [r for r in rows if r["frame"] > 2]
print("n_spikes", len(rows), "tracking", len(tracking))

# Frame ranges / density
frames = [r["frame"] for r in tracking]
print("first_last", frames[0], frames[-1], "span", frames[-1] - frames[0])
# histogram by 50-frame bins
bins = Counter((f // 50) * 50 for f in frames)
print("spikes_by_50frame_bin", dict(sorted(bins.items())))

# Retry vs single call timing
for calls in (1, 2):
    sub = [r for r in tracking if r["lg_calls"] == calls]
    if not sub:
        continue
    print(
        f"lg_calls={calls}: n={len(sub)} "
        f"mean_total={sum(r['total'] for r in sub)/len(sub):.1f} "
        f"mean_lg_fwd={sum(r['lg_fwd'] for r in sub)/len(sub):.1f} "
        f"mean_mm_match={sum(r['mm_match'] for r in sub)/len(sub):.1f} "
        f"mean_inliers={sum(r['inliers'] for r in sub)/len(sub):.1f}"
    )

# Dominant: mm_match_heavy without retry
heavy = [r for r in tracking if "mm_match_heavy" in r["situation"]]
print("mm_match_heavy_n", len(heavy), f"({100*len(heavy)/len(tracking):.0f}% of tracking spikes)")
print(
    "  mean lg_fwd/mm_match/total",
    sum(r["lg_fwd"] for r in heavy) / len(heavy),
    sum(r["mm_match"] for r in heavy) / len(heavy),
    sum(r["total"] for r in heavy) / len(heavy),
)

# Compare extract share on all frames from status last window vs spikes
# Correlate inliers
low_inl = [r for r in tracking if r["inliers"] < 40]
ok_inl = [r for r in tracking if r["inliers"] >= 40]
print("low_inliers_spikes", len(low_inl), "ok_inliers_spikes", len(ok_inl))
if low_inl:
    print(
        "  low_inl mean total/lg_fwd/calls",
        sum(r["total"] for r in low_inl) / len(low_inl),
        sum(r["lg_fwd"] for r in low_inl) / len(low_inl),
        sum(r["lg_calls"] for r in low_inl) / len(low_inl),
    )

# Non-spike context: how often is total>100 vs status over_100ms
status = json.loads((Path.home() / "build/holdout_timing_1024.json").read_text())
t = status["timing"]
print("status over_100ms_count", t.get("over_100ms_count"), "of tracks", status.get("successful_tracks"))

# Typical non-spike from p50
print("typical (p50) total/extract/mm_match/lg_fwd",
      t["p50_ms"]["total"], t["p50_ms"]["extract"], t["p50_ms"]["pose_mm_match"], t["p50_ms"]["lg_fwd"])
print("tail (p95) total/extract/mm_match/lg_fwd",
      t["p95_ms"]["total"], t["p95_ms"]["extract"], t["p95_ms"]["pose_mm_match"], t["p95_ms"]["lg_fwd"])

# Fraction: among spikes, how much of pose is lg_fwd
print(
    "among tracking spikes, lg_fwd/pose mean ratio",
    sum(r["lg_fwd"] / max(r["pose"], 1e-6) for r in tracking) / len(tracking),
)
print(
    "among tracking spikes, extract/total mean ratio",
    sum(r["extract"] / max(r["total"], 1e-6) for r in tracking) / len(tracking),
)

# List all retry frames
retries = [r for r in tracking if r["lg_calls"] >= 2]
print("retry frames", [(r["frame"], round(r["total"]), round(r["lg_fwd"]), r["inliers"]) for r in retries])
refkf = [r for r in tracking if r["refkf"] > 5]
print("refkf spikes", [(r["frame"], round(r["total"]), round(r["refkf"]), round(r["mm"]), r["inliers"]) for r in refkf])
