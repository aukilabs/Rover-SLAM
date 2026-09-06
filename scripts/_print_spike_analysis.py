#!/usr/bin/env python3
import json
from pathlib import Path

d = json.loads((Path.home() / "build/holdout_spike_analysis_1024.json").read_text())
keys = [
    "spike_count",
    "tracking_spike_count",
    "saw_ROVER_LG_MM_TOPK_log",
    "lg_kpts_hist",
    "lg_calls_hist",
    "tracking_spikes_top_stage",
    "pose_subpath",
    "mm_subpath",
    "lg_subpath",
    "situation_tracking",
    "tracking_stage_means_ms",
    "tracking_frac_of_spike_total",
    "burst_count",
]
for k in keys:
    v = d.get(k)
    print(f"{k}: {json.dumps(v, indent=2) if isinstance(v, dict) else v}")
st = d.get("status") or {}
print("over_100ms", st.get("over_100ms_count"))
print("p50", json.dumps(st.get("p50_ms"), indent=2))
print("p95", json.dumps(st.get("p95_ms"), indent=2))
print(
    "max_mm",
    st.get("max_pose_mm_ms"),
    "max_mm_match",
    st.get("max_pose_mm_match_ms"),
    "max_lg_fwd",
    st.get("max_lg_fwd_ms"),
    "max_refkf",
    st.get("max_pose_refkf_ms"),
)
print("--- examples ---")
for r in d.get("example_spikes") or []:
    print(
        f"f{r['frame']}: total={r['total']:.0f} extract={r['extract']:.0f} "
        f"pose={r['pose']:.0f} mm={r['mm']:.0f} match={r['mm_match']:.0f} "
        f"lg_fwd={r['lg_fwd']:.0f} calls={r['lg_calls']} "
        f"kpts={r['lg_kpts0']}/{r['lg_kpts1']} refkf={r['refkf']:.0f} "
        f"reloc={r['reloc']:.0f} inliers={r['inliers']} pts={r['local_pts']} "
        f"sit={r['situation']}"
    )
print("bursts sample", (d.get("spike_bursts_frames") or [])[:15])
