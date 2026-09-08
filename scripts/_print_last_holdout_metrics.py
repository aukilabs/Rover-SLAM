import json
from pathlib import Path

m = json.loads((Path.home() / "build/holdout_pose_eval/holdout_pose_metrics.json").read_text())
print("pose keys", sorted(m.keys()))
print(json.dumps(m, indent=2)[:2500])
t = json.loads((Path.home() / "build/holdout_timing_status.json").read_text())
tim = t.get("timing") or {}
print("timing p50", json.dumps(tim.get("p50_ms"), indent=2)[:1500])
print("timing p95", json.dumps(tim.get("p95_ms"), indent=2)[:1500])
