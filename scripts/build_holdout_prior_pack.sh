#!/usr/bin/env bash
# Slice C: build + validate real prior pack from refined global recon export.
# Artifact: /mnt/d/agent-workspaces/Rover-SLAM/out/prior_pack_map_refined
# (1655 map images, 2 portals MQAUVO7PXZ1 + HBORRXLUOD6; holdout 13-42-53 excluded)
set -euo pipefail

ROOT=/mnt/d/agent-workspaces/Rover-SLAM
RECON="${RECON:-$ROOT/out/recon_verify_slice_b}"
PACK="${PACK:-$ROOT/out/prior_pack_map_refined}"

cd "$ROOT"

if [[ ! -f "$RECON/images.json" ]]; then
  echo "Recon missing at $RECON — run export_colmap_h5_to_recon.py first" >&2
  exit 1
fi

# Image symlinks from WSL export are not readable from Windows Python; build in WSL.
python3 scripts/convert_to_prior_pack.py --input "$RECON" --output "$PACK"
python3 scripts/validate_prior_pack.py "$PACK"

python3 - <<PY
import json
from pathlib import Path
import numpy as np
from PIL import Image

pack = Path("$PACK")
manifest = json.loads((pack / "manifest.json").read_text())
counts = manifest["counts"]
assert counts["portals"] >= 2, counts
portals = json.loads((pack / "portals.json").read_text())["portals"]
ids = sorted(p["shortId"] for p in portals)
print("portals:", ids)

img = json.loads((pack / "images.json").read_text())["images"][0]
jpeg = Image.open(pack / img["path"])
feat = np.load(pack / "features" / f"{img['id']}.npz")
if "image_size" in feat:
    fs = tuple(int(x) for x in feat["image_size"])
    assert fs == jpeg.size[::-1] or fs == jpeg.size, (fs, jpeg.size)
print("spot-check image 0:", img["path"], "jpeg=", jpeg.size, "feat_kpts=", feat["keypoints"].shape)
print("OK:", "$PACK", counts)
PY

echo "Prior pack ready: $PACK"
