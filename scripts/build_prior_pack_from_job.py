#!/usr/bin/env python3
"""Build a prior pack from a full reconstruction-server job (all scans, no holdout).

Does not start slam_server — pack only. Launch manually with --prior-pack <output>.

Example (WSL):
  python3 scripts/build_prior_pack_from_job.py \\
    --job-artifacts /path/to/job-folder \\
    --output out/prior_pack_live
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.export_colmap_h5_to_recon_all import (  # noqa: E402
    export_colmap_h5_to_recon,
    validate_recon_shape,
    validate_track_self_reproj,
)
from scripts.prior_pack.convert import convert_recon_to_prior_pack  # noqa: E402
from scripts.prior_pack.validate import validate_prior_pack_or_exit  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Export all refined scans from a reconstruction-server job and convert "
            "to a prior pack (no holdout). slam_server launch is manual."
        )
    )
    parser.add_argument(
        "--job-artifacts",
        type=Path,
        required=True,
        help="Reconstruction job root (contains refined/ and datasets/)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Output prior pack directory",
    )
    parser.add_argument(
        "--recon-output",
        type=Path,
        default=None,
        help="Intermediate recon layout dir (default: <output>_recon)",
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
        help="Skip linking/copying JPEGs during export",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=None,
        help="Limit exported images (smoke / dry-run)",
    )
    parser.add_argument(
        "--skip-self-reproj",
        action="store_true",
        help="Skip track self-reproj gate on recon export",
    )
    parser.add_argument(
        "--no-validate",
        action="store_true",
        help="Skip prior-pack validation after convert",
    )
    args = parser.parse_args()

    recon_root = args.recon_output
    if recon_root is None:
        recon_root = Path(str(args.output) + "_recon")

    meta = export_colmap_h5_to_recon(
        args.job_artifacts,
        recon_root,
        max_images=args.max_images,
        image_mode=args.image_mode,
        skip_images=args.skip_images,
    )
    print("map_scans:", meta["map_scans"])
    print("recon counts:", json.dumps(meta["counts"], indent=2))
    validate_recon_shape(recon_root)
    print("recon layout validation: OK")
    if not args.skip_self_reproj:
        stats = validate_track_self_reproj(recon_root)
        print("track self-reproj gate: OK", json.dumps(stats, indent=2))

    pack_root = convert_recon_to_prior_pack(recon_root, args.output)
    print(f"Wrote prior pack to {pack_root}")
    if args.no_validate:
        print("Skipped pack validation (--no-validate)")
        print(f"Manual next step: slam_server ... --prior-pack {pack_root}")
        return 0

    rc = validate_prior_pack_or_exit(pack_root)
    print(f"Prior pack ready: {pack_root}")
    print(f"Manual next step: slam_server ... --prior-pack {pack_root}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
