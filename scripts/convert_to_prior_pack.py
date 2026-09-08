#!/usr/bin/env python3
"""Convert reconstruction layout to prior pack v1, or generate synthetic fixture."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.prior_pack.convert import convert_recon_to_prior_pack  # noqa: E402
from scripts.prior_pack.generate_fixture import generate_fixture_pack  # noqa: E402
from scripts.prior_pack.validate import validate_prior_pack_or_exit  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build prior pack v1 from recon layout or generate test fixture."
    )
    parser.add_argument(
        "--input",
        type=Path,
        help="Reconstruction input root (images.json, features/, tracks.json, ...)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output prior pack directory (created/overwritten)",
    )
    parser.add_argument(
        "--generate-fixture",
        action="store_true",
        help="Write testdata/recon_fixture and testdata/prior_pack_fixture under repo root",
    )
    parser.add_argument(
        "--no-validate",
        action="store_true",
        help="Skip post-convert validation",
    )
    args = parser.parse_args()

    if args.generate_fixture:
        pack_root = generate_fixture_pack(_REPO_ROOT)
        print(f"Generated fixture pack at {pack_root}")
        if args.no_validate:
            return 0
        return validate_prior_pack_or_exit(pack_root)

    if not args.input or not args.output:
        parser.error("--input and --output are required unless --generate-fixture is set")

    pack_root = convert_recon_to_prior_pack(args.input, args.output)
    print(f"Wrote prior pack to {pack_root}")
    if args.no_validate:
        return 0
    return validate_prior_pack_or_exit(pack_root)


if __name__ == "__main__":
    raise SystemExit(main())
