#!/usr/bin/env python3
"""Validate a prior pack v1 directory."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow running as scripts/validate_prior_pack.py without installing the package.
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.prior_pack.validate import validate_prior_pack_or_exit  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a prior pack v1 directory.")
    parser.add_argument(
        "pack_root",
        type=Path,
        help="Path to prior pack root (contains manifest.json)",
    )
    args = parser.parse_args()
    return validate_prior_pack_or_exit(args.pack_root)


if __name__ == "__main__":
    raise SystemExit(main())
