#!/usr/bin/env python3
"""Assert recon→prior-pack convert preserves optional track rgb."""

from __future__ import annotations

import json
import shutil
import sys
import tempfile
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from scripts.prior_pack.convert import convert_recon_to_prior_pack  # noqa: E402
from scripts.prior_pack.generate_fixture import write_recon_fixture  # noqa: E402
from scripts.prior_pack.validate import validate_prior_pack  # noqa: E402


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        recon_root = root / "recon"
        pack_root = root / "pack"

        write_recon_fixture(recon_root)
        recon_tracks = json.loads((recon_root / "tracks.json").read_text(encoding="utf-8"))[
            "tracks"
        ]
        colored = [t for t in recon_tracks if "rgb" in t]
        if not colored:
            print("FAIL: recon fixture has no colored tracks", file=sys.stderr)
            return 1

        convert_recon_to_prior_pack(recon_root, pack_root)
        validate_prior_pack(pack_root)

        pack_tracks = json.loads((pack_root / "tracks.json").read_text(encoding="utf-8"))[
            "tracks"
        ]
        pack_by_id = {t["id"]: t for t in pack_tracks}
        for src in colored:
            dst = pack_by_id.get(src["id"])
            if dst is None:
                print(f"FAIL: track {src['id']} missing in pack", file=sys.stderr)
                return 1
            if dst.get("rgb") != src["rgb"]:
                print(
                    f"FAIL: track {src['id']} rgb {dst.get('rgb')} != {src['rgb']}",
                    file=sys.stderr,
                )
                return 1

        manifest = json.loads((pack_root / "manifest.json").read_text(encoding="utf-8"))
        if manifest["format_version"] < 2:
            print(
                f"FAIL: expected format_version >= 2, got {manifest['format_version']}",
                file=sys.stderr,
            )
            return 1

        # v1 packs without rgb still validate (backward compat).
        legacy_root = root / "legacy_pack"
        shutil.copytree(pack_root, legacy_root)
        legacy_tracks_path = legacy_root / "tracks.json"
        legacy_doc = json.loads(legacy_tracks_path.read_text(encoding="utf-8"))
        for track in legacy_doc["tracks"]:
            track.pop("rgb", None)
        legacy_tracks_path.write_text(
            json.dumps(legacy_doc, indent=2) + "\n", encoding="utf-8"
        )
        legacy_manifest_path = legacy_root / "manifest.json"
        legacy_manifest = json.loads(legacy_manifest_path.read_text(encoding="utf-8"))
        legacy_manifest["format_version"] = 1
        legacy_manifest_path.write_text(
            json.dumps(legacy_manifest, indent=2) + "\n", encoding="utf-8"
        )
        validate_prior_pack(legacy_root)

    print("OK: prior pack rgb preserved through convert; v1 without rgb validates")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
