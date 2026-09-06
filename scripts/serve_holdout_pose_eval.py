#!/usr/bin/env python3
"""Serve holdout pose eval results on 0.0.0.0:8080."""
from __future__ import annotations

import argparse
import functools
import http.server
import os
import socketserver
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument(
        "--dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "out" / "holdout_pose_eval",
    )
    args = ap.parse_args()
    root = args.dir.resolve()
    if not root.is_dir():
        raise SystemExit(f"missing dir: {root}")

    os.chdir(root)
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(root))
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer((args.host, args.port), handler) as httpd:
        print(f"Serving {root}")
        print(f"Open http://<this-host-ip>:{args.port}/")
        print(f"Bound {args.host}:{args.port}")
        httpd.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
