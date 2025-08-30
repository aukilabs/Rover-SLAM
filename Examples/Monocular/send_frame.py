#!/usr/bin/env python3
"""
Send a single local image file OR a folder of images to the SLAM server.

Usage (single image):
  python3 send_frame.py path/to/image.jpg \
      --server http://localhost:8000 \
      --frame-id 123 \
      --timestamp now \
      --intrinsics 500 500 320 240 \
      [--raw]

Usage (folder at N FPS, one by one):
  python3 send_frame.py path/to/folder \
      --server http://localhost:8000 \
      --fps 10 \
      [--loop] \
      [--intrinsics fx fy cx cy | path/to/intrinsics.csv] \
      [--raw]
"""

import argparse
import base64
import json
import sys
import time
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import requests


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a frame or a folder of frames to the SLAM server")
    parser.add_argument("path", type=str, help="Path to image file or folder")
    parser.add_argument("--server", default="http://localhost:8000", help="Server base URL")
    parser.add_argument("--frame-id", type=int, default=None, help="Optional frame id (default: epoch ms or sequential)")
    parser.add_argument(
        "--timestamp",
        default=None,
        help="Optional timestamp in seconds (float). Use 'now' to use current time. Default: now",
    )
    # Accept either four floats or a single CSV path
    parser.add_argument(
        "--intrinsics",
        nargs="+",
        metavar="INTRINSICS",
        help="Either four floats 'fx fy cx cy' or a CSV path with one 'fx,fy,cx,cy' per line",
    )
    parser.add_argument("--raw", action="store_true", help="Send raw image bytes instead of base64 JSON")
    parser.add_argument("--fps", type=float, default=0.0, help="If path is a folder, send at this FPS (0 = as fast as possible)")
    parser.add_argument("--loop", action="store_true", help="If path is a folder, loop indefinitely over the images")
    return parser.parse_args()


def read_file_bytes(p: Path) -> bytes:
    with open(p, "rb") as f:
        return f.read()


def guess_content_type(p: Path) -> str:
    ext = p.suffix.lower()
    if ext in (".jpg", ".jpeg"):
        return "image/jpeg"
    if ext == ".png":
        return "image/png"
    if ext == ".bmp":
        return "image/bmp"
    return "application/octet-stream"


def current_timestamp(ts_arg: Optional[str]) -> float:
    if ts_arg is None or ts_arg == "now":
        return time.time()
    return float(ts_arg)


def send_bytes(url_track: str, data: bytes, headers: dict) -> requests.Response:
    return requests.post(url_track, data=data, headers=headers, timeout=30)


def send_base64_json(url_track: str, data: bytes, headers: dict, timestamp: float, frame_id: int, intrinsics: Optional[List[float]]) -> requests.Response:
    b64 = base64.b64encode(data).decode("utf-8")
    payload = {
        "image_base64": b64,
        "timestamp": timestamp,
        "frame_id": frame_id,
    }
    if intrinsics is not None:
        payload["intrinsics"] = intrinsics
    # Ensure JSON header
    headers = {**headers, "Content-Type": "application/json"}
    return requests.post(url_track, json=payload, headers=headers, timeout=30)


def parse_intrinsics_values(values: List[str]) -> List[float]:
    """Parse a list of 4 strings into [fx, fy, cx, cy] floats."""
    if len(values) != 4:
        raise ValueError("Expected exactly 4 values for intrinsics: fx fy cx cy")
    try:
        return [float(v) for v in values]
    except Exception as exc:
        raise ValueError(f"Invalid intrinsics floats: {values}") from exc


def load_intrinsics_csv(csv_path: Path) -> List[List[float]]:
    """Load per-image intrinsics from a CSV/whitespace-separated file.

    Each non-empty, non-comment line should contain 5 or more numeric values: timestamp, fx, fy, cx, cy.
    Timestamp is ignored.
    Extra columns are ignored.
    Comma or whitespace are both supported separators.
    """
    print("Loading intrinsics from CSV: ", csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f"Intrinsics CSV not found: {csv_path}")
    per_image: List[List[float]] = []
    with open(csv_path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            # Support comma and/or whitespace
            # Replace commas with spaces, then split
            parts = line.replace(",", " ").split()
            if len(parts) < 5:
                raise ValueError(f"Intrinsics CSV line has fewer than 4 values: '{raw_line.strip()}'")
            try:
                parts = parts[1:]
                fx, fy, cx, cy = (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
            except Exception as exc:
                raise ValueError(f"Invalid numeric values in intrinsics CSV line: '{raw_line.strip()}'") from exc
            per_image.append([fx, fy, cx, cy])
        print("Loaded intrinsics count: ", len(per_image))
    if not per_image:
        raise ValueError(f"No valid intrinsics rows found in: {csv_path}")
    return per_image


def send_one_file(
    server: str,
    p: Path,
    raw: bool,
    ts_arg: Optional[str],
    frame_id: Optional[int],
    intrinsics: Optional[List[float]],
    client_id: str = "send_frame.py",
) -> Tuple[int, str]:
    url_track = server.rstrip("/") + "/api/v1/track"

    ts = current_timestamp(ts_arg)
    fid = frame_id if frame_id is not None else int(ts * 1000)

    data = read_file_bytes(p)

    if raw:
        headers = {"Content-Type": guess_content_type(p), "X-Client-ID": client_id}
        print("intrinsics: ", intrinsics)
        # If intrinsics provided, pass via header for raw mode
        if intrinsics is not None:
            headers["X-Intrinsics"] = ",".join(str(x) for x in intrinsics)
        resp = send_bytes(url_track, data, headers)
    else:
        headers = {"X-Client-ID": client_id}
        resp = send_base64_json(url_track, data, headers, ts, fid, intrinsics)

    ctype = resp.headers.get("Content-Type", "")
    body = resp.text if "application/json" not in ctype else json.dumps(resp.json(), indent=2)
    return resp.status_code, body


def list_images(folder: Path) -> List[Path]:
    files = [p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_EXTS]
    files.sort()
    return files


def send_folder(
    folder: Path,
    server: str,
    fps: float,
    loop: bool,
    raw: bool,
    ts_arg: Optional[str],
    intrinsics: Optional[List[float]],
    intrinsics_per_image: Optional[List[List[float]]] = None,
) -> int:
    images = list_images(folder)
    if not images:
        print(f"No images found in folder: {folder}", file=sys.stderr)
        return 1

    interval = 0.0 if fps <= 0 else 1.0 / fps
    seq_id = int(time.time() * 1000)

    print(f"Sending {len(images)} images from {folder} at {('max speed' if interval==0 else f'{fps} FPS')} (loop={'on' if loop else 'off'})")

    try:
        while True:
            start_loop = time.time()
            for idx, p in enumerate(images):
                t0 = time.time()
                this_intrinsics: Optional[List[float]] = intrinsics
                if intrinsics_per_image is not None:
                    if idx >= len(intrinsics_per_image):
                        print(
                            f"Error: intrinsics CSV has {len(intrinsics_per_image)} rows but there are {len(images)} images",
                            file=sys.stderr,
                        )
                        return 1
                    this_intrinsics = intrinsics_per_image[idx]

                status, body = send_one_file(server, p, raw, ts_arg, seq_id, this_intrinsics)
                print(f"[{p.name}] Status: {status}")
                if status != 200:
                    # Print error body for non-200s
                    print(body)
                seq_id += 1
                if interval > 0:
                    elapsed = time.time() - t0
                    sleep_time = interval - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
            if not loop:
                break
            # Prevent tight loop in case of very small folder
            if interval == 0:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print("Interrupted by user")
        return 130
    return 0


def main() -> int:
    args = parse_args()
    path = Path(args.path)

    if not path.exists():
        print(f"Error: path not found: {path}", file=sys.stderr)
        return 1

    # Interpret --intrinsics which can be four floats or a CSV path
    fixed_intrinsics: Optional[List[float]] = None
    per_image_intrinsics: Optional[List[List[float]]] = None
    if args.intrinsics is not None:
        if len(args.intrinsics) == 1:
            csv_path = Path(args.intrinsics[0])
            try:
                per_image_intrinsics = load_intrinsics_csv(csv_path)
            except Exception as e:
                print(f"Error loading intrinsics CSV: {e}", file=sys.stderr)
                return 1
        elif len(args.intrinsics) == 4:
            try:
                fixed_intrinsics = parse_intrinsics_values(args.intrinsics)
            except Exception as e:
                print(f"Error parsing intrinsics: {e}", file=sys.stderr)
                return 1
        else:
            print("Error: --intrinsics expects either 4 floats or a single CSV path", file=sys.stderr)
            return 1

    if path.is_dir():
        # If CSV provided, enforce row count equals number of images inside send_folder loop
        return send_folder(
            folder=path,
            server=args.server,
            fps=args.fps,
            loop=args.loop,
            raw=args.raw,
            ts_arg=args.timestamp,
            intrinsics=fixed_intrinsics,
            intrinsics_per_image=per_image_intrinsics,
        )

    # Single file mode
    try:
        # Single file mode: if CSV was provided, use the first row
        single_file_intrinsics = fixed_intrinsics
        if per_image_intrinsics is not None:
            single_file_intrinsics = per_image_intrinsics[0]

        status, body = send_one_file(
            server=args.server,
            p=path,
            raw=args.raw,
            ts_arg=args.timestamp,
            frame_id=args.frame_id,
            intrinsics=single_file_intrinsics,
        )
        print(f"Status: {status}")
        print(body)
    except requests.RequestException as e:
        print(f"HTTP error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())