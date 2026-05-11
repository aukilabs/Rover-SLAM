# SLAM server HTTP API

This document describes the REST API implemented by `slam_server` (`Examples/Monocular/slam_server.cc`). The server wraps monocular ORB-SLAM3 tracking: clients **POST** images; SLAM runs on a **background thread**; clients read **status** and **trajectory** separately.

**Base URL:** `http://<host>:<port>/` (port from `--port`, default `8080`).

---

## CORS (browser clients)

The server sets:

- `Access-Control-Allow-Origin: *`
- `Access-Control-Allow-Methods: GET, POST, OPTIONS`
- `Access-Control-Allow-Headers: Content-Type, X-Intrinsics, X-Client-ID`

**Note:** If you add other custom headers, extend `Access-Control-Allow-Headers` in `slam_server.cc` so browser preflight succeeds.

## `GET /index.html`

Serves the **phone camera stream** static page from `Examples/Monocular/index.html` (loaded at startup from the executable’s directory, or `Examples/Monocular/index.html` relative to the current working directory). Same host/port as the API, so the page can `fetch('/api/v1/track', …)` without CORS issues.

If the file is missing, returns **404** with a short plain-text hint.

When the file loads successfully, the server logs a line such as: `Phone stream UI: http://<this-host>:<port>/index.html` (replace `<this-host>` with your PC’s LAN IP on the phone).

---

## `POST /api/v1/track`

Submit **one** image for tracking. The handler **queues** the frame and returns immediately; it does **not** wait for SLAM or return the pose in this response.

### Success

- **HTTP status:** `200`
- **Body:** JSON

```json
{
  "success": true,
  "message": "Image queued for processing",
  "frame_id": 12345,
  "timestamp": 1735689600.123
}
```

### Errors

| Status | Meaning |
|--------|--------|
| `400` | Failed to parse body / decode image (`success: false`, `error` message). |
| `503` | Image queue is full (`success: false`, `error` explains retry). |
| `500` | Internal exception (`success: false`, `error`). |

Queue depth is limited by `--max-queue` (default `10`).

---

### Request format A — JSON + Base64

Use when `Content-Type` contains **`application/json`** (e.g. `application/json; charset=utf-8`).

**Body:** JSON object.

| Field | Required | Type | Description |
|--------|------------|------|-------------|
| `image_base64` | Yes | string | Base64 encoding of **file-encoded** image bytes (e.g. JPEG/PNG as you would write to disk). Server decodes base64 then `cv::imdecode`. |
| `timestamp` | No | number | Time in **seconds** (double). If omitted, server uses a monotonic high-resolution clock value (seconds since an arbitrary origin—not Unix time). |
| `frame_id` | No | integer | Frame identifier. If omitted, server uses `total_requests + 1` at parse time. |
| `intrinsics` | No | array | Four floats `[fx, fy, cx, cy]` for this frame. Omitted → empty intrinsics vector passed to SLAM. |

**Example:**

```json
{
  "image_base64": "/9j/4AAQSkZJRg...",
  "timestamp": 1735689600.5,
  "frame_id": 42,
  "intrinsics": [500.0, 500.0, 320.0, 240.0]
}
```

---

### Request format B — Raw image bytes

Use when `Content-Type` does **not** contain the substring `application/json`.

**Body:** Raw bytes of a single image (JPEG, PNG, BMP, or other format OpenCV can decode).

**Headers (optional):**

| Header | Description |
|--------|-------------|
| `X-Intrinsics` | Four numbers: **`fx,fy,cx,cy`** (commas) or **space-separated**. Parsed into per-frame intrinsics. |
| `X-Client-ID` | Optional client label (stored on the request; default `"unknown"`). |

**Example:**

```http
POST /api/v1/track HTTP/1.1
Host: localhost:8081
Content-Type: image/jpeg
X-Intrinsics: 500,500,320,240

<binary JPEG bytes>
```

---

### Semantics

- **No IMU:** The server always calls monocular tracking with an **empty** IMU vector. YAML IMU sections do not receive HTTP data.
- **No pose in POST response:** After `POST`, poll **`GET /api/v1/trajectory`** (or use the Pangolin viewer with `--viewer` on the server machine).
- **Intrinsics:** Should match the **decoded image** you send (resolution and principal point). If they match your YAML defaults, you can omit them.

---

## `GET /api/v1/status`

**Response:** JSON

| Field | Description |
|--------|-------------|
| `queue_size` | Current number of images waiting to be processed. |
| `max_queue_size` | Configured maximum queue length. |
| `total_requests` | Count of accepted `POST /api/v1/track` requests (incremented on enqueue). |
| `successful_tracks` | Frames where tracking returned a non-zero pose. |
| `failed_tracks` | Frames where tracking failed or threw. |
| `avg_processing_time_ms` | Running average processing time per frame (milliseconds). |
| `tracking_state` | ORB-SLAM3 tracking state (when SLAM is initialized). |
| `is_lost` | Whether the tracker reports lost (when SLAM is initialized). |

---

## `GET /api/v1/trajectory`

Returns recent **successful** poses held in memory (ring buffer; oldest entries dropped when over cap, default **1000** poses in code).

**Response:** JSON

```json
{
  "trajectory": [
    {
      "timestamp": 1735689600.123,
      "frame_id": 42,
      "position": [x, y, z],
      "orientation": [w, x, y, z],
      "processing_time_ms": 45.2
    }
  ],
  "count": 1
}
```

- **`position`:** Translation in meters (same frame as SLAM output).
- **`orientation`:** Unit quaternion **`[w, x, y, z]`** (scalar-first).

---

## `GET /api/v1/health`

Liveness / metadata.

**Response:** JSON

| Field | Description |
|--------|-------------|
| `status` | e.g. `"healthy"` |
| `server` | Human-readable name |
| `version` | API/server version string |
| `uptime_seconds` | Server uptime |

---

## `GET /`

Returns a short JSON description and the list of endpoints (same information as this document).

---

## Command-line (server)

```text
./slam_server <vocabulary> <settings.yaml> [--port N] [--output-dir DIR] [--max-queue N] [--viewer]
```

See `README_slam_server.md` for build notes and examples.

---

## Related files

- Implementation: `Examples/Monocular/slam_server.cc`
- Python sender: `Examples/Monocular/send_frame.py`
- Overview: `Examples/Monocular/README_slam_server.md`
