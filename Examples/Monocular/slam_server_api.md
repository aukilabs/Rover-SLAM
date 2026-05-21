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

Queue depth is limited by `--max-queue` (default `10`). If `--slam-interval-nth N` is greater than `1`, accepted frames outside the interval return success but are not queued for SLAM.

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

Returns poses from the **active atlas map** (not a global multi-map export). By default every tracked frame is included; use `?dense=0` for keyframes only. If portal alignment is enabled and at least one portal has been accepted, poses are returned in the domain/COLMAP frame; use `?aligned=0` for raw SLAM coordinates.

If the server was started with `--debug-output <dir>`, every trajectory response is also written as pretty JSON to `<dir>/trajectory_latest.json`, overwriting the previous file.

**Query parameters**

| Param | Default | Meaning |
|--------|---------|---------|
| `dense` | `1` (on) | `dense=0` or `dense=false` → keyframes only; otherwise all frames on the active map. |
| `aligned` | `1` (on) | `aligned=0` or `aligned=false` → raw ORB-SLAM coordinates, even when portal alignment is available. |

**Response:** JSON

```json
{
  "trajectory": [
    {
      "timestamp": 1735689600.123,
      "frame_id": 42,
      "is_keyframe": false,
      "position": [x, y, z],
      "orientation": [w, x, y, z],
      "processing_time_ms": 45.2
    }
  ],
  "count": 1,
  "map_id": 0,
  "dense": true,
  "source": "active_map_dense_frames",
  "coordinate_frame": "domain_colmap",
  "alignment": {
    "enabled": true,
    "aligned": true,
    "status": "aligned",
    "scale": 1.23,
    "scale_source": "single_portal_local_points",
    "portal_observations": 1,
    "last_portal_short_id": "ABCD1234",
    "last_reprojection_error_px": 1.2,
    "last_local_points_used": 12
  }
}
```

### Coordinate convention (trajectory poses)

Each trajectory entry is produced by `TwcToClientJson` in `slam_server.cc` from a **world-from-camera** pose **`Twc`** (OpenCV / ORB camera convention: **+Z forward**, **+Y down**, **+X right** in camera space; translation is the **camera optical center in world**). With portal alignment active, `Twc` is first transformed by the current domain-from-SLAM `Sim(3)`.

| Field | Meaning |
|--------|---------|
| **`position`** | `Twc.translation()` in **meters** — camera center in the reported `coordinate_frame`. **No axis flip** is applied to translation. |
| **`orientation`** | Unit quaternion **`[w, x, y, z]`** (Hamilton, scalar-first) for the **3×3 rotation** `R_three = R_wc × cv_to_three`, where `R_wc` is the rotation part of `Twc` and `cv_to_three = diag(1, −1, −1)`. This right-multiplies the camera frame so **+Y and +Z are negated** relative to OpenCV — the usual fix for plotting ORB/OpenCV camera poses in **Three.js** (Y-up, −Z forward). |
| **`is_keyframe`** | Whether this sample came from a map keyframe (`dense=0`) or is a per-frame track sample (`dense=1`). |

**Not included:** `Tcw` / `cam_from_world`, OpenGL (ARKit) poses, or uncorrected `R_wc` quaternions. Clients that need raw SLAM `Twc` should request `GET /api/v1/trajectory?aligned=0`.

### Portal alignment

Start the server with `--portal-manifest <refined_manifest.json>` to enable portal alignment. The manifest must contain `portals[].shortId`, `portals[].pose.position`, `portals[].pose.rotation`, and metric `portals[].physicalSize`. Portal poses are loaded with the same OpenGL/domain-to-COLMAP conversion used by the reconstruction server: position `[x,y,z] -> [y,x,-z]`, quaternion `[x,y,z,w] -> [y,x,-z,w]`.

For each accepted frame, the server can detect QR payloads matching `https://r8.hr/<SHORT_ID>`, solve a metric portal pose with OpenCV `solvePnP` from the QR corners and `physicalSize`, then align ORB-SLAM's monocular trajectory to the domain frame. A single portal can provide an approximate scale by comparing the metric portal-plane geometry to SLAM map points projected near the QR in the same frame. Repeated detections of the same QR, or detections of multiple portals, refine scale from camera-center baselines.

When `--viewer` is enabled, the Pangolin map viewer applies the same domain-from-SLAM transform at render time after the first accepted portal alignment. This affects the displayed active-map points, keyframes, graph lines, and current camera only; the underlying ORB-SLAM map remains in raw SLAM coordinates for tracking, bundle adjustment, and loop closure.

`alignment.scale_source` indicates the active scale estimate:

| Value | Meaning |
|--------|---------|
| `single_portal_local_points` | Scale from multiple SLAM points projected near one detected portal. |
| `single_portal_local_depth` | Scale from one nearby SLAM point, less stable than point-pair scale. |
| `portal_camera_baseline` | Scale refined from repeated same-QR or multi-portal camera baselines. |
| `unit_fallback` | Portal pose anchored rotation/translation, but no reliable metric scale estimate was available. |

**Implementing the same convention elsewhere** (e.g. image localization servers): given world-from-camera `Twc` in an OpenCV-style camera frame,

```
R_three = R_wc @ [[1,0,0],[0,-1,0],[0,0,-1]]
position  = t_wc
orientation = quaternion(R_three) as [w, x, y, z]
```

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
./slam_server <vocabulary> <settings.yaml> [--port N] [--output-dir DIR] [--debug-output DIR] [--max-queue N] [--slam-interval-nth N] [--viewer] [--portal-manifest refined_manifest.json]
```

See `README_slam_server.md` for build notes and examples.

---

## Related files

- Implementation: `Examples/Monocular/slam_server.cc`
- Python sender: `Examples/Monocular/send_frame.py`
- Overview: `Examples/Monocular/README_slam_server.md`
