# SLAM server HTTP API

This document describes the REST API implemented by `slam_server` (`Examples/Monocular/slam_server.cc`). The server wraps monocular ORB-SLAM3 tracking: clients **POST** images; SLAM runs on a **background thread**; clients read **status** and **trajectory** separately.

**Base URL:** `http://<host>:<port>/` (port from `--port`, default `8080`).

---

## CORS (browser clients)

The server sets:

- `Access-Control-Allow-Origin: *`
- `Access-Control-Allow-Methods: GET, POST, OPTIONS`
- `Access-Control-Allow-Headers: Content-Type, X-Intrinsics, X-Client-ID, X-Capture-Timestamp, X-Frame-Id, X-Run-Detector`

**Note:** If browser clients add other custom headers, extend `Access-Control-Allow-Headers` in `slam_server.cc` so preflight succeeds.

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
  "timestamp": 1735689600.123,
  "tracked": true,
  "detector_queued": false
}
```

High-resolution detector-only stills return immediately without entering the SLAM queue:

```json
{
  "success": true,
  "message": "Detector-only still accepted",
  "frame_id": 57,
  "timestamp": 1735689600.456,
  "tracked": false,
  "detector_only": true,
  "detector_queued": true
}
```

### Errors

| Status | Meaning |
|--------|--------|
| `400` | Failed to parse body / decode image (`success: false`, `error` message). |
| `409` | Detector-only still could not be queued (`success: false`, `detector_only: true`). |
| `503` | Image queue is full (`success: false`, `error` explains retry). |
| `500` | Internal exception (`success: false`, `error`). |

Queue depth is limited by `--max-queue` (default `10`). If `--slam-interval-nth N` is greater than `1`, accepted frames outside the interval return success with `tracked: false`, `slam_interval_nth: N`, and are not queued for SLAM. They may still queue custom detection if requested.

---

### Request format A — JSON + Base64

Use when `Content-Type` contains **`application/json`** (e.g. `application/json; charset=utf-8`).

**Body:** JSON object.

| Field | Required | Type | Description |
|--------|------------|------|-------------|
| `image_base64` | Yes | string | Base64 encoding of **file-encoded** image bytes (e.g. JPEG/PNG as you would write to disk). Server decodes base64 then `cv::imdecode`. |
| `timestamp` | No | number | Time in **seconds** (double). If omitted, server uses a monotonic high-resolution clock value (seconds since an arbitrary origin—not Unix time). |
| `frame_id` | No | integer | Frame identifier. If omitted, server uses `total_requests + 1` at parse time. |
| `intrinsics` | No | array | Four floats `[fx, fy, cx, cy]` for the decoded image. If omitted, the server falls back to YAML camera intrinsics and scales them when it resizes. |
| `run_detector` / `runDetector` | No | boolean-ish | If present, queues the full-resolution custom detector for this image. |
| `content_type` / `contentType` | No | string | Original image MIME type for JSON uploads, useful for HEIC/HEIF decode fallback and detector-only still classification. |

**Example:**

```json
{
  "image_base64": "/9j/4AAQSkZJRg...",
  "timestamp": 1735689600.5,
  "frame_id": 42,
  "intrinsics": [500.0, 500.0, 320.0, 240.0],
  "run_detector": true,
  "content_type": "image/jpeg"
}
```

---

### Request format B — Raw image bytes

Use when `Content-Type` does **not** contain the substring `application/json`.

**Body:** Raw bytes of a single image (JPEG, PNG, BMP, other formats OpenCV can decode, and HEIC/HEIF when the server was built with libheif support).

**Headers (optional):**

| Header | Description |
|--------|-------------|
| `X-Intrinsics` | Four numbers: **`fx,fy,cx,cy`** (commas) or **space-separated**. Parsed into per-frame intrinsics. |
| `X-Client-ID` | Optional client label (stored on the request; default `"unknown"`). |
| `X-Capture-Timestamp` | Capture timestamp in seconds (double). |
| `X-Frame-Id` | Frame identifier. |
| `X-Run-Detector` | If present, queues the full-resolution custom detector for this image. |

**Example:**

```http
POST /api/v1/track HTTP/1.1
Host: localhost:8081
Content-Type: image/jpeg
X-Intrinsics: 500,500,320,240
X-Capture-Timestamp: 1735689600.5
X-Frame-Id: 42
X-Run-Detector: 1

<binary JPEG bytes>
```

---

### Semantics

- **No IMU:** The server always calls monocular tracking with an **empty** IMU vector. YAML IMU sections do not receive HTTP data.
- **No pose in POST response:** After `POST`, poll **`GET /api/v1/trajectory`** (or use the Pangolin viewer with `--viewer` on the server machine).
- **Intrinsics:** Should match the **decoded image** you send (resolution and principal point). If omitted, the server uses YAML defaults; when an input image is resized for SLAM, intrinsics are scaled to the SLAM input size.
- **Custom detector:** `--custom-detect-every N` queues the detector every N accepted frames. `X-Run-Detector` or JSON `run_detector` / `runDetector` forces detection on a specific upload.
- **Detector-only high-res stills:** A forced HEIC/HEIF upload, or a forced image larger than roughly 1.5× the configured SLAM input size, is treated as detector-only. It keeps its original `frame_id` and timestamp, does not enter SLAM tracking, and its downstream placement uses the closest trajectory pose at or before that still frame.

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
| `skipped_tracks` | Accepted frames that did not enter SLAM, either due to `--slam-interval-nth` or detector-only still handling. |
| `slam_interval_nth` | Configured SLAM frame interval. |
| `custom_detect_every` | Configured detector interval; `0` means interval detection is disabled. |
| `custom_detection_enqueued` | Number of detector jobs accepted. |
| `custom_detection_dropped` | Number of detector jobs dropped due to the bounded detector queue. |
| `custom_detection_processed` | Number of detector jobs completed. |
| `custom_detection_queue_size` | Current detector queue depth. |
| `avg_processing_time_ms` | Running average processing time per frame (milliseconds). |
| `tracking_state` | ORB-SLAM3 tracking state (when SLAM is initialized). |
| `is_lost` | Whether the tracker reports lost (when SLAM is initialized). |
| `alignment` | Same alignment status object returned by `GET /api/v1/trajectory`. |

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
  "placed_data": [
    {
      "type": "barcode_ean13",
      "id": "9789129722512",
      "point": [0.12, -0.03, 1.45]
    },
    {
      "type": "portal_corner",
      "id": "ABCD1234_0",
      "point": [1.0, 0.0, 2.0]
    }
  ],
  "frame_detections": [
    {
      "frame_index": 57,
      "pose_frame_index": 54,
      "type": "barcode_ean13",
      "id": "9789129722512",
      "pixel": [1379.0, 2511.0]
    }
  ],
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
  },
  "custom_detection": {
    "detect_every": 1,
    "enqueued": 8,
    "dropped": 0,
    "processed": 8,
    "stored_observations": 3
  }
}
```

### Rendering `placed_data`

Use `placed_data` for app-visible world objects. Each row is already in the same `coordinate_frame` as `trajectory[].position`:

| Field | Meaning |
|--------|---------|
| `type` | Object kind. Current values are `barcode_ean13`, `portal_corner`, and `topography_mesh_obj`. |
| `id` | Stable object id. Barcodes use the EAN-13 text. Portal corners use `<portalShortId>_<cornerIndex>`. Omitted for `topography_mesh_obj`. |
| `point` | `[x, y, z]` world point in meters, in the response `coordinate_frame`. Omitted for `topography_mesh_obj`. |
| `obj` | For `topography_mesh_obj`, the OBJ supplied with `--occlusion-mesh`, with vertex positions converted into the response `coordinate_frame`. |

For Three.js-style rendering, draw `point` directly as a world-space position in the same scene as `trajectory[].position`. Do not apply the camera-orientation `cv_to_three` flip to points or translations; that flip is already baked only into `trajectory[].orientation`.

When `--occlusion-mesh <file.obj>` is provided and portal alignment is available, `barcode_ean13` points are placed by raycasting the latest detection for each barcode id against that OBJ mesh after applying the same manifest-to-`domain_colmap` conversion used for portal poses, so each id appears at most once in `placed_data`. Without a mesh hit, barcodes fall back to triangulation from two or more detections of the same barcode id using the matching camera poses and per-observation intrinsics. `portal_corner` points come from portal QR PnP observations and the active alignment. If there is not enough geometry yet, the detection can appear in `frame_detections` before it appears in `placed_data`.

### Rendering/Debugging `frame_detections`

Use `frame_detections` when you want to overlay 2D detections or debug why an object has not been placed yet:

| Field | Meaning |
|--------|---------|
| `frame_index` | The image frame where the detector saw the object. For detector-only high-res stills, this is the still's original frame id, not the associated SLAM frame. |
| `pose_frame_index` | Optional trajectory frame id used for 3D placement. It is the exact frame if available, otherwise the closest earlier trajectory frame. |
| `type` | Detection kind: `barcode_ean13` or `portal_corner`. |
| `id` | Barcode text or portal corner id. |
| `pixel` | `[x, y]` pixel in the detector source image. For high-res stills this is in high-res still coordinates, not the lower-resolution SLAM frame. |

If your app needs to draw a high-res still detection on a lower-resolution video frame, scale `pixel` by the ratio between the display/video frame size and the still size. The API intentionally keeps the original detector pixel so it matches the image that produced the detection.

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
./slam_server <vocabulary> <settings.yaml> [--port N] [--output-dir DIR] [--debug-output DIR] [--max-queue N] [--slam-interval-nth N] [--custom-detect-every N] [--min-buffer N] [--portal-manifest refined_manifest.json] [--portal-detect-every N] [--portal-max-reproj-px N] [--occlusion-mesh mesh.obj] [--viewer]
```

See `README_slam_server.md` for build notes and examples.

---

## Related files

- Implementation: `Examples/Monocular/slam_server.cc`
- Python sender: `Examples/Monocular/send_frame.py`
- Overview: `Examples/Monocular/README_slam_server.md`
