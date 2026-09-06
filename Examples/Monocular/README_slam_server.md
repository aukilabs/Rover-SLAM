# SLAM Real-Time Server - Phase 1 Implementation

## Overview

This is the Phase 1 implementation of the real-time SLAM server that converts the ORB-SLAM3 monocular tracking system into a REST API service. The server receives images over HTTP and returns pose estimates in real-time.

**Full HTTP API reference (request headers, raw vs JSON, errors, CORS):** [slam_server_api.md](slam_server_api.md).

**Phone camera UI:** with the server running, open `http://<PC-LAN-IP>:<port>/index.html` on your phone (same Wi‑Fi). Source file: [index.html](index.html) (served by the binary from that path next to `slam_server`). The same page draws live **timing** and **top-down trajectory** plots from `/api/v1/status` and `/api/v1/trajectory` (restart `slam_server` after editing the HTML — it is loaded once at startup).

**Holdout pose plots (SLAM vs GT / baseline):** separate static page via `scripts/serve_holdout_pose_eval.py` (default `0.0.0.0:8080`, dir `out/holdout_pose_eval/`).

## Features Implemented (Phase 1)

### ✅ Core Server Infrastructure
- **HTTP Server Framework**: Custom lightweight HTTP server implementation
- **REST API Design**: Complete API specification with JSON responses
- **Image Reception**: Support for base64 encoded images in JSON requests
- **Multi-threading**: Concurrent request handling with dedicated processing thread
- **Configuration Management**: Command-line configuration options

### ✅ API Endpoints

#### `POST /api/v1/track`
Submit an image for SLAM tracking.

**Request Format:**
```json
{
    "image_base64": "base64_encoded_image_data",
    "timestamp": 1234567890.123,
    "frame_id": 12345,
    "intrinsics": [fx, fy, cx, cy]  // optional
}
```

**Response Format:**
```json
{
    "success": true,
    "message": "Image queued for processing",
    "frame_id": 12345,
    "timestamp": 1234567890.123
}
```

#### `GET /api/v1/status`
Get current server status and statistics (queue, tracker, portal alignment, **prior pack**, **multi-map publish**, **overlay**, **nav_pose**, **pose_source**). See **[slam_server_api.md](slam_server_api.md)** for the full schema.

**Response Format (core fields):**
```json
{
    "queue_size": 2,
    "max_queue_size": 10,
    "total_requests": 150,
    "successful_tracks": 142,
    "failed_tracks": 8,
    "avg_processing_time_ms": 45.2,
    "tracking_state": 2,
    "is_lost": false,
    "multi_map_publish": { "policy": "active_only", "withheld_map_ids": [] },
    "prior_pack": { "loaded": true },
    "overlay": { "prior_soft_edges_accepted": 0 },
    "pose_source": "slam"
}
```

#### `GET /api/v1/trajectory`
Get trajectory poses for the active map (`?dense=0` for keyframes only, `?aligned=0` for raw SLAM coordinates). The response also includes app-renderable `placed_data` rows (`barcode_ean13`, `portal_corner`) and 2D `frame_detections` rows for overlays/debugging. See **[slam_server_api.md](slam_server_api.md)** for the full response schema, detector-only high-res still behavior, and **coordinate convention**: `position`/`placed_data[].point` are world coordinates in the reported frame; `orientation` = `[w,x,y,z]` for `R_wc × diag(1,−1,−1)` (Three.js-friendly camera rotation).

#### `GET /api/v1/health`
Health check endpoint.

#### `GET /`
API information and available endpoints.

## Architecture

### Components

1. **SLAMServer Class**: Main server orchestrator
   - Manages ORB-SLAM3 system lifecycle
   - Handles HTTP server setup and routing
   - Coordinates between request handling and SLAM processing

2. **HTTP Server**: Lightweight custom implementation
   - Multi-threaded request handling
   - CORS support for web clients
   - Route matching and parameter extraction

3. **Image Processing Pipeline**:
   - Thread-safe image queue
   - Base64 image decoding
   - Integration with ORB-SLAM3 `TrackMonocular()`
   - Pose result serialization

4. **Configuration System**:
   - Command-line argument parsing
   - Configurable server parameters
   - Camera settings from YAML file

### Threading Model

- **Main Thread**: HTTP server and request handling
- **Processing Thread**: Dedicated SLAM processing thread
- **Request Threads**: Individual threads for each HTTP request

### Data Structures

```cpp
struct ImageRequest {
    cv::Mat image;
    double timestamp;
    string client_id;
    uint64_t frame_id;
    vector<float> intrinsics;
};

struct TrackingResponse {
    bool success;
    Sophus::SE3f pose;
    double timestamp;
    uint64_t frame_id;
    string error_message;
    double processing_time;
};
```

## Example (demo)

Start server:
```
./Examples/Monocular/slam_server Vocabulary/voc_binary_tartan_8u_6.yml.gz  Examples/Monocular-Inertial/DMT.yaml --output-dir ./outputs/slam_server  --port 8000 --viewer | tee log.txt
```

Send RGB frames from folder:
```
python3 Examples/Monocular/send_frame.py datasets/dmt_euroc/85a134bb-ecbd-40c0-bff8-e0381f6ef580/2025-01-10_22-03-56/mav0/cam0/data/ --fps 10 --server http://localhost:8000 --raw
```


## ALIKED + LightGlue (default stack)

When built with LibTorch (`LIBTORCH_ROOT` set at configure time), `slam_server` uses the MrNeRF ALIKED extractor (`aliked-n16`, top_k≈1024) and LightGlue matcher from `third_party/Light_Glue_CPP`. Weights are read from `third_party/Light_Glue_CPP/models/` (`aliked-n16.pt`, `aliked_lightglue.pt`).

### Build requirements

| Dependency | Notes |
|------------|--------|
| `LIBTORCH_ROOT` | CUDA LibTorch prefix (env var or `-DLIBTORCH_ROOT=...`). Required for the ALIKED+LightGlue path. |
| CUDA | **12.1+** (enforced by `cmake/modules/light_glue_cpp.cmake`). |
| Model weights | `third_party/Light_Glue_CPP/models/aliked-n16.pt`, `aliked_lightglue.pt` (and siblings if you switch models). |

```bash
export LIBTORCH_ROOT=/opt/libtorch
mkdir build && cd build
cmake .. -DLIBTORCH_ROOT=$LIBTORCH_ROOT
make slam_server
```

Without `LIBTORCH_ROOT`, CMake skips `auki_lightglue_lib` and the legacy ONNX SuperPoint/LightGlue path is used instead.

### Loop closure and relocalization (deferred in v1)

**Set `loopClosing: 0` in your settings YAML.** If the key is omitted, `System.cc` defaults loop closing **on**, which is not supported for the ALIKED v1 stack.

Loop closure and relocalization are **not available** in v1 (no ALIKED DBoW vocabulary; EigenPlaces global retrieval is a planned follow-on). Tracking and mapping continue to work with loop closing disabled.

Example configs (`iphoneSLAM.yaml`, `DMT.yaml`, `Mentra.yaml`) ship with `loopClosing: 0`.

### Prior pack (offline converter, S1)

Hybrid prior-map work uses an on-disk **prior pack** (manifest, images, ALIKED features, covisibility, tracks, portals). Schema: `scripts/prior_pack/schema.md`.

```bash
# Generate synthetic fixture under testdata/
python scripts/convert_to_prior_pack.py --generate-fixture

# Convert recon layout → pack
python scripts/convert_to_prior_pack.py --input <recon_root> --output <pack_root>

# Validate required parts (exit 1 + clear error on missing/corrupt)
python scripts/validate_prior_pack.py <pack_root>
```

Pack loading in `slam_server` is optional (slice S2). Without a pack, startup and tracking behave as before.

```bash
# CLI (overrides settings YAML)
./slam_server vocabulary.txt settings.yaml --prior-pack testdata/prior_pack_fixture

# Or in settings YAML:
# Server.priorPack: testdata/prior_pack_fixture
# (alternate key: prior_pack)
```

`GET /api/v1/status` includes a `prior_pack` object when loaded (`loaded`, `path`, `counts.images`, `counts.tracks`, `counts.portals`, `counts.covis_edges`, `counts.keypoints`). When no pack is configured, `prior_pack.loaded` is `false`.

Invalid or incomplete packs fail at startup with a clear error (same as `--occlusion-mesh`). JSON feature files are supported; `.npz` features are rejected until a later slice adds NumPy archive parsing.

## Human demo (prior pack + multi-map publish)

Use this walkthrough to confirm §5 human checks: pack load, domain pin, lose-track history vanish, re-pin restore, and overlay status fields.

### 1. Build and fixture pack

From the repo root (after [ALIKED + LightGlue build](#aliked--lightglue-default-stack)):

```bash
# Fixture pack (if missing)
python scripts/convert_to_prior_pack.py --generate-fixture
python scripts/validate_prior_pack.py testdata/prior_pack_fixture
```

### 2. Start `slam_server`

Run from repo root so `testdata/` and `Examples/Monocular/index.html` resolve. Default HTTP port is **8080** (`--port` to change).

```bash
./build/Examples/Monocular/slam_server \
  Vocabulary/voc_binary_tartan_8u_6.yml.gz \
  Examples/Monocular/iphoneSLAM.yaml \
  --port 8080 \
  --prior-pack testdata/prior_pack_fixture \
  --portal-manifest testdata/prior_pack_fixture/portals.json \
  --max-queue 10 \
  --slam-interval-nth 1 \
  --viewer \
  --debug-output ./outputs/slam_server/debug
```

Optional: `ROVER_PRIOR_SOFT_EDGE_FIXTURE_SMOKE=1` before launch exercises prior soft-edge acceptance at startup (sets `overlay.prior_soft_edges_accepted` ≥ 1 on GPU/LibTorch builds).

Startup log includes `Phone stream UI: http://<this-host>:<port>/index.html` — replace `<this-host>` with your PC LAN IP on the phone.

### 3. Phone stream + status legend

On the phone (same Wi‑Fi): open `http://<PC-LAN-IP>:8080/index.html`.

- Tap **Start camera** to POST JPEG frames to `/api/v1/track`.
- The page polls **`GET /api/v1/status` every 2s** and shows:
  - `prior_pack` — loaded path and image count
  - `multi_map_publish` — policy, included / **withheld** / pinned map ids
  - `overlay` — active map pinned, `prior_soft_edges_accepted`
  - `pose_source` — `slam` vs `overlay` (domain navigation pose)
  - `nav_pose` — latest navigation pose when tracking
  - tracker `is_lost` / `tracking_state`

`index.html` is the **camera stream + live plots** page (timing + top-down trajectory). It is **not** a 3D map viewer (use `--viewer` for Pangolin). Offline holdout SLAM-vs-baseline PNGs are served separately via `scripts/_stage_and_serve_holdout_eval.sh` on port **8080**. For other clients, poll **`GET /api/v1/trajectory`** and **draw only returned poses** — withheld pinned maps are omitted server-side when policy is `active_only`.

### 4. Demo script (what humans should see)

| Step | Action | Expect on status / trajectory |
|------|--------|--------------------------------|
| A | Walk with camera; no portal yet | `multi_map_publish.policy` = `active_only`; trajectory = active map only; `pose_source` = `slam`; `overlay.active_map_pinned` = false |
| B | Scan portal QR (`https://r8.hr/<shortId>`) matching manifest | `alignment.status` → aligned; `policy` → `domain_pinned_merge`; `pose_source` → `overlay`; Pangolin viewer (`--viewer`) shows domain-aligned map |
| C | Lose track (cover lens / fast motion until relocal fails) | New map; `policy` → `active_only`; `withheld_map_ids` lists prior pinned map(s); trajectory **drops** old segment — history “vanishes” in any client that redraws from API |
| D | Re-scan same portal on new map | `policy` → `domain_pinned_merge` again; withheld clears; trajectory merges **all** pinned maps in domain coords — history “returns” |

### 5. CLI / API spot checks

```bash
curl -s http://localhost:8080/api/v1/status | jq '.prior_pack,.multi_map_publish,.overlay,.pose_source,.nav_pose'
curl -s http://localhost:8080/api/v1/trajectory | jq '.count,.multi_map_publish,.pose_source'
```

### 6. Ops notes

| Topic | Notes |
|--------|--------|
| **Port** | Default `8080`; set `--port` (e.g. `8000`) if another service occupies it. |
| **LAN phone** | Phone must reach the PC IP; Windows firewall may need an inbound rule for the chosen port. |
| **HTTPS / iOS camera** | iOS often blocks `getUserMedia` on plain HTTP; use Android on LAN, or tunnel HTTPS (e.g. **ngrok**) to the same port. The stream page limits in-flight POSTs to avoid ngrok backlog stalls. |
| **WSL** | GPU builds are typically done inside WSL with LibTorch on Linux ext4; see `plans/aliked-lightglue-mrnerf.md` and `scripts/verify_gpu_smoke.sh`. Run `slam_server` from the same environment that built it. |
| **Folder replay** | Without a phone: `python3 Examples/Monocular/send_frame.py <image_dir> --fps 10 --server http://localhost:8080 --raw` |

Full field reference: [slam_server_api.md](slam_server_api.md).

### Vocabulary CLI argument (still required)

You must still pass a vocabulary file at startup, e.g. `Vocabulary/voc_binary_tartan_8u_6.yml.gz`. The binary loads the SuperPoint DBoW vocabulary during `System` construction even when `loopClosing: 0`. Making the argument optional is deferred until EigenPlaces/place-recognition follow-on work.

## Usage

### Building
```bash
export LIBTORCH_ROOT=/opt/libtorch   # adjust to your LibTorch install
mkdir build && cd build
cmake .. -DLIBTORCH_ROOT=$LIBTORCH_ROOT
make slam_server
```

### Running the Server
```bash
# Basic usage (vocabulary path still required; use a yaml with loopClosing: 0)
./slam_server Vocabulary/voc_binary_tartan_8u_6.yml.gz settings.yaml

# With options
./slam_server vocabulary.txt settings.yaml \
    --port 8080 \
    --output-dir ./results \
    --debug-output ./results/debug \
    --max-queue 20 \
    --slam-interval-nth 2 \
    --portal-manifest /path/to/refined_manifest.json \
    --viewer
```

### Command Line Options
- `--port <number>`: Server port (default: 8080)
- `--output-dir <path>`: Output directory for trajectories (default: ./output)
- `--debug-output <path>`: Directory for `trajectory_latest.json`, overwritten on every `GET /api/v1/trajectory` response (default: disabled)
- `--max-queue <number>`: Maximum image queue size (default: 10)
- `--slam-interval-nth <N>`: Run SLAM on one out of every N accepted frames, starting with the first accepted frame (default: 1)
- `--custom-detect-every <N>`: Run custom EAN-13 barcode detection every N accepted frames; use `X-Run-Detector` or JSON `run_detector` to force detection for a single upload (default: disabled)
- `--min-buffer <number>`: Min frames queued before tracking; oldest timestamp first (default: 3)
- `--portal-manifest <path>`: Enable portal QR alignment from a refined manifest containing `portals[].shortId`, `portals[].pose`, and metric `portals[].physicalSize`.
- `--prior-pack <path>`: Load optional prior pack root (`manifest.json` tree). Alternate config key: `Server.priorPack` or `prior_pack` in settings YAML.
- `--portal-detect-every <N>`: Run portal QR detection every N accepted frames (default: 1)
- `--portal-max-reproj-px <N>`: Reject portal PnP detections above this mean reprojection error (default: 8)
- `--viewer`: Enable SLAM viewer window (default: disabled)
- `--help`: Show usage information

### Portal Alignment

When `--portal-manifest` is set, the server detects QR payloads of the form `https://r8.hr/<SHORT_ID>`, runs OpenCV `solvePnP` with the manifest `physicalSize`, and estimates a domain-from-SLAM `Sim(3)`. One portal can provide an approximate metric scale from SLAM map points projected near the QR; repeated detections of the same QR or multiple portals refine scale from camera baselines. `GET /api/v1/status` and `GET /api/v1/trajectory` include an `alignment` object describing the current status and scale source.

### Testing

Use the provided Python test client:
```bash
python3 test_client.py --server http://localhost:8080 --frames 20 --pattern grid
```

## Dependencies

### Core Dependencies
- ORB-SLAM3 system (existing)
- OpenCV (existing)
- Eigen3 (existing)
- Custom HTTP library (included: `httplib_simple.h`)
- Custom JSON library (included: `json.hpp`)

### Header-Only Libraries
- `include/third_party/httplib_simple.h`: Simplified HTTP server
- `include/third_party/json.hpp`: JSON parsing and serialization

## File Structure

```
Examples/Monocular/
├── slam_server.cc           # Main server implementation
├── test_client.py           # Python test client
└── README_slam_server.md    # This documentation

include/third_party/
├── httplib_simple.h         # HTTP server library
└── json.hpp                 # JSON library
```

## Configuration

The server uses the same ORB-SLAM3 configuration files as the original examples. For ALIKED v1, include **`loopClosing: 0`** (see above). Default example YAMLs: `iphoneSLAM.yaml`, `Examples/Monocular-Inertial/DMT.yaml`, `Mentra.yaml`.

```yaml
# Example settings.yaml
loopClosing: 0   # required for ALIKED v1 (loop/reloc deferred)

Camera.fx: 500.0
Camera.fy: 500.0
Camera.cx: 320.0
Camera.cy: 240.0

# ... other ORB-SLAM3 settings
```

## Current Limitations (Phase 1)

1. **Synchronous Processing**: Currently queues images but doesn't support result polling
2. **Simple HTTP Implementation**: Basic HTTP server, not production-ready
3. **Limited Error Handling**: Basic error responses
4. **No Authentication**: No security measures implemented
5. **Memory Management**: Simple trajectory storage without persistence

## Performance Characteristics

- **Latency**: ~50-100ms per frame (depends on image complexity)
- **Throughput**: ~10-20 FPS sustained processing
- **Memory**: Bounded trajectory storage (configurable)
- **Concurrent Clients**: Supports multiple clients with request queuing

## Testing Results

The implementation successfully:
- ✅ Accepts images via REST API
- ✅ Processes them through ORB-SLAM3
- ✅ Returns JSON responses
- ✅ Maintains trajectory data
- ✅ Provides status monitoring
- ✅ Handles concurrent requests
- ✅ Supports graceful shutdown

## Next Steps (Future Phases)

1. **Enhanced HTTP Library**: Integrate production-ready HTTP server
2. **Result Polling**: Implement async processing with result retrieval
3. **Authentication**: Add API key or token-based security
4. **Persistence**: Trajectory and map saving/loading
5. **WebSocket Support**: Lower latency real-time communication
6. **Client SDKs**: Libraries for popular programming languages
7. **Performance Optimization**: Benchmarking and optimization
8. **Production Deployment**: Docker containers, monitoring, logging

## Smart Glasses Integration

The server is designed to be compatible with smart glasses or mobile devices:

```python
# Example smart glasses integration
import requests
import base64

def send_frame_to_slam(image_data):
    # Encode image
    encoded = base64.b64encode(image_data).decode('utf-8')
    
    # Send to SLAM server
    response = requests.post('http://slam-server:8080/api/v1/track', json={
        'image_base64': encoded,
        'timestamp': time.time()
    })
    
    return response.json()
```

## Conclusion

Phase 1 provides a solid foundation for real-time SLAM services with:
- Working REST API
- ORB-SLAM3 integration
- Multi-client support
- Comprehensive monitoring
- Easy testing and deployment

The implementation demonstrates the feasibility of converting ORB-SLAM3 into a real-time service and provides a platform for further enhancements in subsequent phases.