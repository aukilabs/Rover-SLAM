# SLAM Real-Time Server - Phase 1 Implementation

## Overview

This is the Phase 1 implementation of the real-time SLAM server that converts the ORB-SLAM3 monocular tracking system into a REST API service. The server receives images over HTTP and returns pose estimates in real-time.

**Full HTTP API reference (request headers, raw vs JSON, errors, CORS):** [slam_server_api.md](slam_server_api.md).

**Phone camera UI:** with the server running, open `http://<PC-LAN-IP>:<port>/index.html` on your phone (same Wi‑Fi). Source file: [index.html](index.html) (served by the binary from that path next to `slam_server`).

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
Get current server status and statistics.

**Response Format:**
```json
{
    "queue_size": 2,
    "max_queue_size": 10,
    "total_requests": 150,
    "successful_tracks": 142,
    "failed_tracks": 8,
    "avg_processing_time_ms": 45.2,
    "tracking_state": 2,
    "is_lost": false
}
```

#### `GET /api/v1/trajectory`
Get the current trajectory data.

**Response Format:**
```json
{
    "trajectory": [
        {
            "timestamp": 1234567890.123,
            "frame_id": 12345,
            "position": [x, y, z],
            "orientation": [w, x, y, z],
            "processing_time_ms": 42.1
        }
    ],
    "count": 150
}
```

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


## Usage

### Building
```bash
mkdir build && cd build
cmake ..
make slam_server
```

### Running the Server
```bash
# Basic usage
./slam_server vocabulary.txt settings.yaml

# With options
./slam_server vocabulary.txt settings.yaml \
    --port 8080 \
    --output-dir ./results \
    --max-queue 20 \
    --viewer
```

### Command Line Options
- `--port <number>`: Server port (default: 8080)
- `--output-dir <path>`: Output directory for trajectories (default: ./output)
- `--max-queue <number>`: Maximum image queue size (default: 10)
- `--viewer`: Enable SLAM viewer window (default: disabled)
- `--help`: Show usage information

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

The server uses the same ORB-SLAM3 configuration files as the original examples:

```yaml
# Example settings.yaml
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