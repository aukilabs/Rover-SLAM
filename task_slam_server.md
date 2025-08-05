# SLAM Real-Time Server Task List

## Goal
Convert `mono_inertial_dmt.cc` into a real-time SLAM server that receives images over REST API from smart glasses or other devices, processes them through ORB-SLAM3, and returns tracking results.

## Architecture Overview
- HTTP REST server listening on configurable port
- Monocular-only SLAM (no IMU initially)
- Camera intrinsics from config file
- Real-time image processing with pose estimation
- JSON responses with tracking results

## Phase 1: Core Server Infrastructure (High Priority)

### 1.1 HTTP Server Framework Integration
- **Task**: Research and integrate C++ HTTP server library
- **Options**: cpp-httplib, crow, pistache, or beast (Boost)
- **Requirements**: Lightweight, handles multipart/form-data, thread-safe
- **Deliverable**: Basic HTTP server that can start/stop and handle requests

### 1.2 Basic REST API Design
- **Task**: Define API endpoints and request/response formats
- **Endpoints**:
  - `POST /api/v1/track` - Submit image for tracking
  - `GET /api/v1/status` - Get system status and tracking state
  - `GET /api/v1/trajectory` - Get current trajectory
  - `GET /api/v1/health` - Health check
- **Deliverable**: API specification document

### 1.3 Image Reception and Parsing
- **Task**: Handle image data in HTTP requests
- **Formats**: Support base64 encoded images and multipart/form-data
- **Validation**: Image format validation (PNG, JPG)
- **Error Handling**: Invalid format, size limits, corrupted data
- **Deliverable**: Image parsing module with error handling

## Phase 2: SLAM Integration (High Priority)

### 2.1 Remove File-Based Loading
- **Task**: Extract SLAM initialization from file-loading logic
- **Changes**: 
  - Remove `LoadImages()`, `LoadIMU()` functions
  - Remove file path arguments from main()
  - Keep camera intrinsics loading from config file
- **Deliverable**: Clean SLAM initialization without file dependencies

### 2.2 Real-Time Processing Pipeline
- **Task**: Adapt SLAM system for real-time operation
- **Changes**:
  - Remove sequential file processing loop
  - Implement request-based processing
  - Handle timestamp generation (use arrival time or provided timestamp)
  - Remove IMU data handling (monocular only)
- **Deliverable**: Single-image processing function

### 2.3 Threading and Concurrency
- **Task**: Design thread-safe processing architecture
- **Components**:
  - HTTP request handler threads
  - SLAM processing thread(s)
  - Result storage and retrieval
- **Synchronization**: Mutexes for SLAM system access
- **Deliverable**: Multi-threaded architecture design

## Phase 3: Request Processing (Medium Priority)

### 3.1 Image Queue Management
- **Task**: Implement thread-safe image queue
- **Features**:
  - FIFO queue for incoming images
  - Configurable queue size limits
  - Queue overflow handling (drop oldest/newest)
  - Priority handling for different clients
- **Deliverable**: Image queue implementation

### 3.2 Timestamp Management
- **Task**: Handle timestamp synchronization
- **Options**:
  - Use HTTP request arrival time
  - Accept timestamp in request payload
  - Generate sequential timestamps
- **Validation**: Ensure monotonic timestamps for SLAM
- **Deliverable**: Timestamp management module

### 3.3 Frame Processing Pipeline
- **Task**: Process individual frames through SLAM
- **Integration**: Call `SLAM.TrackMonocular()` with received images
- **Error Handling**: Track processing failures, lost tracking
- **Performance**: Measure and log processing times
- **Deliverable**: Frame processing wrapper

## Phase 4: Response Generation (Medium Priority)

### 4.1 Pose Data Serialization
- **Task**: Convert SLAM output to JSON responses
- **Data**: Camera pose (position, orientation), tracking confidence
- **Format**: Standardized JSON schema with timestamps
- **Optimization**: Efficient serialization for real-time performance
- **Deliverable**: JSON response generation

### 4.2 Trajectory Management
- **Task**: Store and serve trajectory data
- **Storage**: In-memory trajectory buffer with size limits
- **Queries**: Get full trajectory, trajectory since timestamp, latest N poses
- **Persistence**: Optional trajectory saving to disk
- **Deliverable**: Trajectory storage and query system

### 4.3 Status and Diagnostics
- **Task**: Provide system status information
- **Metrics**: Processing rate, queue size, tracking state, memory usage
- **Health**: System health indicators, error rates
- **Debugging**: Detailed error information for failed tracking
- **Deliverable**: Status and diagnostics API

## Phase 5: Configuration and Deployment (Medium Priority)

### 5.1 Configuration Management
- **Task**: Create server configuration system
- **Config**: Port, queue sizes, buffer limits, camera parameters
- **Format**: YAML or JSON configuration file
- **Runtime**: Hot-reload capability for some settings
- **Deliverable**: Configuration management system

### 5.2 Command Line Interface
- **Task**: Design command line arguments
- **Arguments**: 
  - `--vocabulary <path>` - ORB vocabulary file
  - `--config <path>` - Camera/SLAM configuration
  - `--port <number>` - Server port (default 8080)
  - `--output-dir <path>` - Optional trajectory output directory
- **Validation**: Argument validation and help text
- **Deliverable**: CLI argument parser

### 5.3 Logging and Monitoring
- **Task**: Implement comprehensive logging
- **Levels**: DEBUG, INFO, WARN, ERROR
- **Components**: Request logging, SLAM processing, errors
- **Format**: Structured logging (JSON) for monitoring tools
- **Deliverable**: Logging framework integration

## Phase 6: Performance and Reliability (Low Priority)

### 6.1 Performance Optimization
- **Task**: Optimize for real-time performance
- **Profiling**: Identify bottlenecks in image processing pipeline
- **Memory**: Efficient memory management, avoid allocations in hot path
- **Processing**: Optimize image format conversions
- **Deliverable**: Performance benchmarks and optimizations

### 6.2 Error Recovery
- **Task**: Implement robust error handling
- **SLAM Recovery**: Handle lost tracking, reinitializtion
- **Network**: Connection errors, malformed requests
- **Memory**: Out-of-memory conditions, resource limits
- **Deliverable**: Error recovery mechanisms

### 6.3 Load Testing and Stress Testing
- **Task**: Validate server under load
- **Testing**: Multiple concurrent clients, high frame rates
- **Metrics**: Latency, throughput, memory usage under load
- **Limits**: Determine maximum sustainable load
- **Deliverable**: Load testing results and recommendations

## Phase 7: Smart Glasses Integration (Low Priority)

### 7.1 Client SDK/Library
- **Task**: Create client library for smart glasses
- **Languages**: Consider Python, JavaScript, or native libraries
- **Features**: Image capture, HTTP client, error handling
- **Documentation**: Integration examples and tutorials
- **Deliverable**: Client SDK with examples

### 7.2 Protocol Optimization
- **Task**: Optimize network protocol for real-time use
- **Compression**: Image compression options
- **Batch Processing**: Multiple frames per request
- **WebSocket**: Consider WebSocket for lower latency
- **Deliverable**: Optimized communication protocol

## Technical Considerations

### Dependencies
- C++ HTTP server library (cpp-httplib recommended)
- JSON library (nlohmann/json)
- Base64 encoding/decoding library
- Threading primitives (std::thread, std::mutex)

### Data Structures
```cpp
struct ImageRequest {
    cv::Mat image;
    double timestamp;
    std::string client_id;
    uint64_t frame_id;
};

struct TrackingResponse {
    bool success;
    Sophus::SE3f pose;  // Camera pose
    double timestamp;
    uint64_t frame_id;
    std::string error_message;
};
```

### File Structure
```
src/
├── slam_server.cc          # Main server application
├── http_server.cc          # HTTP server implementation
├── image_processor.cc      # SLAM integration wrapper
├── trajectory_manager.cc   # Trajectory storage/retrieval
├── config_manager.cc       # Configuration handling
└── utils/
    ├── json_utils.cc       # JSON serialization
    ├── image_utils.cc      # Image parsing/conversion
    └── logging.cc          # Logging utilities
```

## Success Criteria
1. Server accepts images via REST and processes them through SLAM
2. Returns accurate pose estimates in real-time (<100ms latency)
3. Handles multiple concurrent clients
4. Maintains trajectory data and provides query capabilities
5. Robust error handling and recovery
6. Comprehensive logging and monitoring
7. Ready for integration with smart glasses or mobile devices

## Estimated Effort
- **Phase 1-2**: 2-3 weeks (Core functionality)
- **Phase 3-4**: 2-3 weeks (Request/response handling)
- **Phase 5-6**: 1-2 weeks (Configuration and optimization)
- **Phase 7**: 1 week (Client integration)
- **Total**: 6-9 weeks for full implementation