/**
* Real-Time SLAM Server
* Based on ORB-SLAM3 Monocular tracking
* 
* Receives images via REST API and returns pose estimates
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <thread>
#include <mutex>
#include <deque>
#include <memory>
#include <unordered_map>
#include <csignal>
#include <iterator>
#include <vector>
#include <atomic>

#ifdef _WIN32
#include <windows.h>
#define usleep(x) Sleep((x)/1000)
#else
#include <unistd.h>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <System.h>

// HTTP server library - cpp-httplib (header-only)
#include "../../include/third_party/httplib.h"

// JSON library - official nlohmann/json single-header
#include "../../include/third_party/nlohmann/json.hpp"

using namespace std;
using json = nlohmann::json;

namespace {

/** ORB camera Twc -> Three.js position + quaternion (w,x,y,z). */
json TwcToClientJson(const Sophus::SE3f &Twc, double timestamp, uint64_t frame_id, bool is_keyframe,
                     double processing_time_ms = 0.0)
{
    Eigen::Matrix3f cv_to_three;
    cv_to_three << 1,  0,  0,
                   0, -1,  0,
                   0,  0, -1;

    Eigen::Matrix3f R_three = Twc.so3().matrix() * cv_to_three;
    Eigen::Vector3f t_three = Twc.translation();
    Eigen::Quaternionf q_three(R_three);
    q_three.normalize();

    json pose_json;
    pose_json["timestamp"] = timestamp;
    pose_json["frame_id"] = frame_id;
    pose_json["is_keyframe"] = is_keyframe;
    pose_json["position"] = {t_three.x(), t_three.y(), t_three.z()};
    pose_json["orientation"] = {q_three.w(), q_three.x(), q_three.y(), q_three.z()};
    pose_json["processing_time_ms"] = processing_time_ms;
    return pose_json;
}

/** Default per-frame on the active atlas map only; ?dense=0 for keyframe-only. */
bool trajectoryUseDense(const httplib::Request& req)
{
    if (!req.has_param("dense"))
        return true;
    const string v = req.get_param_value("dense");
    return v != "0" && v != "false";
}

} // namespace

// Server configuration structure
struct ServerConfig {
    int port = 8080;
    string vocabulary_path;
    string settings_path;
    string output_dir = "./output";
    bool use_viewer = false;
    int max_queue_size = 10;
    /** Wait until this many frames are buffered (unless shutting down); then process oldest by `timestamp`. */
    size_t min_buffer_frames = 3;
    double processing_timeout = 5.0; // seconds
    /** UTF-8 HTML for GET /index.html (phone camera UI); empty => 404 with hint */
    string index_html_body;
};

// Image request structure
struct ImageRequest {
    cv::Mat image;
    double timestamp;
    string client_id;
    uint64_t frame_id;
    vector<float> intrinsics; // Optional frame-specific intrinsics
};

class SLAMServer {
private:
    ServerConfig config_;
    unique_ptr<ORB_SLAM3::System> slam_system_;
    unique_ptr<httplib::Server> http_server_;
    
    // Thread safety
    mutex slam_mutex_;
    mutex queue_mutex_;
    
    // Incoming frames (potentially unordered arrival); worker picks smallest timestamp when buffer policy allows.
    deque<shared_ptr<ImageRequest>> image_queue_;
    condition_variable queue_cv_;
    atomic<bool> processing_active_{true};
    thread processing_thread_;
    
    // Statistics
    atomic<uint64_t> total_requests_{0};
    atomic<uint64_t> successful_tracks_{0};
    atomic<uint64_t> failed_tracks_{0};
    atomic<double> avg_processing_time_{0.0};
    string index_html_body_;
    atomic<bool> shutdown_done_{false};

public:
    SLAMServer(const ServerConfig& config) : config_(config), index_html_body_(config.index_html_body) {
        // Initialize SLAM system
        // Empty save folder: do not auto-enable System.SaveAtlasToFile default "Atlas"
        // (server exits often; atlas I/O is slow and usually not needed here).
        slam_system_ = make_unique<ORB_SLAM3::System>(
            config_.vocabulary_path, 
            config_.settings_path,
            ORB_SLAM3::System::MONOCULAR,
            config_.use_viewer,
            0, 
            "",
            ""
        );

        if (config_.min_buffer_frames < 1) {
            cerr << "Warning: min_buffer_frames must be >= 1; using 1." << endl;
            config_.min_buffer_frames = 1;
        }
        if (config_.max_queue_size < static_cast<int>(config_.min_buffer_frames)) {
            cerr << "Warning: max_queue_size (" << config_.max_queue_size
                 << ") < min_buffer_frames (" << config_.min_buffer_frames
                 << "); raising max_queue_size." << endl;
            config_.max_queue_size = static_cast<int>(config_.min_buffer_frames);
        }
        
        // Initialize HTTP server
        http_server_ = make_unique<httplib::Server>();
        setupRoutes();
        
        // Start processing thread
        processing_thread_ = thread(&SLAMServer::processImages, this);
        
        cout << "SLAM Server initialized successfully" << endl;
        cout << "Vocabulary: " << config_.vocabulary_path << endl;
        cout << "Settings: " << config_.settings_path << endl;
        if (!config_.output_dir.empty())
            cout << "Output directory (reserved / not used for ORB atlas save): " << config_.output_dir << endl;
        if (!index_html_body_.empty())
            cout << "Phone stream UI: http://<this-host>:" << config_.port << "/index.html" << endl;
    }
    
    ~SLAMServer() {
        shutdown();
    }
    
    void start() {
        cout << "Starting SLAM server on port " << config_.port << endl;
        if (!http_server_->listen("0.0.0.0", config_.port)) {
            throw runtime_error("Failed to start HTTP server on port " + to_string(config_.port));
        }
    }

    /** Async-signal-safe enough for SIGINT: only closes the listen socket (unblocks listen). */
    void requestStop() {
        if (http_server_)
            http_server_->stop();
    }
    
    void shutdown() {
        bool expected = false;
        if (!shutdown_done_.compare_exchange_strong(expected, true))
            return;

        cout << "Shutting down SLAM server..." << endl;
        
        // Stop HTTP server
        if (http_server_) {
            http_server_->stop();
        }
        
        // Stop processing
        processing_active_ = false;
        queue_cv_.notify_all();
        
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        
        // Shutdown SLAM
        if (slam_system_) {
            slam_system_->Shutdown();
        }
        
        cout << "SLAM server shutdown complete" << endl;
    }

private:
    void setupRoutes() {
        // Enable CORS for web clients
        http_server_->set_pre_routing_handler([](const httplib::Request& req, httplib::Response& res) {
            res.set_header("Access-Control-Allow-Origin", "*");
            res.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
            res.set_header("Access-Control-Allow-Headers",
                "Content-Type, X-Intrinsics, X-Client-ID, X-Capture-Timestamp, X-Frame-Id");
            return httplib::Server::HandlerResponse::Unhandled;
        });
        
        // Handle OPTIONS requests for CORS
        http_server_->Options(".*", [](const httplib::Request&, httplib::Response& res) {
            return;
        });
        
        // Static phone stream UI (same origin as API)
        http_server_->Get("/index.html", [this](const httplib::Request&, httplib::Response& res) {
            if (index_html_body_.empty()) {
                res.status = 404;
                res.set_content(
                    "index.html not found. Place index.html next to the slam_server executable "
                    "(same folder as Examples/Monocular/slam_server when using default CMake output).",
                    "text/plain; charset=utf-8");
                return;
            }
            res.set_header("Cache-Control", "no-store");
            res.set_content(index_html_body_, "text/html; charset=utf-8");
        });

        // Main tracking endpoint
        http_server_->Post("/api/v1/track", [this](const httplib::Request& req, httplib::Response& res) {
            handleTrackRequest(req, res);
        });
        
        // Status endpoint
        http_server_->Get("/api/v1/status", [this](const httplib::Request& req, httplib::Response& res) {
            handleStatusRequest(req, res);
        });
        
        // Trajectory endpoint
        http_server_->Get("/api/v1/trajectory", [this](const httplib::Request& req, httplib::Response& res) {
            handleTrajectoryRequest(req, res);
        });
        
        // Health check endpoint
        http_server_->Get("/api/v1/health", [this](const httplib::Request& req, httplib::Response& res) {
            handleHealthRequest(req, res);
        });
        
        // Root endpoint with API info
        http_server_->Get("/", [this](const httplib::Request& req, httplib::Response& res) {
            handleRootRequest(req, res);
        });
    }
    
    void handleTrackRequest(const httplib::Request& req, httplib::Response& res) {
        auto start_time = chrono::high_resolution_clock::now();
        json response;
        
        try {
            // Parse request
            auto image_req = parseImageRequest(req);
            if (!image_req) {
                response["success"] = false;
                response["error"] = "Failed to parse image request";
                res.set_content(response.dump(), "application/json");
                res.status = 400;
                return;
            }
            
            // Check queue size
            {
                lock_guard<mutex> lock(queue_mutex_);
                if (image_queue_.size() >= config_.max_queue_size) {
                    response["success"] = false;
                    response["error"] = "Server queue full, try again later";
                    res.set_content(response.dump(), "application/json");
                    res.status = 503;
                    return;
                }
                
                // Add to processing queue
                image_queue_.push_back(image_req);
                queue_cv_.notify_one();
            }
            
            // For now, return immediate response
            // TODO: Implement synchronous processing or result polling
            response["success"] = true;
            response["message"] = "Image queued for processing";
            response["frame_id"] = image_req->frame_id;
            response["timestamp"] = image_req->timestamp;
            
            total_requests_++;
            
            res.set_content(response.dump(), "application/json");
            
        } catch (const exception& e) {
            response["success"] = false;
            response["error"] = string("Internal error: ") + e.what();
            res.set_content(response.dump(), "application/json");
            res.status = 500;
        }
    }
    
    void handleStatusRequest(const httplib::Request& req, httplib::Response& res) {
        json response;
        
        {
            lock_guard<mutex> lock(queue_mutex_);
            response["queue_size"] = image_queue_.size();
        }
        
        response["max_queue_size"] = config_.max_queue_size;
        response["total_requests"] = total_requests_.load();
        response["successful_tracks"] = successful_tracks_.load();
        response["failed_tracks"] = failed_tracks_.load();
        response["avg_processing_time_ms"] = avg_processing_time_.load() * 1000.0;
        
        if (slam_system_) {
            response["tracking_state"] = slam_system_->GetTrackingState();
            response["is_lost"] = slam_system_->isLost();
        }
        
        res.set_content(response.dump(), "application/json");
    }
    
    void handleTrajectoryRequest(const httplib::Request& req, httplib::Response& res) {
        json response;
        json trajectory_json = json::array();

        const bool dense = trajectoryUseDense(req);

        unsigned long current_map_id = 0;
        vector<ORB_SLAM3::System::TrajectorySample> samples;
        if (slam_system_) {
            lock_guard<mutex> slam_lock(slam_mutex_);
            current_map_id = slam_system_->GetCurrentMapId();
            if (dense)
                slam_system_->GetActiveMapDenseTrajectory(samples);
            else
                slam_system_->GetActiveMapTrajectory(samples, true);
        }

        for (const auto& sample : samples) {
            trajectory_json.push_back(
                TwcToClientJson(sample.Twc, sample.timestamp, sample.id, sample.is_keyframe));
        }

        response["trajectory"] = trajectory_json;
        response["count"] = trajectory_json.size();
        response["map_id"] = current_map_id;
        response["dense"] = dense;
        response["source"] = dense ? "active_map_dense_frames" : "active_map_keyframes";

        res.set_content(response.dump(), "application/json");
    }
    
    void handleHealthRequest(const httplib::Request& req, httplib::Response& res) {
        json response;
        response["status"] = "healthy";
        response["server"] = "ORB-SLAM3 Real-Time Server";
        response["version"] = "1.0.0";
        response["uptime_seconds"] = chrono::duration_cast<chrono::seconds>(
            chrono::steady_clock::now() - server_start_time_).count();
        
        res.set_content(response.dump(), "application/json");
    }
    
    void handleRootRequest(const httplib::Request& req, httplib::Response& res) {
        json response;
        response["message"] = "ORB-SLAM3 Real-Time Server";
        response["version"] = "1.0.0";
        response["endpoints"] = {
            {"GET /index.html", "Phone camera stream UI (static page)"},
            {"POST /api/v1/track", "Submit image for tracking"},
            {"GET /api/v1/status", "Get server status"},
            {"GET /api/v1/trajectory", "Active-map trajectory (dense default; ?dense=0 keyframes only)"},
            {"GET /api/v1/health", "Health check"}
        };
        
        res.set_content(response.dump(), "application/json");
    }
    
    shared_ptr<ImageRequest> parseImageRequest(const httplib::Request& req) {
        auto image_req = make_shared<ImageRequest>();
        
        // Generate frame ID and timestamp
        image_req->frame_id = total_requests_.load() + 1;
        image_req->timestamp = chrono::duration<double>(
            chrono::high_resolution_clock::now().time_since_epoch()).count();
        
        // Get client ID from headers or generate one
        auto client_header = req.get_header_value("X-Client-ID");
        image_req->client_id = client_header.empty() ? "unknown" : client_header;
        
        // Parse image from request body
        if (req.get_header_value("Content-Type").find("application/json") != string::npos) {
            // JSON request with base64 encoded image
            try {
                json request_json = json::parse(req.body);
                
                if (request_json.contains("timestamp")) {
                    image_req->timestamp = request_json["timestamp"];
                }
                
                if (request_json.contains("frame_id")) {
                    image_req->frame_id = request_json["frame_id"];
                }
                
                if (request_json.contains("intrinsics")) {
                    image_req->intrinsics = request_json["intrinsics"].get<vector<float>>();
                }
                
                if (request_json.contains("image_base64")) {
                    string base64_data = request_json["image_base64"];
                    vector<uchar> image_data = base64Decode(base64_data);
                    image_req->image = cv::imdecode(image_data, cv::IMREAD_UNCHANGED);
                }
                
            } catch (const exception& e) {
                cout << "Error parsing JSON request: " << e.what() << endl;
                return nullptr;
            }
        } else {
            // Direct image upload
            vector<uchar> image_data(req.body.begin(), req.body.end());
            image_req->image = cv::imdecode(image_data, cv::IMREAD_UNCHANGED);

            const auto ts_hdr = req.get_header_value("X-Capture-Timestamp");
            if (!ts_hdr.empty()) {
                try {
                    image_req->timestamp = stod(ts_hdr);
                } catch (const exception& e) {
                    cerr << "Warning: invalid X-Capture-Timestamp '" << ts_hdr << "': " << e.what() << endl;
                }
            }
            const auto fid_hdr = req.get_header_value("X-Frame-Id");
            if (!fid_hdr.empty()) {
                try {
                    image_req->frame_id = stoull(fid_hdr);
                } catch (const exception& e) {
                    cerr << "Warning: invalid X-Frame-Id '" << fid_hdr << "': " << e.what() << endl;
                }
            }

            // Optional intrinsics via header for raw uploads
            // Expect header format: X-Intrinsics: "fx,fy,cx,cy" or space separated
            const auto intr_hdr = req.get_header_value("X-Intrinsics");
            if (!intr_hdr.empty()) {
                try {
                    cout << "Intrinsics (from request header): " << intr_hdr << endl;
                    string norm = intr_hdr;
                    // Replace commas with spaces and split
                    replace(norm.begin(), norm.end(), ',', ' ');
                    istringstream iss(norm);
                    vector<float> vals;
                    float v;
                    while (iss >> v) { vals.push_back(v); }
                    if (vals.size() >= 4) {
                        image_req->intrinsics = {vals[0], vals[1], vals[2], vals[3]};
                    } else {
                        cerr << "Warning: X-Intrinsics header has fewer than 4 values: '" << intr_hdr << "'" << endl;
                    }
                } catch (const exception& e) {
                    cerr << "Warning: failed to parse X-Intrinsics header: " << e.what() << endl;
                }
            }
            else {
                //cout << "No intrinsics header provided" << endl;
            }
        }
        
        if (image_req->image.empty()) {
            cout << "Failed to decode image from request" << endl;
            return nullptr;
        }
        
        return image_req;
    }
    
    vector<uchar> base64Decode(const string& encoded) {
        // Simple base64 decoder implementation
        // In production, use a proper base64 library
        static const string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        vector<uchar> decoded;
        
        int val = 0, valb = -8;
        for (uchar c : encoded) {
            if (c == '=') break;
            size_t pos = chars.find(c);
            if (pos == string::npos) continue;
            
            val = (val << 6) + pos;
            valb += 6;
            if (valb >= 0) {
                decoded.push_back(char((val >> valb) & 0xFF));
                valb -= 8;
            }
        }
        return decoded;
    }
    
    void processImages() {
        const size_t min_buf = config_.min_buffer_frames;
        cout << "Started image processing thread (min_buffer=" << min_buf
             << ", dequeue oldest timestamp)" << endl;

        while (true) {
            shared_ptr<ImageRequest> request;

            {
                unique_lock<mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this, min_buf] {
                    if (!processing_active_.load())
                        return true;
                    return image_queue_.size() >= min_buf;
                });

                if (image_queue_.empty()) {
                    if (!processing_active_.load())
                        break;
                    continue;
                }

                const bool shutting_down = !processing_active_.load();
                if (!shutting_down && image_queue_.size() < min_buf)
                    continue;

                auto oldest = min_element(
                    image_queue_.begin(),
                    image_queue_.end(),
                    [](const shared_ptr<ImageRequest>& a, const shared_ptr<ImageRequest>& b) {
                        return a->timestamp < b->timestamp;
                    });
                request = *oldest;
                image_queue_.erase(oldest);
            }

            if (request)
                processImageRequest(request);
        }

        cout << "Image processing thread stopped" << endl;
    }
    
    void processImageRequest(shared_ptr<ImageRequest> request) {
        auto start_time = chrono::high_resolution_clock::now();

        cout << "Frame " << request->frame_id << " image resolution: "
             << request->image.cols << " x " << request->image.rows << endl;
        
        bool success = false;
        try {
            lock_guard<mutex> slam_lock(slam_mutex_);
            
            Sophus::SE3f pose = slam_system_->TrackMonocular(
                request->image, 
                request->timestamp,
                vector<ORB_SLAM3::IMU::Point>(),
                "",
                request->intrinsics
            );
            
            success = !pose.matrix().isZero();
            if (success) {
                successful_tracks_++;
            } else {
                failed_tracks_++;
            }

        } catch (const exception& e) {
            cerr << "SLAM processing error: " << e.what() << endl;
            failed_tracks_++;
        }
        
        auto end_time = chrono::high_resolution_clock::now();
        const double processing_time =
            chrono::duration<double>(end_time - start_time).count();
        
        double current_avg = avg_processing_time_.load();
        const auto n = total_requests_.load();
        if (n > 0) {
            double new_avg = current_avg + (processing_time - current_avg) / n;
            avg_processing_time_.store(new_avg);
        }
    }
    
    chrono::steady_clock::time_point server_start_time_ = chrono::steady_clock::now();
};

static string exe_parent_dir(const char* argv0) {
    if (!argv0)
        return ".";
    string p(argv0);
    size_t pos = p.find_last_of("/\\");
    if (pos == string::npos)
        return ".";
    return p.substr(0, pos);
}

static string load_index_html_body(const char* argv0) {
    const string d = exe_parent_dir(argv0);
    vector<string> paths = {d + "/index.html", d + "\\index.html", string("Examples/Monocular/index.html")};
    for (const auto& path : paths) {
        ifstream f(path, ios::binary);
        if (f.good()) {
            string s((istreambuf_iterator<char>(f)), istreambuf_iterator<char>());
            if (!s.empty()) {
                cout << "Loaded stream UI from " << path << " (" << s.size() << " bytes)" << endl;
                return s;
            }
        }
    }
    cerr << "Warning: index.html not found; GET /index.html returns 404 (checked next to slam_server and Examples/Monocular/index.html)." << endl;
    return {};
}

// Plain function for signal(); must not capture (signal() needs a C function pointer).
static SLAMServer* g_active_server = nullptr;

static void signalStopListen(int /*sig*/) {
    if (g_active_server)
        g_active_server->requestStop();
}

void printUsage() {
    cout << endl << "Usage: ./slam_server path_to_vocabulary path_to_settings [options]" << endl;
    cout << "Options:" << endl;
    cout << "  --port <number>        Server port (default: 8080)" << endl;
    cout << "  --output-dir <path>    Output directory (default: ./output)" << endl;
    cout << "  --max-queue <number>   Maximum queue size (default: 10)" << endl;
    cout << "  --min-buffer <number>  Min frames in queue before tracking; oldest timestamp first (default: 3)" << endl;
    cout << "  --viewer               Enable SLAM viewer (default: disabled)" << endl;
    cout << "  --help                 Show this help message" << endl;
    cout << endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        printUsage();
        return 1;
    }
    
    ServerConfig config;
    config.vocabulary_path = argv[1];
    config.settings_path = argv[2];
    
    // Parse command line arguments
    for (int i = 3; i < argc; i++) {
        string arg = argv[i];
        
        if (arg == "--help") {
            printUsage();
            return 0;
        } else if (arg == "--port" && i + 1 < argc) {
            config.port = stoi(argv[++i]);
        } else if (arg == "--output-dir" && i + 1 < argc) {
            config.output_dir = argv[++i];
        } else if (arg == "--max-queue" && i + 1 < argc) {
            config.max_queue_size = stoi(argv[++i]);
        } else if (arg == "--min-buffer" && i + 1 < argc) {
            config.min_buffer_frames = static_cast<size_t>(stoul(argv[++i]));
        } else if (arg == "--viewer") {
            config.use_viewer = true;
        } else {
            cerr << "Unknown argument: " << arg << endl;
            printUsage();
            return 1;
        }
    }
    
    config.index_html_body = load_index_html_body(argv[0]);
    
    try {
        cout << "Initializing SLAM Server..." << endl;
        SLAMServer server(config);
        
        // Unblock listen() from a signal handler — do not run full shutdown() in the handler
        // (locks, joins, iostream are not async-signal-safe on POSIX).
        g_active_server = &server;
        signal(SIGINT, signalStopListen);
#ifndef _WIN32
#ifdef SIGTERM
        signal(SIGTERM, signalStopListen);
#endif
#endif

        server.start();

        cerr << "\nHTTP server stopped, shutting down SLAM..." << endl;
        server.shutdown();
        g_active_server = nullptr;
        
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}