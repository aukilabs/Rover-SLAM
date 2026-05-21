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
#include <cctype>
#include <cmath>
#include <limits>
#include <numeric>
#include <cerrno>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <direct.h>
#include <sys/stat.h>
#define usleep(x) Sleep((x)/1000)
#else
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>

#include <System.h>
#include <MapPoint.h>

// HTTP server library - cpp-httplib (header-only)
#include "../../include/third_party/httplib.h"

// JSON library - official nlohmann/json single-header
#include "../../include/third_party/nlohmann/json.hpp"

using namespace std;
using json = nlohmann::json;

namespace {

constexpr float kMinScale = 0.01f;
constexpr float kMaxScale = 100.0f;
constexpr int kMinLocalScalePoints = 2;
constexpr float kPortalRegionExpand = 0.35f;
constexpr float kMinSlamPairDistance = 1e-4f;
constexpr float kMinDomainPairDistance = 0.02f;
constexpr size_t kMaxPortalCornerObservations = 5000;

struct PortalDefinition {
    string short_id;
    Sophus::SE3f T_domain_portal;
    float physical_size_m = 0.0f;
};

struct PortalPnPResult {
    bool success = false;
    string short_id;
    string decoded_text;
    Sophus::SE3f T_cam_domain;
    Sophus::SE3f T_cam_portal;
    vector<cv::Point2f> image_points;
    vector<cv::Point3f> object_points;
    float physical_size_m = 0.0f;
    float mean_reproj_px = numeric_limits<float>::infinity();
};

struct LocalScaleEstimate {
    bool valid = false;
    float scale = 1.0f;
    int points_used = 0;
    string source = "none";
};

struct PortalObservation {
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    string short_id;
    Eigen::Vector3f p_slam = Eigen::Vector3f::Zero();
    Eigen::Vector3f p_domain = Eigen::Vector3f::Zero();
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    float reproj_px = 0.0f;
    float scale_prior = 1.0f;
    bool has_scale_prior = false;
    int local_points_used = 0;
};

struct PortalCornerObservation {
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    string short_id;
    float physical_size_m = 0.0f;
    vector<float> intrinsics;
    vector<cv::Point2f> image_points;
};

struct AlignmentSnapshot {
    bool enabled = false;
    bool has_transform = false;
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t_domain_slam = Eigen::Vector3f::Zero();
    float scale = 1.0f;
    string status = "disabled";
    string scale_source = "none";
    string last_portal_short_id;
    float last_reproj_px = 0.0f;
    int last_local_points_used = 0;
    size_t portal_observations = 0;
};

string trimCopy(const string& s)
{
    size_t first = 0;
    while (first < s.size() && isspace(static_cast<unsigned char>(s[first])))
        ++first;
    size_t last = s.size();
    while (last > first && isspace(static_cast<unsigned char>(s[last - 1])))
        --last;
    return s.substr(first, last - first);
}

string upperCopy(const string& s)
{
    string out = s;
    transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(toupper(c));
    });
    return out;
}

string lowerCopy(const string& s)
{
    string out = s;
    transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(tolower(c));
    });
    return out;
}

bool parsePortalShortId(const string& decoded, string& short_id)
{
    const string text = trimCopy(decoded);
    const string lower = lowerCopy(text);
    const string prefix = "https://r8.hr/";
    if (lower.rfind(prefix, 0) != 0)
        return false;

    string candidate = trimCopy(text.substr(prefix.size()));
    if (candidate.empty())
        return false;
    for (char c : candidate) {
        if (!isalnum(static_cast<unsigned char>(c)))
            return false;
    }
    short_id = upperCopy(candidate);
    return true;
}

bool requestParamIsFalse(const httplib::Request& req, const string& name)
{
    if (!req.has_param(name.c_str()))
        return false;
    const string v = lowerCopy(req.get_param_value(name.c_str()));
    return v == "0" || v == "false" || v == "no";
}

bool pathIsSeparator(char c)
{
    return c == '/' || c == '\\';
}

string trimTrailingPathSeparators(string path)
{
    while (path.size() > 1 && pathIsSeparator(path.back()))
        path.pop_back();
    return path;
}

bool directoryExists(const string& path)
{
#ifdef _WIN32
    struct _stat info;
    return _stat(path.c_str(), &info) == 0 && (info.st_mode & _S_IFDIR);
#else
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
#endif
}

bool createDirectory(const string& path)
{
#ifdef _WIN32
    if (_mkdir(path.c_str()) == 0)
        return true;
#else
    if (mkdir(path.c_str(), 0755) == 0)
        return true;
#endif
    return errno == EEXIST && directoryExists(path);
}

bool ensureDirectoryRecursive(const string& dir)
{
    const string path = trimTrailingPathSeparators(dir);
    if (path.empty() || directoryExists(path))
        return !path.empty();

    string current;
    size_t i = 0;
    if (path.size() >= 2 && path[1] == ':') {
        current = path.substr(0, 2);
        i = 2;
    } else if (pathIsSeparator(path[0])) {
        current = path.substr(0, 1);
        i = 1;
    }

    while (i < path.size()) {
        while (i < path.size() && pathIsSeparator(path[i]))
            ++i;
        const size_t next = path.find_first_of("/\\", i);
        const string part = path.substr(i, next == string::npos ? string::npos : next - i);
        if (!part.empty()) {
            if (!current.empty() && !pathIsSeparator(current.back()))
                current += '/';
            current += part;
            if (!directoryExists(current) && !createDirectory(current))
                return false;
        }
        if (next == string::npos)
            break;
        i = next + 1;
    }

    return directoryExists(path);
}

string joinPath(const string& dir, const string& filename)
{
    if (dir.empty())
        return filename;
    if (pathIsSeparator(dir.back()))
        return dir + filename;
    return dir + "/" + filename;
}

vector<float> loadDefaultIntrinsics(const string& settings_path)
{
    cv::FileStorage fs(settings_path, cv::FileStorage::READ);
    if (!fs.isOpened())
        return {};

    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    fs["Camera.fx"] >> fx;
    fs["Camera.fy"] >> fy;
    fs["Camera.cx"] >> cx;
    fs["Camera.cy"] >> cy;

    if (fx <= 0.0f || fy <= 0.0f)
        return {};
    return {fx, fy, cx, cy};
}

cv::Mat cameraMatrixFromIntrinsics(const vector<float>& intrinsics)
{
    return (cv::Mat_<double>(3, 3) << intrinsics[0], 0.0, intrinsics[2],
                                      0.0, intrinsics[1], intrinsics[3],
                                      0.0, 0.0, 1.0);
}

vector<cv::Point3f> portalObjectPoints(float qr_size_m)
{
    const float s = 0.5f * qr_size_m;
    return {
        cv::Point3f(-s, -s, 0.0f),
        cv::Point3f( s, -s, 0.0f),
        cv::Point3f( s,  s, 0.0f),
        cv::Point3f(-s,  s, 0.0f),
    };
}

float meanReprojectionError(const vector<cv::Point3f>& object_points,
                            const vector<cv::Point2f>& image_points,
                            const cv::Mat& rvec,
                            const cv::Mat& tvec,
                            const cv::Mat& K)
{
    vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, K, cv::Mat::zeros(5, 1, CV_64F), projected);
    if (projected.size() != image_points.size() || projected.empty())
        return numeric_limits<float>::infinity();

    double sum = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        const cv::Point2f d = projected[i] - image_points[i];
        sum += sqrt(static_cast<double>(d.x * d.x + d.y * d.y));
    }
    return static_cast<float>(sum / static_cast<double>(projected.size()));
}

bool readQrPoint(const cv::Mat& points, int detection_idx, int corner_idx, cv::Point2f& out)
{
    if (points.empty())
        return false;

    cv::Mat pts;
    points.convertTo(pts, CV_64F);
    const int channels = pts.channels();
    const size_t point_count = pts.total();
    if (channels < 2 || point_count < static_cast<size_t>((detection_idx + 1) * 4))
        return false;

    const double* data = pts.ptr<double>(0);
    const size_t offset = static_cast<size_t>(detection_idx * 4 + corner_idx) * channels;
    out.x = static_cast<float>(data[offset]);
    out.y = static_cast<float>(data[offset + 1]);
    return std::isfinite(out.x) && std::isfinite(out.y);
}

float medianFloat(vector<float> values)
{
    if (values.empty())
        return 1.0f;
    sort(values.begin(), values.end());
    const size_t mid = values.size() / 2;
    if ((values.size() % 2) == 1)
        return values[mid];
    return 0.5f * (values[mid - 1] + values[mid]);
}

bool isValidScale(float scale)
{
    return std::isfinite(scale) && scale >= kMinScale && scale <= kMaxScale;
}

Sophus::SE3f applyAlignment(const Sophus::SE3f& Twc, const AlignmentSnapshot& alignment)
{
    if (!alignment.enabled || !alignment.has_transform)
        return Twc;

    const Eigen::Matrix3f R = alignment.R_domain_slam * Twc.so3().matrix();
    const Eigen::Vector3f t =
        alignment.scale * (alignment.R_domain_slam * Twc.translation()) + alignment.t_domain_slam;
    return Sophus::SE3f(R, t);
}

json alignmentSnapshotToJson(const AlignmentSnapshot& alignment)
{
    json out;
    out["enabled"] = alignment.enabled;
    out["aligned"] = alignment.enabled && alignment.has_transform;
    out["status"] = alignment.status;
    out["scale"] = alignment.scale;
    out["scale_source"] = alignment.scale_source;
    out["portal_observations"] = alignment.portal_observations;
    out["last_portal_short_id"] = alignment.last_portal_short_id;
    out["last_reprojection_error_px"] = alignment.last_reproj_px;
    out["last_local_points_used"] = alignment.last_local_points_used;
    return out;
}

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
    string debug_output_dir;
    string portal_manifest_path;
    bool use_viewer = false;
    int max_queue_size = 10;
    size_t slam_interval_nth = 1;
    /** Wait until this many frames are buffered (unless shutting down); then process oldest by `timestamp`. */
    size_t min_buffer_frames = 3;
    size_t portal_detect_every = 1;
    float portal_max_reproj_px = 8.0f;
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
    atomic<uint64_t> skipped_tracks_{0};
    atomic<double> avg_processing_time_{0.0};
    string index_html_body_;
    atomic<bool> shutdown_done_{false};
    vector<float> default_intrinsics_;
    unordered_map<string, PortalDefinition> portals_;
    mutex alignment_mutex_;
    mutex debug_output_mutex_;
    vector<PortalObservation> portal_observations_;
    vector<PortalCornerObservation> portal_corner_observations_;
    AlignmentSnapshot alignment_;

public:
    SLAMServer(const ServerConfig& config) : config_(config), index_html_body_(config.index_html_body) {
        default_intrinsics_ = loadDefaultIntrinsics(config_.settings_path);
        if (default_intrinsics_.empty()) {
            cerr << "Warning: could not load default Camera.fx/fy/cx/cy from settings; "
                 << "portal PnP requires per-frame intrinsics." << endl;
        }

        if (!config_.portal_manifest_path.empty()) {
            loadPortalManifest(config_.portal_manifest_path);
        }

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
        publishViewerTruthPortalSquares();

        if (config_.min_buffer_frames < 1) {
            cerr << "Warning: min_buffer_frames must be >= 1; using 1." << endl;
            config_.min_buffer_frames = 1;
        }
        if (config_.slam_interval_nth < 1) {
            cerr << "Warning: slam_interval_nth must be >= 1; using 1." << endl;
            config_.slam_interval_nth = 1;
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
        cout << "SLAM frame interval: every " << config_.slam_interval_nth
             << " accepted frame(s)" << endl;
        if (!config_.debug_output_dir.empty())
            cout << "Debug trajectory output: "
                 << joinPath(config_.debug_output_dir, "trajectory_latest.json") << endl;
        if (!config_.portal_manifest_path.empty())
            cout << "Portal manifest: " << config_.portal_manifest_path
                 << " (" << portals_.size() << " usable portal(s))" << endl;
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
    void loadPortalManifest(const string& manifest_path) {
        ifstream f(manifest_path);
        if (!f.good()) {
            cerr << "Warning: portal manifest not found: " << manifest_path << endl;
            return;
        }

        json data;
        try {
            f >> data;
        } catch (const exception& e) {
            cerr << "Warning: failed to parse portal manifest '" << manifest_path << "': "
                 << e.what() << endl;
            return;
        }

        size_t skipped = 0;
        for (const auto& entry : data.value("portals", json::array())) {
            try {
                if (!entry.contains("shortId") || !entry.contains("pose") ||
                    !entry.contains("physicalSize") || entry["physicalSize"].is_null()) {
                    skipped++;
                    continue;
                }

                const float physical_size = entry["physicalSize"].get<float>();
                if (!std::isfinite(physical_size) || physical_size <= 0.0f) {
                    skipped++;
                    continue;
                }

                const auto& pose = entry["pose"];
                const auto& pos = pose["position"];
                const auto& rot = pose["rotation"];

                const Eigen::Vector3f t_domain_portal(
                    pos["y"].get<float>(),
                    pos["x"].get<float>(),
                    -pos["z"].get<float>());

                Eigen::Quaternionf q_domain_portal(
                    rot["w"].get<float>(),
                    rot["y"].get<float>(),
                    rot["x"].get<float>(),
                    -rot["z"].get<float>());
                q_domain_portal.normalize();

                PortalDefinition portal;
                portal.short_id = upperCopy(entry["shortId"].get<string>());
                portal.physical_size_m = physical_size;
                portal.T_domain_portal =
                    Sophus::SE3f(q_domain_portal.toRotationMatrix(), t_domain_portal);
                portals_[portal.short_id] = portal;
            } catch (const exception& e) {
                skipped++;
                cerr << "Warning: skipping invalid portal manifest entry: " << e.what() << endl;
            }
        }

        {
            lock_guard<mutex> lock(alignment_mutex_);
            alignment_.enabled = !portals_.empty();
            alignment_.status = portals_.empty()
                ? "portal_manifest_empty"
                : "portal_manifest_loaded_waiting_for_detection";
        }

        cout << "Loaded " << portals_.size() << " portal(s) from " << manifest_path;
        if (skipped > 0)
            cout << " (" << skipped << " skipped)";
        cout << endl;
    }

    bool shouldRunPortalDetection(uint64_t frame_id) const {
        return !portals_.empty() &&
               config_.portal_detect_every > 0 &&
               (frame_id % config_.portal_detect_every) == 0;
    }

    bool shouldRunSlamForAcceptedFrame(uint64_t accepted_frame_index) const {
        return config_.slam_interval_nth <= 1 ||
               ((accepted_frame_index - 1) % config_.slam_interval_nth) == 0;
    }

    void writeDebugTrajectoryResponse(const json& response) {
        if (config_.debug_output_dir.empty())
            return;

        lock_guard<mutex> lock(debug_output_mutex_);
        if (!ensureDirectoryRecursive(config_.debug_output_dir)) {
            cerr << "Warning: could not create debug output directory: "
                 << config_.debug_output_dir << endl;
            return;
        }

        const string path = joinPath(config_.debug_output_dir, "trajectory_latest.json");
        ofstream out(path, ios::binary | ios::trunc);
        if (!out.good()) {
            cerr << "Warning: could not open debug trajectory output: " << path << endl;
            return;
        }

        out << response.dump(2) << endl;
    }

    vector<float> intrinsicsForRequest(const shared_ptr<ImageRequest>& request) const {
        if (request && request->intrinsics.size() >= 4)
            return request->intrinsics;
        return default_intrinsics_;
    }

    bool solvePortalPnP(const cv::Mat& image,
                        const vector<float>& intrinsics,
                        PortalPnPResult& best_result,
                        vector<PortalPnPResult>* all_results = nullptr) const {
        if (portals_.empty() || image.empty() || intrinsics.size() < 4)
            return false;

        cv::QRCodeDetector detector;
        vector<cv::String> decoded_info;
        cv::Mat points;
        bool ok = false;
        try {
            ok = detector.detectAndDecodeMulti(image, decoded_info, points);
        } catch (const cv::Exception& e) {
            cerr << "[portal] QR detectAndDecodeMulti failed: " << e.what() << endl;
            return false;
        }
        if (!ok || decoded_info.empty() || points.empty())
            return false;

        const cv::Mat K = cameraMatrixFromIntrinsics(intrinsics);
        const cv::Mat dist = cv::Mat::zeros(5, 1, CV_64F);
        float best_error = numeric_limits<float>::infinity();

        for (size_t i = 0; i < decoded_info.size(); ++i) {
            string short_id;
            const string decoded = string(decoded_info[i]);
            if (!parsePortalShortId(decoded, short_id))
                continue;

            auto portal_it = portals_.find(short_id);
            if (portal_it == portals_.end())
                continue;
            const PortalDefinition& portal = portal_it->second;

            vector<cv::Point2f> image_points(4);
            bool have_points = true;
            for (int c = 0; c < 4; ++c) {
                if (!readQrPoint(points, static_cast<int>(i), c, image_points[c])) {
                    have_points = false;
                    break;
                }
            }
            if (!have_points)
                continue;

            const vector<cv::Point3f> object_points = portalObjectPoints(portal.physical_size_m);
            cv::Mat rvec;
            cv::Mat tvec;
            bool pnp_ok = false;
            try {
                pnp_ok = cv::solvePnP(
                    object_points, image_points, K, dist, rvec, tvec, false, cv::SOLVEPNP_SQPNP);
            } catch (const cv::Exception& e) {
                cerr << "[portal] solvePnP failed for " << short_id << ": " << e.what() << endl;
                continue;
            }
            if (!pnp_ok)
                continue;

            const float reproj_px = meanReprojectionError(object_points, image_points, rvec, tvec, K);
            if (!std::isfinite(reproj_px) || reproj_px > config_.portal_max_reproj_px) {
                continue;
            }

            cv::Mat R_cv;
            cv::Rodrigues(rvec, R_cv);
            Eigen::Matrix3f R_cam_portal;
            Eigen::Vector3f t_cam_portal;
            for (int r = 0; r < 3; ++r) {
                t_cam_portal[r] = static_cast<float>(tvec.at<double>(r));
                for (int c = 0; c < 3; ++c)
                    R_cam_portal(r, c) = static_cast<float>(R_cv.at<double>(r, c));
            }

            const Eigen::Matrix3f R_domain_portal = portal.T_domain_portal.so3().matrix();
            const Eigen::Vector3f t_domain_portal = portal.T_domain_portal.translation();
            const Eigen::Matrix3f R_cam_domain = R_cam_portal * R_domain_portal.transpose();
            const Eigen::Vector3f t_cam_domain =
                t_cam_portal - R_cam_domain * t_domain_portal;

            PortalPnPResult result;
            result.success = true;
            result.short_id = short_id;
            result.decoded_text = decoded;
            result.T_cam_domain = Sophus::SE3f(R_cam_domain, t_cam_domain);
            result.T_cam_portal = Sophus::SE3f(R_cam_portal, t_cam_portal);
            result.image_points = image_points;
            result.object_points = object_points;
            result.physical_size_m = portal.physical_size_m;
            result.mean_reproj_px = reproj_px;

            if (all_results)
                all_results->push_back(result);

            if (reproj_px >= best_error)
                continue;
            best_error = reproj_px;
            best_result = result;
        }

        return best_result.success;
    }

    void storePortalCornerObservations(const vector<PortalPnPResult>& pnp_results,
                                       uint64_t frame_id,
                                       double timestamp,
                                       const vector<float>& intrinsics) {
        if (pnp_results.empty() || intrinsics.size() < 4)
            return;

        lock_guard<mutex> lock(alignment_mutex_);
        for (const auto& pnp : pnp_results) {
            if (!pnp.success || pnp.image_points.size() != 4 ||
                !std::isfinite(pnp.physical_size_m) || pnp.physical_size_m <= 0.0f) {
                continue;
            }

            PortalCornerObservation obs;
            obs.frame_id = frame_id;
            obs.timestamp = timestamp;
            obs.short_id = pnp.short_id;
            obs.physical_size_m = pnp.physical_size_m;
            obs.intrinsics = intrinsics;
            obs.image_points = pnp.image_points;
            portal_corner_observations_.push_back(obs);
        }

        if (portal_corner_observations_.size() > kMaxPortalCornerObservations) {
            const size_t overflow =
                portal_corner_observations_.size() - kMaxPortalCornerObservations;
            portal_corner_observations_.erase(
                portal_corner_observations_.begin(),
                portal_corner_observations_.begin() + static_cast<ptrdiff_t>(overflow));
        }
    }

    vector<PortalCornerObservation> portalCornerObservationsSnapshot() {
        lock_guard<mutex> lock(alignment_mutex_);
        return portal_corner_observations_;
    }

    vector<ORB_SLAM3::MapDrawer::PortalSquare> truthPortalSquaresFromManifest() const {
        vector<ORB_SLAM3::MapDrawer::PortalSquare> squares;
        squares.reserve(portals_.size());
        for (const auto& kv : portals_) {
            const PortalDefinition& portal = kv.second;
            if (!std::isfinite(portal.physical_size_m) || portal.physical_size_m <= 0.0f)
                continue;

            ORB_SLAM3::MapDrawer::PortalSquare square;
            square.id = portal.short_id;
            const vector<cv::Point3f> object_points =
                portalObjectPoints(portal.physical_size_m);
            const Eigen::Matrix3f R_domain_portal =
                portal.T_domain_portal.so3().matrix();
            const Eigen::Vector3f t_domain_portal =
                portal.T_domain_portal.translation();
            for (size_t i = 0; i < object_points.size() && i < square.corners.size(); ++i) {
                const cv::Point3f& obj = object_points[i];
                const Eigen::Vector3f p_portal(obj.x, obj.y, obj.z);
                const Eigen::Vector3f p_domain =
                    R_domain_portal * p_portal + t_domain_portal;
                square.corners[i] = {p_domain.x(), p_domain.y(), p_domain.z()};
            }
            squares.push_back(square);
        }
        return squares;
    }

    void publishViewerTruthPortalSquares() {
        if (!slam_system_)
            return;
        slam_system_->SetViewerTruthPortalSquares(truthPortalSquaresFromManifest());
    }

    vector<ORB_SLAM3::MapDrawer::PortalSquare> computePortalSquares(
        const vector<PortalCornerObservation>& observations,
        const unordered_map<uint64_t, Sophus::SE3f>& Twc_raw_by_frame,
        const AlignmentSnapshot& alignment) const {
        struct SquareAccum {
            std::array<Eigen::Vector3f, 4> sums;
            std::array<int, 4> counts;

            SquareAccum() {
                for (size_t i = 0; i < 4; ++i) {
                    sums[i] = Eigen::Vector3f::Zero();
                    counts[i] = 0;
                }
            }
        };

        unordered_map<string, SquareAccum> accums;
        vector<string> order;
        const float metric_to_slam =
            (alignment.has_transform && isValidScale(alignment.scale))
                ? 1.0f / alignment.scale
                : 1.0f;

        for (const auto& obs : observations) {
            if (obs.image_points.size() != 4 || obs.intrinsics.size() < 4)
                continue;

            const auto pose_it = Twc_raw_by_frame.find(obs.frame_id);
            if (pose_it == Twc_raw_by_frame.end())
                continue;

            const vector<cv::Point3f> object_points = portalObjectPoints(obs.physical_size_m);
            cv::Mat rvec;
            cv::Mat tvec;
            bool pnp_ok = false;
            try {
                pnp_ok = cv::solvePnP(
                    object_points,
                    obs.image_points,
                    cameraMatrixFromIntrinsics(obs.intrinsics),
                    cv::Mat::zeros(5, 1, CV_64F),
                    rvec,
                    tvec,
                    false,
                    cv::SOLVEPNP_SQPNP);
            } catch (const cv::Exception& e) {
                cerr << "[portal] placement solvePnP failed for "
                     << obs.short_id << " frame " << obs.frame_id << ": "
                     << e.what() << endl;
                continue;
            }
            if (!pnp_ok)
                continue;

            cv::Mat R_cv;
            cv::Rodrigues(rvec, R_cv);
            Eigen::Matrix3f R_cam_portal;
            Eigen::Vector3f t_cam_portal;
            for (int r = 0; r < 3; ++r) {
                t_cam_portal[r] = static_cast<float>(tvec.at<double>(r));
                for (int c = 0; c < 3; ++c)
                    R_cam_portal(r, c) = static_cast<float>(R_cv.at<double>(r, c));
            }

            const Sophus::SE3f& Twc_slam = pose_it->second;
            if (accums.find(obs.short_id) == accums.end()) {
                accums[obs.short_id] = SquareAccum();
                order.push_back(obs.short_id);
            }
            SquareAccum& accum = accums[obs.short_id];
            for (size_t i = 0; i < object_points.size(); ++i) {
                const cv::Point3f& obj = object_points[i];
                const Eigen::Vector3f p_portal(obj.x, obj.y, obj.z);
                const Eigen::Vector3f p_cam = R_cam_portal * p_portal + t_cam_portal;
                const Eigen::Vector3f p_slam =
                    Twc_slam.so3().matrix() * (metric_to_slam * p_cam) +
                    Twc_slam.translation();
                const Eigen::Vector3f p_out =
                    (alignment.enabled && alignment.has_transform)
                        ? alignment.scale * (alignment.R_domain_slam * p_slam) +
                              alignment.t_domain_slam
                        : p_slam;
                accum.sums[i] += p_out;
                accum.counts[i] += 1;
            }
        }

        vector<ORB_SLAM3::MapDrawer::PortalSquare> squares;
        for (const string& id : order) {
            const SquareAccum& accum = accums[id];
            ORB_SLAM3::MapDrawer::PortalSquare square;
            square.id = id;
            bool complete = true;
            for (size_t i = 0; i < 4; ++i) {
                if (accum.counts[i] <= 0) {
                    complete = false;
                    break;
                }
                const Eigen::Vector3f p = accum.sums[i] / static_cast<float>(accum.counts[i]);
                square.corners[i] = {p.x(), p.y(), p.z()};
            }
            if (complete)
                squares.push_back(square);
        }

        return squares;
    }

    void refreshViewerPortalSquares() {
        if (!slam_system_)
            return;

        vector<PortalCornerObservation> observations;
        AlignmentSnapshot alignment;
        {
            lock_guard<mutex> lock(alignment_mutex_);
            observations = portal_corner_observations_;
            alignment = alignment_;
        }
        if (observations.empty()) {
            slam_system_->ClearViewerPortalSquares();
            return;
        }

        vector<ORB_SLAM3::System::TrajectorySample> samples;
        {
            lock_guard<mutex> slam_lock(slam_mutex_);
            slam_system_->GetActiveMapDenseTrajectory(samples);
            if (samples.empty())
                slam_system_->GetActiveMapTrajectory(samples, true);
        }

        unordered_map<uint64_t, Sophus::SE3f> Twc_raw_by_frame;
        for (const auto& sample : samples)
            Twc_raw_by_frame[sample.id] = sample.Twc;
        if (Twc_raw_by_frame.empty()) {
            slam_system_->ClearViewerPortalSquares();
            return;
        }

        slam_system_->SetViewerPortalSquares(
            computePortalSquares(observations, Twc_raw_by_frame, alignment));
    }

    json portalFrameDetectionsToJson(const vector<PortalCornerObservation>& observations) const {
        json rows = json::array();
        for (const auto& obs : observations) {
            if (obs.image_points.size() != 4)
                continue;
            for (size_t i = 0; i < obs.image_points.size(); ++i) {
                const cv::Point2f& p = obs.image_points[i];
                rows.push_back({
                    {"frame_index", obs.frame_id},
                    {"type", "portal_corner"},
                    {"id", obs.short_id + "_" + to_string(i)},
                    {"pixel", {p.x, p.y}},
                });
            }
        }
        return rows;
    }

    json placedDataToJson(const vector<ORB_SLAM3::MapDrawer::PortalSquare>& squares) const {
        json rows = json::array();
        for (const auto& square : squares) {
            for (size_t i = 0; i < square.corners.size(); ++i) {
                const auto& p = square.corners[i];
                rows.push_back({
                    {"type", "portal_corner"},
                    {"id", square.id + "_" + to_string(i)},
                    {"point", {p[0], p[1], p[2]}},
                });
            }
        }
        return rows;
    }

    LocalScaleEstimate estimateLocalScaleFromPortalRegion(
        const PortalPnPResult& pnp,
        const Sophus::SE3f& Tcw_slam,
        const vector<ORB_SLAM3::MapPoint*>& map_points,
        const vector<cv::KeyPoint>& keypoints,
        const vector<float>& intrinsics) const {
        LocalScaleEstimate estimate;
        if (!pnp.success || intrinsics.size() < 4 ||
            map_points.empty() || keypoints.empty() || map_points.size() != keypoints.size()) {
            return estimate;
        }

        float min_x = numeric_limits<float>::max();
        float min_y = numeric_limits<float>::max();
        float max_x = -numeric_limits<float>::max();
        float max_y = -numeric_limits<float>::max();
        for (const auto& p : pnp.image_points) {
            min_x = min(min_x, p.x);
            min_y = min(min_y, p.y);
            max_x = max(max_x, p.x);
            max_y = max(max_y, p.y);
        }
        const float width = max_x - min_x;
        const float height = max_y - min_y;
        const float expand = kPortalRegionExpand * max(width, height);
        min_x -= expand;
        min_y -= expand;
        max_x += expand;
        max_y += expand;

        const Eigen::Matrix3f R_cam_portal = pnp.T_cam_portal.so3().matrix();
        const Eigen::Vector3f t_cam_portal = pnp.T_cam_portal.translation();
        const Eigen::Vector3f portal_normal_cam = R_cam_portal * Eigen::Vector3f(0.0f, 0.0f, 1.0f);

        vector<Eigen::Vector3f> slam_cam_points;
        vector<Eigen::Vector3f> metric_cam_points;
        for (size_t i = 0; i < map_points.size(); ++i) {
            ORB_SLAM3::MapPoint* mp = map_points[i];
            if (!mp || mp->isBad())
                continue;

            const cv::Point2f uv = keypoints[i].pt;
            if (uv.x < min_x || uv.x > max_x || uv.y < min_y || uv.y > max_y)
                continue;

            const Eigen::Vector3f slam_world = mp->GetWorldPos();
            const Eigen::Vector3f slam_cam =
                Tcw_slam.so3().matrix() * slam_world + Tcw_slam.translation();
            if (!slam_cam.allFinite() || slam_cam.z() <= 0.0f)
                continue;

            const Eigen::Vector3f ray(
                (uv.x - intrinsics[2]) / intrinsics[0],
                (uv.y - intrinsics[3]) / intrinsics[1],
                1.0f);
            const float denom = portal_normal_cam.dot(ray);
            if (std::fabs(denom) < 1e-6f)
                continue;
            const float lambda = portal_normal_cam.dot(t_cam_portal) / denom;
            if (!std::isfinite(lambda) || lambda <= 0.0f)
                continue;

            slam_cam_points.push_back(slam_cam);
            metric_cam_points.push_back(lambda * ray);
        }

        vector<float> ratios;
        for (size_t i = 0; i < slam_cam_points.size(); ++i) {
            for (size_t j = i + 1; j < slam_cam_points.size(); ++j) {
                const float slam_dist = (slam_cam_points[i] - slam_cam_points[j]).norm();
                const float metric_dist = (metric_cam_points[i] - metric_cam_points[j]).norm();
                if (slam_dist > kMinSlamPairDistance && metric_dist > 1e-4f)
                    ratios.push_back(metric_dist / slam_dist);
            }
        }

        if (ratios.empty() && slam_cam_points.size() == 1) {
            const float slam_dist = slam_cam_points.front().norm();
            const float metric_dist = metric_cam_points.front().norm();
            if (slam_dist > kMinSlamPairDistance && metric_dist > 1e-4f)
                ratios.push_back(metric_dist / slam_dist);
        }

        if (ratios.empty())
            return estimate;

        const float scale = medianFloat(ratios);
        if (!isValidScale(scale))
            return estimate;

        estimate.valid = true;
        estimate.scale = scale;
        estimate.points_used = static_cast<int>(slam_cam_points.size());
        estimate.source = estimate.points_used >= kMinLocalScalePoints
            ? "single_portal_local_points"
            : "single_portal_local_depth";
        return estimate;
    }

    void updatePortalAlignment(const PortalPnPResult& pnp,
                               const Sophus::SE3f& Tcw_slam,
                               const LocalScaleEstimate& local_scale,
                               uint64_t frame_id,
                               double timestamp) {
        const Sophus::SE3f Twc_slam = Tcw_slam.inverse();
        const Sophus::SE3f Twc_domain = pnp.T_cam_domain.inverse();

        PortalObservation obs;
        obs.frame_id = frame_id;
        obs.timestamp = timestamp;
        obs.short_id = pnp.short_id;
        obs.p_slam = Twc_slam.translation();
        obs.p_domain = Twc_domain.translation();
        obs.R_domain_slam = Twc_domain.so3().matrix() * Twc_slam.so3().matrix().transpose();
        obs.reproj_px = pnp.mean_reproj_px;
        obs.has_scale_prior = local_scale.valid;
        obs.scale_prior = local_scale.scale;
        obs.local_points_used = local_scale.points_used;

        bool publish_viewer_alignment = false;
        Eigen::Matrix3f viewer_R_domain_slam = Eigen::Matrix3f::Identity();
        Eigen::Vector3f viewer_t_domain_slam = Eigen::Vector3f::Zero();
        float viewer_scale = 1.0f;
        {
            lock_guard<mutex> lock(alignment_mutex_);
            portal_observations_.push_back(obs);
            if (portal_observations_.size() > 64)
                portal_observations_.erase(portal_observations_.begin());

            vector<float> baseline_scales;
            vector<float> prior_scales;
            for (const auto& o : portal_observations_) {
                if (o.has_scale_prior && isValidScale(o.scale_prior))
                    prior_scales.push_back(o.scale_prior);
            }

            for (size_t i = 0; i < portal_observations_.size(); ++i) {
                for (size_t j = i + 1; j < portal_observations_.size(); ++j) {
                    const float slam_dist =
                        (portal_observations_[i].p_slam - portal_observations_[j].p_slam).norm();
                    const float domain_dist =
                        (portal_observations_[i].p_domain - portal_observations_[j].p_domain).norm();
                    if (slam_dist > kMinSlamPairDistance && domain_dist > kMinDomainPairDistance)
                        baseline_scales.push_back(domain_dist / slam_dist);
                }
            }

            string scale_source;
            float scale = alignment_.has_transform ? alignment_.scale : 1.0f;
            if (!baseline_scales.empty()) {
                scale = medianFloat(baseline_scales);
                scale_source = "portal_camera_baseline";
            } else if (!prior_scales.empty()) {
                scale = medianFloat(prior_scales);
                scale_source = local_scale.source.empty() ? "single_portal_local_points" : local_scale.source;
            } else {
                scale_source = "unit_fallback";
            }
            if (!isValidScale(scale)) {
                scale = 1.0f;
                scale_source = "unit_fallback";
            }

            alignment_.enabled = true;
            alignment_.has_transform = true;
            alignment_.R_domain_slam = obs.R_domain_slam;
            alignment_.scale = scale;
            alignment_.t_domain_slam = obs.p_domain - scale * (obs.R_domain_slam * obs.p_slam);
            alignment_.scale_source = scale_source;
            alignment_.last_portal_short_id = obs.short_id;
            alignment_.last_reproj_px = obs.reproj_px;
            alignment_.last_local_points_used = obs.local_points_used;
            alignment_.portal_observations = portal_observations_.size();
            alignment_.status = scale_source == "unit_fallback"
                ? "aligned_unit_scale_fallback"
                : "aligned";

            publish_viewer_alignment = alignment_.enabled && alignment_.has_transform;
            viewer_R_domain_slam = alignment_.R_domain_slam;
            viewer_t_domain_slam = alignment_.t_domain_slam;
            viewer_scale = alignment_.scale;

            cout << "[portal] aligned via " << obs.short_id
                 << " reproj_px=" << obs.reproj_px
                 << " scale=" << alignment_.scale
                 << " scale_source=" << alignment_.scale_source
                 << " observations=" << alignment_.portal_observations << endl;
        }

        if (publish_viewer_alignment && slam_system_) {
            slam_system_->SetViewerAlignment(
                viewer_R_domain_slam, viewer_t_domain_slam, viewer_scale);
        }
    }

    AlignmentSnapshot getAlignmentSnapshot(bool request_aligned) {
        lock_guard<mutex> lock(alignment_mutex_);
        AlignmentSnapshot snapshot = alignment_;
        if (!request_aligned) {
            snapshot.enabled = false;
            snapshot.has_transform = false;
            snapshot.status = portals_.empty() ? "disabled" : "raw_requested";
        }
        return snapshot;
    }

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

            const uint64_t accepted_frame_index = total_requests_.load() + 1;
            if (!shouldRunSlamForAcceptedFrame(accepted_frame_index)) {
                total_requests_++;
                skipped_tracks_++;

                response["success"] = true;
                response["message"] = "Image accepted but skipped by SLAM frame interval";
                response["frame_id"] = image_req->frame_id;
                response["timestamp"] = image_req->timestamp;
                response["tracked"] = false;
                response["slam_interval_nth"] = config_.slam_interval_nth;

                res.set_content(response.dump(), "application/json");
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
            response["tracked"] = true;
            
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
        response["skipped_tracks"] = skipped_tracks_.load();
        response["slam_interval_nth"] = config_.slam_interval_nth;
        response["avg_processing_time_ms"] = avg_processing_time_.load() * 1000.0;
        
        if (slam_system_) {
            response["tracking_state"] = slam_system_->GetTrackingState();
            response["is_lost"] = slam_system_->isLost();
        }

        response["alignment"] = alignmentSnapshotToJson(getAlignmentSnapshot(true));
        
        res.set_content(response.dump(), "application/json");
    }
    
    void handleTrajectoryRequest(const httplib::Request& req, httplib::Response& res) {
        json response;
        json trajectory_json = json::array();

        const bool dense = trajectoryUseDense(req);
        const bool request_aligned = !requestParamIsFalse(req, "aligned");
        const AlignmentSnapshot alignment = getAlignmentSnapshot(request_aligned);

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

        unordered_map<uint64_t, Sophus::SE3f> Twc_raw_by_frame;
        for (const auto& sample : samples) {
            const Sophus::SE3f Twc = applyAlignment(sample.Twc, alignment);
            Twc_raw_by_frame[sample.id] = sample.Twc;
            trajectory_json.push_back(
                TwcToClientJson(Twc, sample.timestamp, sample.id, sample.is_keyframe));
        }

        const vector<PortalCornerObservation> portal_corner_observations =
            portalCornerObservationsSnapshot();
        const vector<ORB_SLAM3::MapDrawer::PortalSquare> portal_squares =
            computePortalSquares(portal_corner_observations, Twc_raw_by_frame, alignment);

        response["trajectory"] = trajectory_json;
        response["count"] = trajectory_json.size();
        response["placed_data"] = placedDataToJson(portal_squares);
        response["frame_detections"] = portalFrameDetectionsToJson(portal_corner_observations);
        response["map_id"] = current_map_id;
        response["dense"] = dense;
        response["source"] = dense ? "active_map_dense_frames" : "active_map_keyframes";
        response["coordinate_frame"] =
            (alignment.enabled && alignment.has_transform) ? "domain_colmap" : "slam";
        response["alignment"] = alignmentSnapshotToJson(alignment);

        if (request_aligned && slam_system_)
            slam_system_->SetViewerPortalSquares(portal_squares);
        writeDebugTrajectoryResponse(response);
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
            {"GET /api/v1/trajectory", "Active-map trajectory (dense default; ?dense=0 keyframes only; ?aligned=0 raw SLAM frame)"},
            {"GET /api/v1/health", "Health check"}
        };
        response["portal_alignment_enabled"] = !portals_.empty();
        
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
        Sophus::SE3f pose;
        vector<ORB_SLAM3::MapPoint*> tracked_map_points;
        vector<cv::KeyPoint> tracked_keypoints;
        vector<float> portal_intrinsics;
        try {
            {
                lock_guard<mutex> slam_lock(slam_mutex_);

                pose = slam_system_->TrackMonocular(
                    request->image,
                    request->timestamp,
                    vector<ORB_SLAM3::IMU::Point>(),
                    "",
                    request->intrinsics
                );

                success = !pose.matrix().isZero();
                if (success) {
                    successful_tracks_++;
                    if (shouldRunPortalDetection(request->frame_id)) {
                        tracked_map_points = slam_system_->GetTrackedMapPoints();
                        tracked_keypoints = slam_system_->GetTrackedKeyPointsUn();
                        portal_intrinsics = intrinsicsForRequest(request);
                    }
                } else {
                    failed_tracks_++;
                }
            }

            if (success && shouldRunPortalDetection(request->frame_id)) {
                if (portal_intrinsics.size() < 4) {
                    cerr << "[portal] skipping QR PnP for frame " << request->frame_id
                         << ": no intrinsics available" << endl;
                } else {
                    PortalPnPResult pnp;
                    vector<PortalPnPResult> portal_detections;
                    if (solvePortalPnP(request->image, portal_intrinsics, pnp, &portal_detections)) {
                        storePortalCornerObservations(
                            portal_detections,
                            request->frame_id,
                            request->timestamp,
                            portal_intrinsics);
                        const LocalScaleEstimate local_scale = estimateLocalScaleFromPortalRegion(
                            pnp, pose, tracked_map_points, tracked_keypoints, portal_intrinsics);
                        updatePortalAlignment(
                            pnp, pose, local_scale, request->frame_id, request->timestamp);
                        refreshViewerPortalSquares();
                    }
                }
            }

        } catch (const exception& e) {
            cerr << "SLAM processing error: " << e.what() << endl;
            failed_tracks_++;
        }
        
        auto end_time = chrono::high_resolution_clock::now();
        const double processing_time =
            chrono::duration<double>(end_time - start_time).count();
        
        double current_avg = avg_processing_time_.load();
        const auto n = successful_tracks_.load() + failed_tracks_.load();
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
    cout << "  --debug-output <path>  Directory for latest trajectory JSON debug file (default: disabled)" << endl;
    cout << "  --max-queue <number>   Maximum queue size (default: 10)" << endl;
    cout << "  --slam-interval-nth <N> Run SLAM on one out of every N accepted frames (default: 1)" << endl;
    cout << "  --min-buffer <number>  Min frames in queue before tracking; oldest timestamp first (default: 3)" << endl;
    cout << "  --portal-manifest <path>  refined_manifest.json with portals[].pose and physicalSize" << endl;
    cout << "  --portal-detect-every <N> Run portal QR detection every N accepted frames (default: 1)" << endl;
    cout << "  --portal-max-reproj-px <N> Reject portal PnP above this mean reprojection error (default: 8)" << endl;
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
        } else if (arg == "--debug-output" && i + 1 < argc) {
            config.debug_output_dir = argv[++i];
        } else if (arg == "--max-queue" && i + 1 < argc) {
            config.max_queue_size = stoi(argv[++i]);
        } else if (arg == "--slam-interval-nth" && i + 1 < argc) {
            config.slam_interval_nth = static_cast<size_t>(stoul(argv[++i]));
            if (config.slam_interval_nth < 1)
                config.slam_interval_nth = 1;
        } else if (arg == "--min-buffer" && i + 1 < argc) {
            config.min_buffer_frames = static_cast<size_t>(stoul(argv[++i]));
        } else if (arg == "--portal-manifest" && i + 1 < argc) {
            config.portal_manifest_path = argv[++i];
        } else if (arg == "--portal-detect-every" && i + 1 < argc) {
            config.portal_detect_every = static_cast<size_t>(stoul(argv[++i]));
            if (config.portal_detect_every < 1)
                config.portal_detect_every = 1;
        } else if (arg == "--portal-max-reproj-px" && i + 1 < argc) {
            config.portal_max_reproj_px = stof(argv[++i]);
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