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
#include <iomanip>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <array>
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
#include <cstdint>
#include <cstring>
#include <cstdlib>

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
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>

#ifdef SLAM_SERVER_HAVE_LIBHEIF
#include <libheif/heif.h>
#endif

#ifdef SLAM_SERVER_ZXING_SYSTEM_HEADERS
#include <ZXing/BarcodeFormat.h>
#include <ZXing/ReadBarcode.h>
#else
#include <BarcodeFormat.h>
#include <ReadBarcode.h>
#endif

#include <System.h>
#include <MapPoint.h>

// HTTP server library - cpp-httplib (header-only)
#include "../../include/third_party/httplib.h"

// JSON library - official nlohmann/json single-header
#include "../../include/third_party/nlohmann/json.hpp"

#include "prior_pack_loader.h"
#include "overlay_pose_graph.h"
#include "prior_soft_edges.h"

using namespace std;
using json = nlohmann::json;

namespace {

constexpr float kMinScale = 0.01f;
constexpr float kMaxScale = 100.0f;
constexpr int kMinLocalScalePoints = 2;
constexpr float kPortalRegionExpand = 0.35f;
// Min SLAM separation for a camera-baseline ratio. Tiny slam motion with noisy
// PnP domain motion produces absurd ratios (e.g. slam_dist=0.01 → ratio≈8).
constexpr float kMinSlamBaselinePairDistance = 0.05f;
// Local map-point scale pairs can be denser near the QR.
constexpr float kMinSlamLocalPointPairDistance = 1e-4f;
constexpr float kMinScaleBaselineDistanceM = 0.10f;
// Scale-update gate: require SLAM motion since last accepted scale. Do NOT
// require domain motion — at correct scale, domain moves ~s×slam; requiring
// 10 cm domain blocked updates while slam moved ~30 cm (held_stationary).
constexpr float kMinScaleUpdateSlamDistance = 0.05f;
// Collapse portal obs that share nearly the same slam pose (e.g. first N
// frames stuck at origin) so pairwise ratios are not N× the same baseline.
constexpr float kSlamPoseClusterEps = 0.02f;
constexpr size_t kMaxPortalAlignmentObservations = 64;
constexpr size_t kMaxPortalCornerObservations = 5000;
constexpr size_t kMaxCustomDetectionObservations = 10000;
constexpr size_t kMaxCustomDetectionQueue = 2;
constexpr int kMaxBarcodeResultsPerTrial = 64;
constexpr float kBarcodeDuplicateCenterTolerancePx = 32.0f;
constexpr float kPortalAlignmentSmoothingAlpha = 0.20f;
constexpr float kPortalScaleSmoothingAlpha = 0.20f;
constexpr float kRadiansToDegrees = 57.29577951308232f;

double elapsedMs(chrono::high_resolution_clock::time_point start,
                 chrono::high_resolution_clock::time_point end)
{
    return chrono::duration<double, milli>(end - start).count();
}

string formatKiB(size_t bytes)
{
    ostringstream oss;
    oss << fixed << setprecision(1)
        << (static_cast<double>(bytes) / 1024.0);
    return oss.str();
}

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

struct CustomDetection {
    string type;
    string id;
    cv::Point2f pixel;
    cv::Rect bounds;
};

struct CustomDetectionObservation {
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    bool has_capture_timestamp = false;
    string type;
    string id;
    vector<float> intrinsics;
    cv::Point2f pixel;
};

struct AlignmentSnapshot {
    bool enabled = false;
    bool has_transform = false;
    Eigen::Matrix3f R_domain_slam = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t_domain_slam = Eigen::Vector3f::Zero();
    float scale = 1.0f;
    bool has_scale_update_observation = false;
    Eigen::Vector3f last_scale_update_p_slam = Eigen::Vector3f::Zero();
    Eigen::Vector3f last_scale_update_p_domain = Eigen::Vector3f::Zero();
    string status = "disabled";
    string scale_source = "none";
    string last_portal_short_id;
    float last_reproj_px = 0.0f;
    int last_local_points_used = 0;
    size_t portal_observations = 0;
};

struct LoadedObjMesh {
    string path;
    string obj_content;
    vector<ORB_SLAM3::MapDrawer::MeshTriangle> triangles;

    bool hasTriangles() const {
        return !triangles.empty();
    }
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

bool readTextFile(const string& path, string& contents, string& error)
{
    ifstream in(path, ios::binary);
    if (!in.good()) {
        error = "could not open " + path;
        return false;
    }

    ostringstream ss;
    ss << in.rdbuf();
    contents = ss.str();
    return true;
}

bool parseObjFaceIndex(const string& token, size_t vertex_count, size_t& out)
{
    const size_t slash = token.find('/');
    const string index_text = slash == string::npos ? token : token.substr(0, slash);
    if (index_text.empty())
        return false;

    long index = 0;
    try {
        index = stol(index_text);
    } catch (const exception&) {
        return false;
    }

    long resolved = index > 0
        ? index - 1
        : static_cast<long>(vertex_count) + index;
    if (resolved < 0 || resolved >= static_cast<long>(vertex_count))
        return false;

    out = static_cast<size_t>(resolved);
    return true;
}

array<float, 3> manifestPointToDomainColmap(float x, float y, float z)
{
    return {y, x, -z};
}

bool loadObjMesh(const string& path, LoadedObjMesh& mesh, string& error)
{
    LoadedObjMesh parsed;
    parsed.path = path;
    if (!readTextFile(path, parsed.obj_content, error))
        return false;

    vector<array<float, 3>> vertices;
    istringstream obj(parsed.obj_content);
    ostringstream converted_obj;
    converted_obj << setprecision(9);
    string line;
    while (getline(obj, line)) {
        const string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            converted_obj << line << '\n';
            continue;
        }

        istringstream ls(trimmed);
        string tag;
        ls >> tag;
        if (tag == "v") {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if (ls >> x >> y >> z) {
                if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                    const array<float, 3> converted =
                        manifestPointToDomainColmap(x, y, z);
                    vertices.push_back(converted);
                    converted_obj << "v " << converted[0] << " "
                                  << converted[1] << " "
                                  << converted[2] << '\n';
                } else {
                    converted_obj << line << '\n';
                }
            } else {
                converted_obj << line << '\n';
            }
        } else if (tag == "f") {
            vector<size_t> face;
            string token;
            while (ls >> token) {
                size_t vertex_index = 0;
                if (parseObjFaceIndex(token, vertices.size(), vertex_index))
                    face.push_back(vertex_index);
            }
            if (face.size() < 3) {
                converted_obj << line << '\n';
                continue;
            }

            for (size_t i = 1; i + 1 < face.size(); ++i) {
                ORB_SLAM3::MapDrawer::MeshTriangle triangle;
                triangle.vertices[0] = vertices[face[0]];
                triangle.vertices[1] = vertices[face[i]];
                triangle.vertices[2] = vertices[face[i + 1]];
                parsed.triangles.push_back(triangle);
            }
            converted_obj << line << '\n';
        } else {
            converted_obj << line << '\n';
        }
    }

    parsed.obj_content = converted_obj.str();
    mesh = parsed;
    return true;
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
    if (fx <= 0.0f || fy <= 0.0f) {
        fs["Camera1.fx"] >> fx;
        fs["Camera1.fy"] >> fy;
        fs["Camera1.cx"] >> cx;
        fs["Camera1.cy"] >> cy;
    }

    if (fx <= 0.0f || fy <= 0.0f)
        return {};
    return {fx, fy, cx, cy};
}

cv::Size loadConfiguredInputSize(const string& settings_path)
{
    cv::FileStorage fs(settings_path, cv::FileStorage::READ);
    if (!fs.isOpened())
        return {};

    int width = 0;
    int height = 0;
    fs["Camera.width"] >> width;
    fs["Camera.height"] >> height;
    if (width <= 0 || height <= 0)
        return {};
    return cv::Size(width, height);
}

/** ALIKED keypoints/descriptors live in Settings newImSize when Camera.newWidth/Height set. */
cv::Size loadFeatureImageSize(const string& settings_path)
{
    cv::FileStorage fs(settings_path, cv::FileStorage::READ);
    if (!fs.isOpened())
        return {};

    int new_width = 0;
    int new_height = 0;
    fs["Camera.newWidth"] >> new_width;
    fs["Camera.newHeight"] >> new_height;
    if (new_width > 0 && new_height > 0)
        return cv::Size(new_width, new_height);
    return loadConfiguredInputSize(settings_path);
}

string loadPriorPackPathFromSettings(const string& settings_path)
{
    cv::FileStorage fs(settings_path, cv::FileStorage::READ);
    if (!fs.isOpened())
        return {};

    string path;
    fs["Server.priorPack"] >> path;
    if (path.empty())
        fs["prior_pack"] >> path;
    return path;
}

bool isValidImageSize(const cv::Size& size)
{
    return size.width > 0 && size.height > 0;
}

bool sameImageSize(const cv::Size& a, const cv::Size& b)
{
    return a.width == b.width && a.height == b.height;
}

vector<float> scaleIntrinsicsToSize(const vector<float>& intrinsics,
                                    const cv::Size& from_size,
                                    const cv::Size& to_size)
{
    if (intrinsics.size() < 4 || !isValidImageSize(from_size) || !isValidImageSize(to_size))
        return intrinsics;

    const float sx = static_cast<float>(to_size.width) / static_cast<float>(from_size.width);
    const float sy = static_cast<float>(to_size.height) / static_cast<float>(from_size.height);
    return {
        intrinsics[0] * sx,
        intrinsics[1] * sy,
        intrinsics[2] * sx,
        intrinsics[3] * sy,
    };
}

bool hasRunDetectorHeader(const httplib::Request& req)
{
    return req.has_header("X-Run-Detector") || req.has_header("X-run-detector");
}

bool contentTypeIsHeif(const string& content_type)
{
    const string lower = lowerCopy(content_type);
    return lower.find("image/heic") != string::npos ||
           lower.find("image/heif") != string::npos;
}

bool isHeifBrand(const unsigned char* brand)
{
    return memcmp(brand, "heic", 4) == 0 ||
           memcmp(brand, "heix", 4) == 0 ||
           memcmp(brand, "hevc", 4) == 0 ||
           memcmp(brand, "hevx", 4) == 0 ||
           memcmp(brand, "heim", 4) == 0 ||
           memcmp(brand, "heis", 4) == 0 ||
           memcmp(brand, "hevm", 4) == 0 ||
           memcmp(brand, "hevs", 4) == 0 ||
           memcmp(brand, "mif1", 4) == 0 ||
           memcmp(brand, "msf1", 4) == 0;
}

bool dataLooksLikeHeif(const vector<uchar>& data)
{
    if (data.size() < 12 || memcmp(data.data() + 4, "ftyp", 4) != 0)
        return false;

    if (isHeifBrand(data.data() + 8))
        return true;

    const size_t scan_limit = min(data.size(), static_cast<size_t>(128));
    for (size_t offset = 16; offset + 4 <= scan_limit; offset += 4) {
        if (isHeifBrand(data.data() + offset))
            return true;
    }
    return false;
}

#ifdef SLAM_SERVER_HAVE_LIBHEIF
string heifErrorMessage(const heif_error& err)
{
    if (err.code == heif_error_Ok)
        return {};

    ostringstream oss;
    oss << "code=" << static_cast<int>(err.code)
        << " subcode=" << static_cast<int>(err.subcode);
    if (err.message)
        oss << " message=" << err.message;
    return oss.str();
}

cv::Mat decodeHeifBytes(const vector<uchar>& data, string* decode_error)
{
    unique_ptr<heif_context, decltype(&heif_context_free)> ctx(
        heif_context_alloc(), heif_context_free);
    if (!ctx) {
        if (decode_error)
            *decode_error = "failed to allocate libheif context";
        return {};
    }

    heif_error err = heif_context_read_from_memory_without_copy(
        ctx.get(), data.data(), data.size(), nullptr);
    if (err.code != heif_error_Ok) {
        if (decode_error)
            *decode_error = heifErrorMessage(err);
        return {};
    }

    heif_image_handle* raw_handle = nullptr;
    err = heif_context_get_primary_image_handle(ctx.get(), &raw_handle);
    unique_ptr<heif_image_handle, decltype(&heif_image_handle_release)> handle(
        raw_handle, heif_image_handle_release);
    if (err.code != heif_error_Ok || !handle) {
        if (decode_error)
            *decode_error = heifErrorMessage(err);
        return {};
    }

    const int width = heif_image_handle_get_width(handle.get());
    const int height = heif_image_handle_get_height(handle.get());
    if (width <= 0 || height <= 0) {
        if (decode_error)
            *decode_error = "invalid HEIC dimensions";
        return {};
    }

    heif_image* raw_image = nullptr;
    err = heif_decode_image(
        handle.get(), &raw_image, heif_colorspace_RGB, heif_chroma_interleaved_RGB, nullptr);
    unique_ptr<heif_image, decltype(&heif_image_release)> heif_image(
        raw_image, heif_image_release);
    if (err.code != heif_error_Ok || !heif_image) {
        if (decode_error)
            *decode_error = heifErrorMessage(err);
        return {};
    }

    int stride = 0;
    const uint8_t* plane =
        heif_image_get_plane_readonly(heif_image.get(), heif_channel_interleaved, &stride);
    const size_t row_bytes = static_cast<size_t>(width) * 3;
    if (!plane || stride < static_cast<int>(row_bytes)) {
        if (decode_error)
            *decode_error = "invalid libheif output plane";
        return {};
    }

    cv::Mat rgb(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        memcpy(rgb.ptr(y), plane + static_cast<size_t>(y) * stride, row_bytes);
    }

    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr;
}
#endif

cv::Mat decodeImageBytes(const vector<uchar>& data,
                         const string& content_type,
                         string* decode_error)
{
    cv::Mat image = cv::imdecode(data, cv::IMREAD_UNCHANGED);
    if (!image.empty())
        return image;

    if (!contentTypeIsHeif(content_type) && !dataLooksLikeHeif(data)) {
        if (decode_error)
            *decode_error = "OpenCV failed to decode image bytes";
        return {};
    }

#ifdef SLAM_SERVER_HAVE_LIBHEIF
    string heif_error;
    image = decodeHeifBytes(data, &heif_error);
    if (!image.empty())
        return image;

    if (decode_error) {
        *decode_error = "OpenCV failed to decode HEIC/HEIF image";
        if (!heif_error.empty())
            *decode_error += "; libheif fallback failed: " + heif_error;
    }
#else
    if (decode_error) {
        *decode_error =
            "OpenCV failed to decode HEIC/HEIF image; rebuild with libheif support "
            "or an OpenCV build that includes a HEIF codec";
    }
#endif

    return {};
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

cv::Mat barcodeLuminanceImage(const cv::Mat& image)
{
    if (image.empty())
        return {};

    if (image.depth() == CV_8U && image.channels() == 1)
        return image;

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    } else if (image.channels() == 1) {
        image.convertTo(gray, CV_8U);
    }
    return gray;
}

cv::Mat claheLuminance(const cv::Mat& gray)
{
    if (gray.empty())
        return {};

    cv::Mat enhanced;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, enhanced);
    return enhanced;
}

struct BarcodeRoiCandidate {
    cv::Rect rect;
    float score = 0.0f;
};

double integralRectSum(const cv::Mat& integral, const cv::Rect& rect)
{
    const int x0 = rect.x;
    const int y0 = rect.y;
    const int x1 = rect.x + rect.width;
    const int y1 = rect.y + rect.height;
    return integral.at<double>(y1, x1) -
           integral.at<double>(y0, x1) -
           integral.at<double>(y1, x0) +
           integral.at<double>(y0, x0);
}

vector<BarcodeRoiCandidate> barcodeRoiCandidates(const cv::Mat& gray,
                                                  size_t max_candidates)
{
    vector<BarcodeRoiCandidate> candidates;
    if (gray.empty() || gray.cols < 320 || gray.rows < 320 || max_candidates == 0)
        return candidates;

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0.0);

    cv::Mat grad_x;
    cv::Sobel(blurred, grad_x, CV_16S, 1, 0, 3);
    cv::Mat abs_grad_x;
    cv::convertScaleAbs(grad_x, abs_grad_x);

    cv::Mat strong_edges;
    cv::threshold(abs_grad_x, strong_edges, 90.0, 255.0, cv::THRESH_BINARY);
    cv::Mat dark_pixels;
    cv::threshold(gray, dark_pixels, 120.0, 255.0, cv::THRESH_BINARY_INV);
    cv::Mat bright_pixels;
    cv::threshold(gray, bright_pixels, 180.0, 255.0, cv::THRESH_BINARY);

    cv::Mat edge_integral;
    cv::Mat dark_integral;
    cv::Mat bright_integral;
    cv::integral(strong_edges, edge_integral, CV_64F);
    cv::integral(dark_pixels, dark_integral, CV_64F);
    cv::integral(bright_pixels, bright_integral, CV_64F);

    const int tile_w = std::min(gray.cols, std::max(240, gray.cols / 4));
    const int tile_h = std::min(gray.rows, std::max(180, gray.rows / 8));
    if (tile_w <= 0 || tile_h <= 0)
        return candidates;

    const int stride = std::max(120, tile_w / 3);
    const int pad = std::max(80, tile_w / 6);
    const double denom = 255.0 * static_cast<double>(tile_w) * static_cast<double>(tile_h);

    for (int y = 0; y + tile_h <= gray.rows; y += stride) {
        for (int x = 0; x + tile_w <= gray.cols; x += stride) {
            const cv::Rect tile(x, y, tile_w, tile_h);
            const double edge_ratio = integralRectSum(edge_integral, tile) / denom;
            const double dark_ratio = integralRectSum(dark_integral, tile) / denom;
            const double bright_ratio = integralRectSum(bright_integral, tile) / denom;

            // EAN bars produce many vertical edges, plus a moderate amount of black ink.
            const double dark_band_score =
                std::max(0.0, 0.35 - std::fabs(dark_ratio - 0.18));
            const float score = static_cast<float>(
                120.0 * edge_ratio + 20.0 * dark_band_score + 5.0 * bright_ratio);

            const int x0 = std::max(0, x - pad);
            const int y0 = std::max(0, y - pad);
            const int x1 = std::min(gray.cols, x + tile_w + pad);
            const int y1 = std::min(gray.rows, y + tile_h + pad);
            candidates.push_back({cv::Rect(x0, y0, x1 - x0, y1 - y0), score});
        }
    }

    sort(candidates.begin(), candidates.end(),
         [](const BarcodeRoiCandidate& a, const BarcodeRoiCandidate& b) {
             return a.score > b.score;
         });
    if (candidates.size() > max_candidates)
        candidates.resize(max_candidates);
    return candidates;
}

string narrowAscii(const wstring& text)
{
    string out;
    out.reserve(text.size());
    for (wchar_t c : text) {
        if (c < 0 || c > 127)
            return {};
        out.push_back(static_cast<char>(c));
    }
    return out;
}

bool isEan13Text(const string& text)
{
    return text.size() == 13 &&
           all_of(text.begin(), text.end(), [](unsigned char c) {
               return isdigit(c);
           });
}

cv::Point2f barcodePositionCenter(const ZXing::Position& position)
{
    cv::Point2f center(0.0f, 0.0f);
    int count = 0;
    for (const auto& p : position) {
        center.x += static_cast<float>(p.x);
        center.y += static_cast<float>(p.y);
        ++count;
    }
    if (count > 0) {
        center.x /= static_cast<float>(count);
        center.y /= static_cast<float>(count);
    }
    return center;
}

cv::Point2f barcodePositionCenter(const ZXing::Position& position,
                                  const cv::Point2f& offset,
                                  float scale)
{
    cv::Point2f center = barcodePositionCenter(position);
    if (std::isfinite(scale) && scale > 0.0f) {
        center.x /= scale;
        center.y /= scale;
    }
    center += offset;
    return center;
}

bool barcodePositionBounds(const ZXing::Position& position,
                           const cv::Size& image_size,
                           cv::Rect& bounds)
{
    if (image_size.width <= 0 || image_size.height <= 0)
        return false;

    float min_x = numeric_limits<float>::max();
    float min_y = numeric_limits<float>::max();
    float max_x = -numeric_limits<float>::max();
    float max_y = -numeric_limits<float>::max();
    int count = 0;
    for (const auto& p : position) {
        const float x = static_cast<float>(p.x);
        const float y = static_cast<float>(p.y);
        if (!std::isfinite(x) || !std::isfinite(y))
            continue;
        min_x = min(min_x, x);
        min_y = min(min_y, y);
        max_x = max(max_x, x);
        max_y = max(max_y, y);
        ++count;
    }
    if (count == 0)
        return false;

    const float span = max(max_x - min_x, max_y - min_y);
    const int pad = max(8, static_cast<int>(ceil(0.15f * span)));
    const int x0 = max(0, static_cast<int>(floor(min_x)) - pad);
    const int y0 = max(0, static_cast<int>(floor(min_y)) - pad);
    const int x1 = min(image_size.width, static_cast<int>(ceil(max_x)) + pad + 1);
    const int y1 = min(image_size.height, static_cast<int>(ceil(max_y)) + pad + 1);
    if (x1 <= x0 || y1 <= y0)
        return false;

    bounds = cv::Rect(x0, y0, x1 - x0, y1 - y0);
    return true;
}

bool maskBarcodeResult(cv::Mat& image, const ZXing::Result& result)
{
    cv::Rect bounds;
    if (!barcodePositionBounds(result.position(), image.size(), bounds))
        return false;
    cv::rectangle(image, bounds, cv::Scalar(255), cv::FILLED);
    return true;
}

bool barcodePositionBounds(const ZXing::Position& position,
                           const cv::Point2f& offset,
                           float scale,
                           const cv::Size& image_size,
                           cv::Rect& bounds)
{
    if (image_size.width <= 0 || image_size.height <= 0 ||
        !std::isfinite(scale) || scale <= 0.0f)
        return false;

    float min_x = numeric_limits<float>::max();
    float min_y = numeric_limits<float>::max();
    float max_x = -numeric_limits<float>::max();
    float max_y = -numeric_limits<float>::max();
    int count = 0;
    for (const auto& p : position) {
        const float x = static_cast<float>(p.x) / scale + offset.x;
        const float y = static_cast<float>(p.y) / scale + offset.y;
        if (!std::isfinite(x) || !std::isfinite(y))
            continue;
        min_x = min(min_x, x);
        min_y = min(min_y, y);
        max_x = max(max_x, x);
        max_y = max(max_y, y);
        ++count;
    }
    if (count == 0)
        return false;

    const int x0 = max(0, static_cast<int>(floor(min_x)));
    const int y0 = max(0, static_cast<int>(floor(min_y)));
    const int x1 = min(image_size.width, static_cast<int>(ceil(max_x)) + 1);
    const int y1 = min(image_size.height, static_cast<int>(ceil(max_y)) + 1);
    if (x1 <= x0 || y1 <= y0)
        return false;

    bounds = cv::Rect(x0, y0, x1 - x0, y1 - y0);
    return true;
}

cv::Mat bgrOverlayImage(const cv::Mat& image)
{
    if (image.empty())
        return {};

    cv::Mat eight_bit;
    if (image.depth() == CV_8U) {
        eight_bit = image;
    } else {
        image.convertTo(eight_bit, CV_8U);
    }

    cv::Mat bgr;
    if (eight_bit.channels() == 3) {
        bgr = eight_bit.clone();
    } else if (eight_bit.channels() == 4) {
        cv::cvtColor(eight_bit, bgr, cv::COLOR_BGRA2BGR);
    } else if (eight_bit.channels() == 1) {
        cv::cvtColor(eight_bit, bgr, cv::COLOR_GRAY2BGR);
    }
    return bgr;
}

void drawBarcodeDetections(cv::Mat& image, const vector<CustomDetection>& detections)
{
    if (image.empty())
        return;

    const int min_dim = min(image.cols, image.rows);
    const double font_scale = max(0.5, min(1.0, static_cast<double>(min_dim) / 2200.0));
    const int thickness = max(2, static_cast<int>(round(font_scale * 2.0)));
    const cv::Scalar box_color(0, 255, 255);
    const cv::Scalar text_color(0, 0, 0);
    const cv::Scalar label_bg(0, 255, 255);

    for (const auto& detection : detections) {
        const cv::Rect image_rect(0, 0, image.cols, image.rows);
        const cv::Rect rect = detection.bounds & image_rect;
        if (rect.empty())
            continue;

        cv::rectangle(image, rect, box_color, thickness, cv::LINE_AA);

        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(
            detection.id, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
        const int text_x = max(0, min(rect.x, image.cols - text_size.width - 2));
        int text_y = rect.y - 6;
        if (text_y - text_size.height - baseline < 0)
            text_y = min(image.rows - baseline - 1, rect.y + text_size.height + baseline + 6);

        const cv::Rect label_rect(
            max(0, text_x - 2),
            max(0, text_y - text_size.height - baseline - 2),
            min(image.cols - max(0, text_x - 2), text_size.width + 4),
            min(image.rows - max(0, text_y - text_size.height - baseline - 2),
                text_size.height + baseline + 4));
        if (!label_rect.empty())
            cv::rectangle(image, label_rect, label_bg, cv::FILLED);

        cv::putText(
            image,
            detection.id,
            cv::Point(text_x, text_y),
            cv::FONT_HERSHEY_SIMPLEX,
            font_scale,
            text_color,
            thickness,
            cv::LINE_AA);
    }
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

float smoothScale(float previous, float target, float alpha)
{
    if (!isValidScale(previous))
        return target;
    alpha = std::max(0.0f, std::min(1.0f, alpha));
    return (1.0f - alpha) * previous + alpha * target;
}

bool hasMeaningfulScaleBaseline(const Eigen::Vector3f& slam_a,
                                const Eigen::Vector3f& slam_b,
                                const Eigen::Vector3f& domain_a,
                                const Eigen::Vector3f& domain_b)
{
    if (!slam_a.allFinite() || !slam_b.allFinite() ||
        !domain_a.allFinite() || !domain_b.allFinite()) {
        return false;
    }

    const float slam_dist = (slam_a - slam_b).norm();
    const float domain_dist = (domain_a - domain_b).norm();
    return slam_dist > kMinSlamBaselinePairDistance &&
           domain_dist > kMinScaleBaselineDistanceM;
}

bool scaleBaselineRatio(const PortalObservation& a,
                        const PortalObservation& b,
                        float& ratio)
{
    if (!hasMeaningfulScaleBaseline(a.p_slam, b.p_slam, a.p_domain, b.p_domain))
        return false;

    const float slam_dist = (a.p_slam - b.p_slam).norm();
    const float domain_dist = (a.p_domain - b.p_domain).norm();
    ratio = domain_dist / slam_dist;
    return isValidScale(ratio);
}

/** One observation per slam-pose cluster (best reprojection). */
vector<PortalObservation> dedupePortalObservationsBySlamPose(
    const vector<PortalObservation>& observations)
{
    vector<PortalObservation> unique;
    unique.reserve(observations.size());
    for (const auto& obs : observations) {
        if (!obs.p_slam.allFinite() || !obs.p_domain.allFinite())
            continue;
        size_t match = unique.size();
        for (size_t i = 0; i < unique.size(); ++i) {
            if ((unique[i].p_slam - obs.p_slam).norm() <= kSlamPoseClusterEps) {
                match = i;
                break;
            }
        }
        if (match == unique.size()) {
            unique.push_back(obs);
            continue;
        }
        // Prefer lower reprojection error within a slam cluster.
        if (std::isfinite(obs.reproj_px) &&
            (!std::isfinite(unique[match].reproj_px) ||
             obs.reproj_px < unique[match].reproj_px)) {
            unique[match] = obs;
        }
    }
    return unique;
}

float portalObservationWeight(const PortalObservation& obs)
{
    if (!std::isfinite(obs.reproj_px))
        return 0.0f;
    return 1.0f / std::max(1.0f, obs.reproj_px);
}

Eigen::Matrix3f averagePortalAlignmentRotation(const vector<PortalObservation>& observations)
{
    Eigen::Vector4f accum = Eigen::Vector4f::Zero();
    Eigen::Quaternionf reference;
    bool have_reference = false;
    float total_weight = 0.0f;

    for (const auto& obs : observations) {
        const float weight = portalObservationWeight(obs);
        if (weight <= 0.0f || !obs.R_domain_slam.allFinite())
            continue;

        Eigen::Quaternionf q(obs.R_domain_slam);
        q.normalize();
        if (!q.coeffs().allFinite())
            continue;

        if (!have_reference) {
            reference = q;
            have_reference = true;
        } else if (reference.dot(q) < 0.0f) {
            q.coeffs() *= -1.0f;
        }

        accum += weight * q.coeffs();
        total_weight += weight;
    }

    if (!have_reference || total_weight <= 0.0f)
        return Eigen::Matrix3f::Identity();

    Eigen::Quaternionf averaged;
    averaged.coeffs() = accum / total_weight;
    averaged.normalize();
    return averaged.toRotationMatrix();
}

Eigen::Vector3f averagePortalAlignmentTranslation(
    const vector<PortalObservation>& observations,
    float scale,
    const Eigen::Matrix3f& R_domain_slam,
    const PortalObservation& fallback)
{
    Eigen::Vector3f accum = Eigen::Vector3f::Zero();
    float total_weight = 0.0f;

    for (const auto& obs : observations) {
        const float weight = portalObservationWeight(obs);
        if (weight <= 0.0f || !obs.p_slam.allFinite() || !obs.p_domain.allFinite())
            continue;

        accum += weight * (obs.p_domain - scale * (R_domain_slam * obs.p_slam));
        total_weight += weight;
    }

    if (total_weight <= 0.0f)
        return fallback.p_domain - scale * (R_domain_slam * fallback.p_slam);
    return accum / total_weight;
}

Eigen::Matrix3f smoothRotation(
    const Eigen::Matrix3f& previous,
    const Eigen::Matrix3f& target,
    float alpha)
{
    alpha = std::max(0.0f, std::min(1.0f, alpha));
    Eigen::Quaternionf q_previous(previous);
    Eigen::Quaternionf q_target(target);
    q_previous.normalize();
    q_target.normalize();
    if (q_previous.dot(q_target) < 0.0f)
        q_target.coeffs() *= -1.0f;

    Eigen::Quaternionf q = q_previous.slerp(alpha, q_target);
    q.normalize();
    return q.toRotationMatrix();
}

float rotationDeltaDegrees(const Eigen::Matrix3f& from, const Eigen::Matrix3f& to)
{
    const Eigen::Matrix3f delta = from.transpose() * to;
    const float cos_angle = std::max(-1.0f, std::min(1.0f, 0.5f * (delta.trace() - 1.0f)));
    return std::acos(cos_angle) * kRadiansToDegrees;
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

struct MultiMapPublishPlan {
    string policy = "active_only";
    unsigned long active_map_id = 0;
    bool active_map_domain_aligned = false;
    vector<unsigned long> included_map_ids;
    vector<unsigned long> withheld_map_ids;
    vector<unsigned long> pinned_map_ids;
};

json multiMapPublishToJson(const MultiMapPublishPlan& plan)
{
    json out;
    out["policy"] = plan.policy;
    out["active_map_id"] = plan.active_map_id;
    out["active_map_domain_aligned"] = plan.active_map_domain_aligned;
    out["included_map_ids"] = plan.included_map_ids;
    out["withheld_map_ids"] = plan.withheld_map_ids;
    out["pinned_map_ids"] = plan.pinned_map_ids;
    return out;
}

bool mapAlignmentIsDomainPinned(const AlignmentSnapshot& alignment)
{
    return alignment.enabled && alignment.has_transform;
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
    string prior_pack_path;
    string occlusion_mesh_path;
    LoadedObjMesh occlusion_mesh;
    bool use_viewer = false;
    /** Deep enough for prior-match-every=1 under ~10 FPS ingest without 503s. */
    int max_queue_size = 100;
    size_t slam_interval_nth = 1;
    size_t custom_detect_every = 0;
    /** Wait until this many frames are buffered (unless shutting down); then process oldest by `timestamp`. */
    size_t min_buffer_frames = 3;
    size_t portal_detect_every = 1;
    /** Accuracy vs cost tradeoff: every 5th tracked frame (holdout uses CLI override). */
    size_t prior_match_every_nth = 5;
    float portal_max_reproj_px = 8.0f;
    double processing_timeout = 5.0; // seconds
    /** UTF-8 HTML for GET /index.html (phone camera UI); empty => 404 with hint */
    string index_html_body;
};

// Image request structure
struct ImageRequest {
    cv::Mat image;
    double timestamp;
    bool has_capture_timestamp = false;
    size_t encoded_image_bytes = 0;
    string client_id;
    uint64_t frame_id;
    uint64_t accepted_index = 0;
    bool force_custom_detection = false;
    string content_type;
    vector<float> intrinsics; // Optional frame-specific intrinsics
};

struct CustomDetectionRequest {
    cv::Mat image;
    vector<float> intrinsics;
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    bool has_capture_timestamp = false;
    bool forced = false;
    bool run_portal_detection = false;
    bool has_associated_slam_pose = false;
    Sophus::SE3f associated_Tcw_slam;
    uint64_t original_frame_id = 0;
    double original_timestamp = 0.0;
};

struct LatestSlamFrame {
    bool valid = false;
    uint64_t frame_id = 0;
    double timestamp = 0.0;
    Sophus::SE3f Tcw_slam;
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

    // Custom detection can be slower than tracking; keep it off the SLAM worker path.
    mutex custom_detection_queue_mutex_;
    deque<shared_ptr<CustomDetectionRequest>> custom_detection_queue_;
    condition_variable custom_detection_queue_cv_;
    atomic<bool> custom_detection_active_{true};
    thread custom_detection_thread_;
    mutex latest_slam_frame_mutex_;
    LatestSlamFrame latest_slam_frame_;
    
    // Statistics
    atomic<uint64_t> total_requests_{0};
    atomic<uint64_t> successful_tracks_{0};
    atomic<uint64_t> failed_tracks_{0};
    atomic<uint64_t> skipped_tracks_{0};
    atomic<uint64_t> custom_detection_enqueued_{0};
    atomic<uint64_t> custom_detection_dropped_{0};
    atomic<uint64_t> custom_detection_processed_{0};
    atomic<double> avg_processing_time_{0.0};

    // Rolling VO stage timing (instrument-only; no behavior change).
    struct StageTimingStats {
        static constexpr size_t kWindow = 300;
        mutable mutex mu;
        array<double, kWindow> totals{};
        array<double, kWindow> extract{};
        array<double, kWindow> pose_pred{};
        array<double, kWindow> pose_mm{};
        array<double, kWindow> pose_refkf{};
        array<double, kWindow> pose_reloc{};
        array<double, kWindow> pose_mm_match{};
        array<double, kWindow> pose_mm_opt{};
        array<double, kWindow> lg_prep{};
        array<double, kWindow> lg_fwd{};
        array<double, kWindow> lg_drain{};
        array<double, kWindow> lg_post{};
        array<double, kWindow> local_map{};
        array<double, kWindow> new_kf{};
        array<double, kWindow> portal{};
        array<double, kWindow> prior{};
        size_t count = 0;
        size_t next = 0;
        uint64_t over_100ms = 0;
        uint64_t last_spike_frame = 0;
        ORB_SLAM3::FrameTimingMs last_vo{};
        double last_prepare_ms = 0;
        double last_portal_ms = 0;
        double last_prior_ms = 0;
        double last_total_ms = 0;
        double max_total_ms = 0;
        double max_local_map_ms = 0;
        double max_pose_mm_ms = 0;
        double max_pose_refkf_ms = 0;
        double max_pose_reloc_ms = 0;
        double max_pose_mm_match_ms = 0;
        double max_pose_mm_opt_ms = 0;
        double max_lg_fwd_ms = 0;
        double max_lg_drain_ms = 0;
        int max_local_map_points = 0;

        static double percentile(vector<double> v, double p) {
            if (v.empty()) return 0;
            sort(v.begin(), v.end());
            const double idx = p * static_cast<double>(v.size() - 1);
            const size_t i = static_cast<size_t>(idx);
            const double f = idx - static_cast<double>(i);
            if (i + 1 < v.size())
                return v[i] * (1.0 - f) + v[i + 1] * f;
            return v[i];
        }

        void record(uint64_t frame_id,
                    double prepare_ms,
                    const ORB_SLAM3::FrameTimingMs& vo,
                    double portal_ms,
                    double prior_ms,
                    double total_ms) {
            lock_guard<mutex> lock(mu);
            totals[next] = total_ms;
            extract[next] = vo.extract_ms;
            pose_pred[next] = vo.pose_pred_ms;
            pose_mm[next] = vo.pose_mm_ms;
            pose_refkf[next] = vo.pose_refkf_ms;
            pose_reloc[next] = vo.pose_reloc_ms;
            pose_mm_match[next] = vo.pose_mm_match_ms;
            pose_mm_opt[next] = vo.pose_mm_opt_ms;
            lg_prep[next] = vo.lg_prep_ms;
            lg_fwd[next] = vo.lg_fwd_ms;
            lg_drain[next] = vo.lg_drain_ms;
            lg_post[next] = vo.lg_post_ms;
            local_map[next] = vo.local_map_ms;
            new_kf[next] = vo.new_kf_ms;
            portal[next] = portal_ms;
            prior[next] = prior_ms;
            next = (next + 1) % kWindow;
            if (count < kWindow) count++;

            last_vo = vo;
            last_prepare_ms = prepare_ms;
            last_portal_ms = portal_ms;
            last_prior_ms = prior_ms;
            last_total_ms = total_ms;
            if (total_ms > max_total_ms) max_total_ms = total_ms;
            if (vo.local_map_ms > max_local_map_ms) max_local_map_ms = vo.local_map_ms;
            if (vo.pose_mm_ms > max_pose_mm_ms) max_pose_mm_ms = vo.pose_mm_ms;
            if (vo.pose_refkf_ms > max_pose_refkf_ms) max_pose_refkf_ms = vo.pose_refkf_ms;
            if (vo.pose_reloc_ms > max_pose_reloc_ms) max_pose_reloc_ms = vo.pose_reloc_ms;
            if (vo.pose_mm_match_ms > max_pose_mm_match_ms) max_pose_mm_match_ms = vo.pose_mm_match_ms;
            if (vo.pose_mm_opt_ms > max_pose_mm_opt_ms) max_pose_mm_opt_ms = vo.pose_mm_opt_ms;
            if (vo.lg_fwd_ms > max_lg_fwd_ms) max_lg_fwd_ms = vo.lg_fwd_ms;
            if (vo.lg_drain_ms > max_lg_drain_ms) max_lg_drain_ms = vo.lg_drain_ms;
            if (vo.local_map_points > max_local_map_points) max_local_map_points = vo.local_map_points;
            if (total_ms > 100.0) {
                over_100ms++;
                last_spike_frame = frame_id;
            }
        }

        json toJson() const {
            lock_guard<mutex> lock(mu);
            auto copy_window = [&](const array<double, kWindow>& src) {
                vector<double> out;
                out.reserve(count);
                if (count < kWindow) {
                    for (size_t i = 0; i < count; ++i) out.push_back(src[i]);
                } else {
                    for (size_t i = 0; i < kWindow; ++i)
                        out.push_back(src[(next + i) % kWindow]);
                }
                return out;
            };
            const auto tot = copy_window(totals);
            const auto ext = copy_window(extract);
            const auto pred = copy_window(pose_pred);
            const auto mm = copy_window(pose_mm);
            const auto refkf = copy_window(pose_refkf);
            const auto reloc = copy_window(pose_reloc);
            const auto mm_match = copy_window(pose_mm_match);
            const auto mm_opt = copy_window(pose_mm_opt);
            const auto lgprep = copy_window(lg_prep);
            const auto lgfwd = copy_window(lg_fwd);
            const auto lgdrain = copy_window(lg_drain);
            const auto lgpost = copy_window(lg_post);
            const auto loc = copy_window(local_map);
            const auto nkf = copy_window(new_kf);
            json j;
            j["budget_ms"] = 100.0;
            j["samples"] = count;
            j["over_100ms_count"] = over_100ms;
            j["last_spike_frame"] = last_spike_frame;
            j["max_total_ms"] = max_total_ms;
            j["max_local_map_ms"] = max_local_map_ms;
            j["max_pose_mm_ms"] = max_pose_mm_ms;
            j["max_pose_refkf_ms"] = max_pose_refkf_ms;
            j["max_pose_reloc_ms"] = max_pose_reloc_ms;
            j["max_pose_mm_match_ms"] = max_pose_mm_match_ms;
            j["max_pose_mm_opt_ms"] = max_pose_mm_opt_ms;
            j["max_lg_fwd_ms"] = max_lg_fwd_ms;
            j["max_lg_drain_ms"] = max_lg_drain_ms;
            j["max_local_map_points"] = max_local_map_points;
            j["last"] = {
                {"prepare_ms", last_prepare_ms},
                {"extract_ms", last_vo.extract_ms},
                {"pose_pred_ms", last_vo.pose_pred_ms},
                {"pose_mm_ms", last_vo.pose_mm_ms},
                {"pose_refkf_ms", last_vo.pose_refkf_ms},
                {"pose_reloc_ms", last_vo.pose_reloc_ms},
                {"pose_mm_match_ms", last_vo.pose_mm_match_ms},
                {"pose_mm_opt_ms", last_vo.pose_mm_opt_ms},
                {"lg_prep_ms", last_vo.lg_prep_ms},
                {"lg_fwd_ms", last_vo.lg_fwd_ms},
                {"lg_drain_ms", last_vo.lg_drain_ms},
                {"lg_post_ms", last_vo.lg_post_ms},
                {"lg_calls", last_vo.lg_calls},
                {"lg_kpts0", last_vo.lg_kpts0},
                {"lg_kpts1", last_vo.lg_kpts1},
                {"local_map_ms", last_vo.local_map_ms},
                {"new_kf_ms", last_vo.new_kf_ms},
                {"portal_ms", last_portal_ms},
                {"prior_ms", last_prior_ms},
                {"track_total_ms", last_vo.track_total_ms},
                {"total_ms", last_total_ms},
                {"local_map_points", last_vo.local_map_points},
                {"matches_inliers", last_vo.matches_inliers},
                {"state", last_vo.state}
            };
            j["p50_ms"] = {
                {"total", percentile(tot, 0.50)},
                {"extract", percentile(ext, 0.50)},
                {"pose_pred", percentile(pred, 0.50)},
                {"pose_mm", percentile(mm, 0.50)},
                {"pose_refkf", percentile(refkf, 0.50)},
                {"pose_reloc", percentile(reloc, 0.50)},
                {"pose_mm_match", percentile(mm_match, 0.50)},
                {"pose_mm_opt", percentile(mm_opt, 0.50)},
                {"lg_prep", percentile(lgprep, 0.50)},
                {"lg_fwd", percentile(lgfwd, 0.50)},
                {"lg_drain", percentile(lgdrain, 0.50)},
                {"lg_post", percentile(lgpost, 0.50)},
                {"local_map", percentile(loc, 0.50)},
                {"new_kf", percentile(nkf, 0.50)}
            };
            j["p95_ms"] = {
                {"total", percentile(tot, 0.95)},
                {"extract", percentile(ext, 0.95)},
                {"pose_pred", percentile(pred, 0.95)},
                {"pose_mm", percentile(mm, 0.95)},
                {"pose_refkf", percentile(refkf, 0.95)},
                {"pose_reloc", percentile(reloc, 0.95)},
                {"pose_mm_match", percentile(mm_match, 0.95)},
                {"pose_mm_opt", percentile(mm_opt, 0.95)},
                {"lg_prep", percentile(lgprep, 0.95)},
                {"lg_fwd", percentile(lgfwd, 0.95)},
                {"lg_drain", percentile(lgdrain, 0.95)},
                {"lg_post", percentile(lgpost, 0.95)},
                {"local_map", percentile(loc, 0.95)},
                {"new_kf", percentile(nkf, 0.95)}
            };
            // Chronological rolling window for live plots on /index.html.
            j["series_ms"] = {
                {"total", tot},
                {"extract", ext},
                {"lg_fwd", lgfwd},
                {"pose_mm_match", mm_match},
                {"local_map", loc}
            };
            return j;
        }
    };
    mutable StageTimingStats stage_timing_;
    string index_html_body_;
    atomic<bool> shutdown_done_{false};
    vector<float> default_intrinsics_;
    cv::Size slam_input_size_;
    cv::Size feature_image_size_;
    unordered_map<string, PortalDefinition> portals_;
    mutable mutex alignment_mutex_;
    mutex debug_output_mutex_;
    vector<PortalObservation> portal_observations_;
    vector<PortalCornerObservation> portal_corner_observations_;
    vector<CustomDetectionObservation> custom_detection_observations_;
    AlignmentSnapshot alignment_;
    unordered_map<unsigned long, AlignmentSnapshot> map_alignments_;
    unordered_map<unsigned long, vector<PortalObservation>> map_portal_observations_;
    unsigned long last_synced_map_id_ = numeric_limits<unsigned long>::max();
    prior_pack::PriorPack prior_pack_;
    overlay::OverlayPoseGraph overlay_graph_;
    prior_soft::PriorMatchConfig prior_match_config_;
    unique_ptr<prior_soft::PriorSoftEdgeMatcher> prior_soft_matcher_;
    int last_prior_seed_image_id_ = -1;

public:
    SLAMServer(const ServerConfig& config) : config_(config), index_html_body_(config.index_html_body) {
        default_intrinsics_ = loadDefaultIntrinsics(config_.settings_path);
        slam_input_size_ = loadConfiguredInputSize(config_.settings_path);
        feature_image_size_ = loadFeatureImageSize(config_.settings_path);
        if (!isValidImageSize(feature_image_size_))
            feature_image_size_ = slam_input_size_;
        if (default_intrinsics_.empty()) {
            cerr << "Warning: could not load default Camera1.fx/fy/cx/cy from settings; "
                 << "portal PnP requires per-frame intrinsics." << endl;
        }
        if (!isValidImageSize(slam_input_size_)) {
            cerr << "Warning: could not load Camera.width/height from settings; "
                 << "SLAM will receive request image resolution directly." << endl;
        }

        prior_match_config_.match_every_nth_frame = config_.prior_match_every_nth;
        if (const char* every_nth = getenv("ROVER_PRIOR_MATCH_EVERY_NTH")) {
            try {
                const size_t n = static_cast<size_t>(stoul(every_nth));
                if (n > 0)
                    prior_match_config_.match_every_nth_frame = n;
            } catch (...) {
                cerr << "Warning: invalid ROVER_PRIOR_MATCH_EVERY_NTH" << endl;
            }
        }
        prior_match_config_.fixture_smoke = priorSoftEdgeFixtureSmokeEnabled();
        prior_soft_matcher_ =
            make_unique<prior_soft::PriorSoftEdgeMatcher>(prior_match_config_);

        if (!config_.portal_manifest_path.empty()) {
            loadPortalManifest(config_.portal_manifest_path);
        }

        if (!config_.prior_pack_path.empty()) {
            loadPriorPack(config_.prior_pack_path);
            runPriorPackFixtureSmokeIfEnabled();
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
        publishViewerTopographyMesh();

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
        custom_detection_thread_ = thread(&SLAMServer::processCustomDetections, this);
        
        cout << "SLAM Server initialized successfully" << endl;
        cout << "Vocabulary: " << config_.vocabulary_path << endl;
        cout << "Settings: " << config_.settings_path << endl;
        cout << "SLAM frame interval: every " << config_.slam_interval_nth
             << " accepted frame(s)" << endl;
        if (config_.custom_detect_every > 0) {
            cout << "Custom detection interval: every " << config_.custom_detect_every
                 << " accepted frame(s)" << endl;
        } else {
            cout << "Custom detection interval: disabled (use X-Run-Detector per frame)" << endl;
        }
        if (isValidImageSize(slam_input_size_)) {
            cout << "SLAM input resolution: " << slam_input_size_.width
                 << " x " << slam_input_size_.height << endl;
        }
        if (isValidImageSize(feature_image_size_) &&
            !sameImageSize(feature_image_size_, slam_input_size_)) {
            cout << "Feature extraction resolution: " << feature_image_size_.width
                 << " x " << feature_image_size_.height << endl;
        }
        if (!config_.debug_output_dir.empty())
            cout << "Debug trajectory output: "
                 << joinPath(config_.debug_output_dir, "trajectory_latest.json") << endl;
        if (!config_.portal_manifest_path.empty()) {
            cout << "Portal manifest: " << config_.portal_manifest_path
                 << " (" << portals_.size() << " usable portal(s))" << endl;
        } else if (!portals_.empty()) {
            cout << "Portal catalog: prior pack (" << portals_.size()
                 << " usable portal(s) for alignment)" << endl;
        }
        if (prior_pack_.loaded)
            cout << "Prior pack: " << prior_pack_.pack_root
                 << " (" << prior_pack_.image_count() << " images, "
                 << prior_pack_.track_count() << " tracks, "
                 << prior_pack_.portal_count() << " portals)" << endl;
        if (!config_.occlusion_mesh_path.empty())
            cout << "Occlusion mesh: " << config_.occlusion_mesh_path
                 << " (" << config_.occlusion_mesh.triangles.size() << " triangle(s))" << endl;
        if (!config_.output_dir.empty())
            cout << "Detection overlay output directory: " << config_.output_dir << endl;
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
        custom_detection_active_ = false;
        custom_detection_queue_cv_.notify_all();
        
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        if (custom_detection_thread_.joinable()) {
            custom_detection_thread_.join();
        }
        
        // Shutdown SLAM
        if (slam_system_) {
            slam_system_->Shutdown();
        }
        
        cout << "SLAM server shutdown complete" << endl;
    }

private:
    /** Shared portal catalog ingest: JSON entries (manifest or pack shape) → portals_. */
    size_t ingestPortalCatalogEntries(const json& entries) {
        const Eigen::Quaternionf q_manifest_wall_to_qr_plane(
            Eigen::AngleAxisf(-1.57079632679489661923f, Eigen::Vector3f::UnitY()));

        size_t skipped = 0;
        for (const auto& entry : entries) {
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
                    pos["x"].get<float>(),
                    pos["y"].get<float>(),
                    pos["z"].get<float>());

                Eigen::Quaternionf q_domain_portal(
                    rot["w"].get<float>(),
                    rot["x"].get<float>(),
                    rot["y"].get<float>(),
                    rot["z"].get<float>());
                // Wall-mounted pose; QR/PnP object points use z=0 as the portal plane (−π/2 about Y).
                q_domain_portal = q_domain_portal * q_manifest_wall_to_qr_plane;
                q_domain_portal.normalize();

                PortalDefinition portal;
                portal.short_id = upperCopy(entry["shortId"].get<string>());
                portal.physical_size_m = physical_size;
                portal.T_domain_portal =
                    Sophus::SE3f(q_domain_portal.toRotationMatrix(), t_domain_portal);
                portals_[portal.short_id] = portal;
            } catch (const exception& e) {
                skipped++;
                cerr << "Warning: skipping invalid portal catalog entry: " << e.what() << endl;
            }
        }
        return skipped;
    }

    void setAlignmentPortalCatalogLoaded(const string& empty_status, const string& loaded_status) {
        lock_guard<mutex> lock(alignment_mutex_);
        alignment_.enabled = !portals_.empty();
        alignment_.status = portals_.empty() ? empty_status : loaded_status;
    }

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

        const size_t skipped =
            ingestPortalCatalogEntries(data.value("portals", json::array()));
        setAlignmentPortalCatalogLoaded(
            "portal_manifest_empty", "portal_manifest_loaded_waiting_for_detection");

        cout << "Loaded " << portals_.size() << " portal(s) from " << manifest_path;
        if (skipped > 0)
            cout << " (" << skipped << " skipped)";
        cout << endl;
    }

    /** When --portal-manifest is unset, prior pack portals[] feed the alignment catalog. */
    void loadPortalsFromPriorPackIfNoManifest() {
        if (!config_.portal_manifest_path.empty())
            return;
        if (prior_pack_.portals.empty())
            return;

        json entries = json::array();
        for (const auto& rec : prior_pack_.portals) {
            entries.push_back(
                {{"shortId", rec.short_id},
                 {"physicalSize", rec.physical_size},
                 {"pose",
                  {{"position",
                    {{"x", rec.pose.position.x},
                     {"y", rec.pose.position.y},
                     {"z", rec.pose.position.z}}},
                   {"rotation",
                    {{"w", rec.pose.rotation.w},
                     {"x", rec.pose.rotation.x},
                     {"y", rec.pose.rotation.y},
                     {"z", rec.pose.rotation.z}}}}}});
        }

        const size_t skipped = ingestPortalCatalogEntries(entries);
        setAlignmentPortalCatalogLoaded(
            "prior_pack_portals_empty", "prior_pack_portals_loaded_waiting_for_detection");

        cout << "Loaded " << portals_.size() << " portal(s) from prior pack for alignment";
        if (skipped > 0)
            cout << " (" << skipped << " skipped)";
        cout << endl;
    }

    void loadPriorPack(const string& pack_path) {
        try {
            prior_pack_ = prior_pack::load_prior_pack(pack_path);
        } catch (const prior_pack::PriorPackError& e) {
            throw runtime_error(string("Failed to load --prior-pack: ") + e.what());
        }
        cout << "Loaded prior pack from " << pack_path << ": "
             << prior_pack_.image_count() << " image(s), "
             << prior_pack_.track_count() << " track(s), "
             << prior_pack_.portal_count() << " portal(s), "
             << prior_pack_.covis_edge_count() << " covis edge(s), "
             << prior_pack_.total_keypoints() << " keypoint(s)" << endl;
        loadPortalsFromPriorPackIfNoManifest();
    }

    static bool priorSoftEdgeFixtureSmokeEnabled() {
        const char* v = getenv("ROVER_PRIOR_SOFT_EDGE_FIXTURE_SMOKE");
        if (!v)
            return false;
        return strcmp(v, "1") == 0 || strcmp(v, "true") == 0;
    }

    void runPriorPackFixtureSmokeIfEnabled() {
        if (!prior_match_config_.fixture_smoke || !prior_soft_matcher_ || !prior_pack_.loaded ||
            prior_pack_.images.empty())
            return;

        const prior_pack::ImageRecord& img = prior_pack_.images.front();
        prior_soft::PriorMatchInput input;
        input.pack = &prior_pack_;
        for (const auto& kp : img.features.keypoints) {
            cv::KeyPoint keypoint;
            keypoint.pt = cv::Point2f(kp.first, kp.second);
            input.live_kpts.push_back(keypoint);
        }
        if (input.live_kpts.empty() || img.features.descriptors.empty())
            return;

        const int desc_dim = prior_pack_.descriptor_dim;
        input.live_desc = cv::Mat(static_cast<int>(img.features.descriptors.size()),
                                  desc_dim,
                                  CV_32F);
        for (int r = 0; r < input.live_desc.rows; ++r) {
            for (int c = 0; c < desc_dim; ++c)
                input.live_desc.at<float>(r, c) = img.features.descriptors[r][c];
        }

        Eigen::Quaternionf q(img.pose.rotation.w,
                             img.pose.rotation.x,
                             img.pose.rotation.y,
                             img.pose.rotation.z);
        q.normalize();
        const Eigen::Vector3f t(img.pose.position.x, img.pose.position.y, img.pose.position.z);
        const Sophus::SE3f Twc(q.toRotationMatrix(), t);
        input.Twc_slam = Twc;
        input.has_domain_pose = true;
        input.Twc_domain = Twc;
        input.intrinsics = {img.intrinsics.fx, img.intrinsics.fy, img.intrinsics.cx, img.intrinsics.cy};
        input.image_width = static_cast<int>(2.f * img.intrinsics.cx);
        input.image_height = static_cast<int>(2.f * img.intrinsics.cy);
        if (input.image_width <= 0 || input.image_height <= 0) {
            input.image_width = 128;
            input.image_height = 96;
        }
        input.frame_id = 0;
        input.timestamp = 0.0;
        input.seed_prior_image_id = img.id;

        const prior_soft::PriorMatchResult result = prior_soft_matcher_->tryMatch(input);
        if (result.accepted) {
            overlay_graph_.addPriorSoftEdgeFactor(0, result.factor);
            last_prior_seed_image_id_ = result.matched_prior_image_id;
            cout << "[prior] fixture smoke accepted image=" << result.matched_prior_image_id
                 << " inliers=" << result.factor.inlier_count
                 << " reproj_px=" << result.factor.mean_reproj_px
                 << " total_accepted=" << overlay_graph_.priorSoftEdgesAccepted() << endl;
        } else {
            cout << "[prior] fixture smoke skip: " << result.skip_reason << endl;
        }
    }

    bool shouldRunPriorMatch(uint64_t frame_id) const {
        return prior_pack_.loaded &&
               config_.prior_match_every_nth > 0 &&
               (frame_id % config_.prior_match_every_nth) == 0;
    }

    void tryPriorSoftEdgesAfterTrack(unsigned long map_id,
                                     uint64_t frame_id,
                                     double timestamp,
                                     const Sophus::SE3f& Tcw_slam,
                                     const vector<cv::KeyPoint>& live_kpts,
                                     const cv::Mat& live_desc,
                                     const vector<float>& intrinsics,
                                     int image_width,
                                     int image_height) {
        if (!prior_soft_matcher_ || live_desc.empty() || live_kpts.empty())
            return;
        if (!overlay_graph_.isMapPinned(map_id)) {
            return;
        }

        prior_soft::PriorMatchInput input;
        input.pack = &prior_pack_;
        input.live_kpts = live_kpts;
        input.live_desc = live_desc;
        input.Twc_slam = Tcw_slam.inverse();
        input.intrinsics = intrinsics;
        input.image_width = image_width;
        input.image_height = image_height;
        input.frame_id = frame_id;
        input.timestamp = timestamp;
        const Sophus::SE3f Twc_slam = input.Twc_slam;
        input.has_domain_pose = true;
        input.Twc_domain = overlay_graph_.applyPinToTwc(map_id, Twc_slam);
        // Always re-pick nearest prior image under the current pin (incl. refined scale),
        // instead of sticking to the last accepted match's covis cluster.
        if (prior_pack_.loaded) {
            input.seed_prior_image_id =
                prior_soft::PriorSoftEdgeMatcher::nearestPriorImageId(prior_pack_, input.Twc_domain);
        }

        const prior_soft::PriorMatchResult result = prior_soft_matcher_->tryMatch(input);
        if (result.accepted) {
            overlay::PriorSoftEdgeFactor factor = result.factor;
            const overlay::RefineResult refine =
                overlay_graph_.addPriorSoftEdgeFactor(map_id, factor);
            if (refine.published) {
                applyOverlayRefineToAlignment(map_id, refine);
            }
            cout << "[prior] soft edge accepted image=" << result.matched_prior_image_id
                 << " seed_img=" << input.seed_prior_image_id
                 << " inliers=" << result.factor.inlier_count
                 << " obs=" << result.factor.observations.size()
                 << " reproj_px=" << result.factor.mean_reproj_px
                 << " err_live_p50=" << result.debug_err_live_p50
                 << " err_prior_p50=" << result.debug_err_prior_p50
                 << " pass_live=" << result.debug_pass_live
                 << " pass_prior=" << result.debug_pass_prior
                 << " total_accepted=" << overlay_graph_.priorSoftEdgesAccepted() << endl;
        } else if (result.attempted && !result.skip_reason.empty()) {
            if (result.skip_reason == "slam_disagreement") {
                overlay_graph_.recordPriorSoftEdgeReject(result.skip_reason);
            }
            cout << "[prior] soft edge skip frame " << frame_id << ": " << result.skip_reason
                 << " rejected_total=" << overlay_graph_.priorSoftEdgesRejected();
            if (result.skip_reason == "no_gated_match") {
                cout << " lg_matches=" << result.debug_lg_matches
                     << " best_inliers=" << result.debug_best_inliers
                     << " with_track=" << result.debug_with_track
                     << " err_live_p50=" << result.debug_err_live_p50
                     << " err_prior_p50=" << result.debug_err_prior_p50
                     << " pass_live=" << result.debug_pass_live
                     << " pass_prior=" << result.debug_pass_prior
                     << " pass_both=" << result.debug_pass_both
                     << " twc_seed_dist_m=" << result.debug_twc_seed_dist_m
                     << " seed_img=" << result.debug_seed_prior_image_id
                     << " mean_abs_z_live=" << result.debug_mean_abs_z_live;
            }
            cout << endl;
        }
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

    bool shouldRunCustomDetection(const shared_ptr<ImageRequest>& request) const {
        if (!request)
            return false;
        if (request->force_custom_detection)
            return true;
        return config_.custom_detect_every > 0 &&
               request->accepted_index > 0 &&
               ((request->accepted_index - 1) % config_.custom_detect_every) == 0;
    }

    bool shouldTreatAsDetectorOnlyStill(const shared_ptr<ImageRequest>& request) const {
        if (!request || !request->force_custom_detection)
            return false;
        if (contentTypeIsHeif(request->content_type))
            return true;
        if (!isValidImageSize(slam_input_size_))
            return false;

        return request->image.cols > (slam_input_size_.width * 3) / 2 ||
               request->image.rows > (slam_input_size_.height * 3) / 2;
    }

    LatestSlamFrame latestSlamFrameSnapshot() {
        lock_guard<mutex> lock(latest_slam_frame_mutex_);
        return latest_slam_frame_;
    }

    void updateLatestSlamFrame(uint64_t frame_id,
                               double timestamp,
                               const Sophus::SE3f& Tcw_slam) {
        lock_guard<mutex> lock(latest_slam_frame_mutex_);
        latest_slam_frame_.valid = true;
        latest_slam_frame_.frame_id = frame_id;
        latest_slam_frame_.timestamp = timestamp;
        latest_slam_frame_.Tcw_slam = Tcw_slam;
    }

    vector<float> fullResolutionIntrinsicsForRequest(const shared_ptr<ImageRequest>& request) const {
        if (!request)
            return {};
        if (request->intrinsics.size() >= 4)
            return request->intrinsics;
        if (default_intrinsics_.size() < 4)
            return {};

        const cv::Size image_size(request->image.cols, request->image.rows);
        if (isValidImageSize(slam_input_size_) && isValidImageSize(image_size) &&
            !sameImageSize(image_size, slam_input_size_)) {
            return scaleIntrinsicsToSize(default_intrinsics_, slam_input_size_, image_size);
        }
        return default_intrinsics_;
    }

    cv::Mat prepareSlamImage(const shared_ptr<ImageRequest>& request,
                             vector<float>& slam_intrinsics) const {
        slam_intrinsics = fullResolutionIntrinsicsForRequest(request);
        if (!request)
            return {};

        const cv::Size source_size(request->image.cols, request->image.rows);
        if (!isValidImageSize(slam_input_size_) || sameImageSize(source_size, slam_input_size_))
            return request->image;

        cv::Mat slam_image;
        cv::resize(request->image, slam_image, slam_input_size_);
        slam_intrinsics = scaleIntrinsicsToSize(slam_intrinsics, source_size, slam_input_size_);
        return slam_image;
    }

    bool enqueueCustomDetection(const shared_ptr<ImageRequest>& request) {
        if (!request || request->image.empty())
            return false;

        auto detection_request = make_shared<CustomDetectionRequest>();
        detection_request->image = request->image;
        detection_request->intrinsics = fullResolutionIntrinsicsForRequest(request);
        detection_request->frame_id = request->frame_id;
        detection_request->timestamp = request->timestamp;
        detection_request->has_capture_timestamp = request->has_capture_timestamp;
        detection_request->forced = request->force_custom_detection;
        detection_request->original_frame_id = request->frame_id;
        detection_request->original_timestamp = request->timestamp;

        cout << "[custom] enqueue frame " << request->frame_id
             << " size=" << request->image.cols << "x" << request->image.rows
             << " image_kib=" << formatKiB(request->encoded_image_bytes)
             << " timestamp_source=" << (request->has_capture_timestamp ? "client" : "server")
             << " forced=" << (request->force_custom_detection ? "true" : "false")
             << " content_type=" << request->content_type << endl;

        if (shouldTreatAsDetectorOnlyStill(request)) {
            const LatestSlamFrame latest = latestSlamFrameSnapshot();
            if (latest.valid) {
                detection_request->has_associated_slam_pose = true;
                detection_request->associated_Tcw_slam = latest.Tcw_slam;
                detection_request->run_portal_detection = true;

                cout << "[custom] detector-only still frame " << request->frame_id
                     << " associated with previous SLAM frame " << latest.frame_id << endl;
            } else {
                cout << "[custom] detector-only still frame " << request->frame_id
                     << " queued without SLAM pose association" << endl;
            }
        }

        {
            lock_guard<mutex> lock(custom_detection_queue_mutex_);
            if (!custom_detection_active_.load())
                return false;

            if (custom_detection_queue_.size() >= kMaxCustomDetectionQueue) {
                if (!detection_request->forced) {
                    custom_detection_dropped_++;
                    return false;
                }

                auto drop_it = find_if(
                    custom_detection_queue_.begin(),
                    custom_detection_queue_.end(),
                    [](const shared_ptr<CustomDetectionRequest>& queued) {
                        return queued && !queued->forced;
                    });
                if (drop_it != custom_detection_queue_.end()) {
                    custom_detection_queue_.erase(drop_it);
                    custom_detection_dropped_++;
                }
            }

            custom_detection_queue_.push_back(detection_request);
            custom_detection_enqueued_++;
        }
        custom_detection_queue_cv_.notify_one();
        return true;
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

    void writeCustomDetectionOverlay(const cv::Mat& image,
                                     const vector<CustomDetection>& detections,
                                     uint64_t frame_id) const {
        if (config_.output_dir.empty() || image.empty())
            return;

        if (!ensureDirectoryRecursive(config_.output_dir)) {
            cerr << "Warning: could not create detection output directory: "
                 << config_.output_dir << endl;
            return;
        }

        cv::Mat overlay = bgrOverlayImage(image);
        if (overlay.empty()) {
            cerr << "Warning: could not prepare detection overlay for frame "
                 << frame_id << endl;
            return;
        }

        drawBarcodeDetections(overlay, detections);

        const string filename =
            "frame_" + to_string(frame_id) + "_" +
            to_string(image.cols) + "x" + to_string(image.rows) +
            "_detections.png";
        const string path = joinPath(config_.output_dir, filename);
        if (!cv::imwrite(path, overlay)) {
            cerr << "Warning: could not write detection overlay: " << path << endl;
            return;
        }

        cout << "[custom] frame " << frame_id
             << " wrote detection overlay " << path << endl;
    }

    vector<CustomDetection> customDetect(const cv::Mat& image,
                                         const vector<float>& intrinsics,
                                         uint64_t frame_id) const {
        (void)intrinsics;

        const auto detector_start = chrono::high_resolution_clock::now();
        double luminance_ms = 0.0;
        double full_gray_ms = 0.0;
        double clahe_prepare_ms = 0.0;
        double full_clahe_ms = 0.0;
        double roi_candidates_ms = 0.0;
        double roi_scans_ms = 0.0;
        int scan_calls = 0;
        int zxing_calls = 0;
        vector<CustomDetection> detections;
        cv::imwrite("frame_" + to_string(frame_id) + "_original.png", image);
        const auto luminance_start = chrono::high_resolution_clock::now();
        const cv::Mat gray = barcodeLuminanceImage(image);
        luminance_ms = elapsedMs(luminance_start, chrono::high_resolution_clock::now());
        if (gray.empty()) {
            cout << "[custom] frame " << frame_id
                 << " skipped: unsupported image format channels=" << image.channels()
                 << " depth=" << image.depth() << endl;
            writeCustomDetectionOverlay(image, detections, frame_id);
            return detections;
        }

        const auto appendResult = [&](const ZXing::Result& result,
                                      const string& variant,
                                      const cv::Point2f& offset,
                                      float scale) -> bool {
            if (!result.isValid())
                return false;

            const string id = narrowAscii(result.text());
            cout << "[custom] frame " << frame_id
                 << " ZXing " << variant
                 << " format=" << ZXing::ToString(result.format())
                 << " text=" << id << endl;

            if (result.format() != ZXing::BarcodeFormat::EAN_13 ||
                !isEan13Text(id))
                return false;

            CustomDetection detection;
            detection.type = "barcode_ean13";
            detection.id = id;
            detection.pixel = barcodePositionCenter(result.position(), offset, scale);
            if (!std::isfinite(detection.pixel.x) || !std::isfinite(detection.pixel.y))
                return false;
            if (!barcodePositionBounds(
                    result.position(),
                    offset,
                    scale,
                    cv::Size(image.cols, image.rows),
                    detection.bounds))
                return false;

            const float duplicate_radius_sq =
                kBarcodeDuplicateCenterTolerancePx * kBarcodeDuplicateCenterTolerancePx;
            for (const auto& existing : detections) {
                if (existing.id != detection.id)
                    continue;
                const float dx = existing.pixel.x - detection.pixel.x;
                const float dy = existing.pixel.y - detection.pixel.y;
                if (dx * dx + dy * dy <= duplicate_radius_sq)
                    return false;
            }

            detections.push_back(detection);
            cout << "[custom] frame " << frame_id << " detected EAN-13 " << id
                 << " at pixel (" << detection.pixel.x << ", " << detection.pixel.y << ")" << endl;
            return true;
        };

        const auto scan = [&](const cv::Mat& lum,
                              ZXing::Binarizer binarizer,
                              const string& variant,
                              const cv::Point2f& offset = cv::Point2f(0.0f, 0.0f),
                              float scale = 1.0f) {
            if (lum.empty())
                return;

            ZXing::DecodeHints hints;
            hints.setFormats(ZXing::BarcodeFormat::EAN_13)
                 .setTryHarder(true)
                 .setTryRotate(true)
                 .setBinarizer(binarizer);

            const ZXing::ImageView view(
                lum.ptr<uint8_t>(),
                lum.cols,
                lum.rows,
                ZXing::ImageFormat::Lum,
                static_cast<int>(lum.step));
            scan_calls++;
            zxing_calls++;
            ZXing::Result result = ZXing::ReadBarcode(view, hints);
            if (!result.isValid())
                return;

            cv::Mat masked = lum.clone();
            for (int attempt = 0; attempt < kMaxBarcodeResultsPerTrial && result.isValid(); ++attempt) {
                appendResult(result, variant, offset, scale);
                // ZXing v1.1 returns one barcode per call; mask each hit so this trial can reveal more.
                if (!maskBarcodeResult(masked, result))
                    break;

                const ZXing::ImageView masked_view(
                    masked.ptr<uint8_t>(),
                    masked.cols,
                    masked.rows,
                    ZXing::ImageFormat::Lum,
                    static_cast<int>(masked.step));
                zxing_calls++;
                result = ZXing::ReadBarcode(masked_view, hints);
            }
        };

        cv::imwrite("frame_" + to_string(frame_id) + "_gray.png", gray);
        const auto full_gray_start = chrono::high_resolution_clock::now();
        scan(gray, ZXing::Binarizer::LocalAverage, "gray/local");
        full_gray_ms = elapsedMs(full_gray_start, chrono::high_resolution_clock::now());

        const auto clahe_prepare_start = chrono::high_resolution_clock::now();
        cv::Mat clahe = claheLuminance(gray);
        clahe_prepare_ms = elapsedMs(clahe_prepare_start, chrono::high_resolution_clock::now());
        cv::imwrite("frame_" + to_string(frame_id) + "_clahe.png", clahe);
        const auto full_clahe_start = chrono::high_resolution_clock::now();
        scan(clahe, ZXing::Binarizer::LocalAverage, "clahe/local");
        full_clahe_ms = elapsedMs(full_clahe_start, chrono::high_resolution_clock::now());

        const auto roi_candidates_start = chrono::high_resolution_clock::now();
        const vector<BarcodeRoiCandidate> candidates =
            barcodeRoiCandidates(gray, numeric_limits<size_t>::max());
        roi_candidates_ms = elapsedMs(roi_candidates_start, chrono::high_resolution_clock::now());
        const vector<float> scales = {1.0f};//, 2.0f, 1.5f};
        const auto roi_scans_start = chrono::high_resolution_clock::now();
        for (size_t i = 0; i < candidates.size(); ++i) {
            const BarcodeRoiCandidate& candidate = candidates[i];
            if (candidate.rect.width <= 0 || candidate.rect.height <= 0)
                continue;

            const cv::Mat crop = gray(candidate.rect);
            const cv::Mat crop_clahe = claheLuminance(crop);
            const cv::Point2f offset(
                static_cast<float>(candidate.rect.x),
                static_cast<float>(candidate.rect.y));

            
            for (float scale : scales) {
                cv::Mat scaled_crop;
                cv::Mat scaled_clahe;
                const cv::Mat* crop_to_scan = &crop;
                const cv::Mat* clahe_to_scan = &crop_clahe;
                if (std::fabs(scale - 1.0f) > 1e-6f) {
                    cv::resize(crop, scaled_crop, cv::Size(), scale, scale, cv::INTER_CUBIC);
                    cv::resize(crop_clahe, scaled_clahe, cv::Size(), scale, scale, cv::INTER_CUBIC);
                    crop_to_scan = &scaled_crop;
                    clahe_to_scan = &scaled_clahe;
                }

                const string suffix =
                    " roi#" + to_string(i) + " score=" + to_string(candidate.score) +
                    " scale=" + to_string(scale);
                scan(*crop_to_scan, ZXing::Binarizer::LocalAverage, "gray/local" + suffix,
                     offset, scale);
                scan(*clahe_to_scan, ZXing::Binarizer::LocalAverage, "clahe/local" + suffix,
                     offset, scale);
            }
            
            // string suffix =
            //        " roi#" + to_string(i) + " score=" + to_string(candidate.score);
            //scan(crop, ZXing::Binarizer::LocalAverage, "gray/local" + suffix, offset, 1.0f);
        }
        roi_scans_ms = elapsedMs(roi_scans_start, chrono::high_resolution_clock::now());

        if (detections.empty()) {
            cout << "[custom] frame " << frame_id
                 << " no EAN-13 in " << gray.cols << "x" << gray.rows << endl;
        }
        const double total_ms = elapsedMs(detector_start, chrono::high_resolution_clock::now());
        cout << "[custom] frame " << frame_id
             << " barcode timing_ms total=" << total_ms
             << " luminance=" << luminance_ms
             << " full_gray=" << full_gray_ms
             //<< " clahe_prepare=" << clahe_prepare_ms
             //<< " full_clahe=" << full_clahe_ms
             << " roi_candidates=" << roi_candidates_ms
             << " roi_scans=" << roi_scans_ms
             << " candidates=" << candidates.size()
             << " scan_calls=" << scan_calls
             << " zxing_calls=" << zxing_calls
             << " detections=" << detections.size()
             << endl;
        writeCustomDetectionOverlay(image, detections, frame_id);
        return detections;
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

    void storeCustomDetectionObservations(const vector<CustomDetection>& detections,
                                          uint64_t frame_id,
                                          double timestamp,
                                          bool has_capture_timestamp,
                                          const vector<float>& intrinsics) {
        if (detections.empty() || intrinsics.size() < 4)
            return;

        lock_guard<mutex> lock(alignment_mutex_);
        for (const auto& det : detections) {
            if (det.type.empty() || det.id.empty() ||
                !std::isfinite(det.pixel.x) || !std::isfinite(det.pixel.y)) {
                continue;
            }

            CustomDetectionObservation obs;
            obs.frame_id = frame_id;
            obs.timestamp = timestamp;
            obs.has_capture_timestamp = has_capture_timestamp;
            obs.type = det.type;
            obs.id = det.id;
            obs.intrinsics = intrinsics;
            obs.pixel = det.pixel;
            custom_detection_observations_.push_back(obs);
        }

        if (custom_detection_observations_.size() > kMaxCustomDetectionObservations) {
            const size_t overflow =
                custom_detection_observations_.size() - kMaxCustomDetectionObservations;
            custom_detection_observations_.erase(
                custom_detection_observations_.begin(),
                custom_detection_observations_.begin() + static_cast<ptrdiff_t>(overflow));
        }
    }

    vector<CustomDetectionObservation> customDetectionObservationsSnapshot() {
        lock_guard<mutex> lock(alignment_mutex_);
        return custom_detection_observations_;
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

    void publishViewerTopographyMesh() {
        if (!slam_system_)
            return;
        if (config_.occlusion_mesh.triangles.empty())
            slam_system_->ClearViewerTopographyMesh();
        else
            slam_system_->SetViewerTopographyMesh(config_.occlusion_mesh.triangles);
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

            const Sophus::SE3f* Twc_slam = poseAtOrBeforeFrame(Twc_raw_by_frame, obs.frame_id);
            if (!Twc_slam)
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
                    Twc_slam->so3().matrix() * (metric_to_slam * p_cam) +
                    Twc_slam->translation();
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

    void refreshViewerCustomPoints() {
        if (!slam_system_)
            return;

        vector<CustomDetectionObservation> observations;
        AlignmentSnapshot alignment;
        {
            lock_guard<mutex> lock(alignment_mutex_);
            observations = custom_detection_observations_;
            alignment = alignment_;
        }
        if (observations.empty()) {
            slam_system_->ClearViewerCustomPoints();
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
            slam_system_->ClearViewerCustomPoints();
            return;
        }

        const auto points =
            computeCustomPlacedPoints(observations, Twc_raw_by_frame, samples, alignment);
        if (points.empty())
            slam_system_->ClearViewerCustomPoints();
        else
            slam_system_->SetViewerCustomPoints(points);
    }

    struct PoseLookupResult {
        bool valid = false;
        Sophus::SE3f Twc;
        uint64_t frame_id = 0;
        uint64_t frame_id_after = 0;
        bool interpolated = false;
        float alpha = 0.0f;
    };

    Sophus::SE3f interpolateTwc(const Sophus::SE3f& a,
                                const Sophus::SE3f& b,
                                float alpha) const {
        alpha = std::max(0.0f, std::min(1.0f, alpha));
        Eigen::Quaternionf qa = a.so3().unit_quaternion();
        Eigen::Quaternionf qb = b.so3().unit_quaternion();
        if (qa.dot(qb) < 0.0f)
            qb.coeffs() *= -1.0f;
        Eigen::Quaternionf q = qa.slerp(alpha, qb);
        q.normalize();
        const Eigen::Vector3f t =
            (1.0f - alpha) * a.translation() + alpha * b.translation();
        return Sophus::SE3f(q.toRotationMatrix(), t);
    }

    bool poseByTimestamp(const vector<ORB_SLAM3::System::TrajectorySample>& samples,
                         double timestamp,
                         PoseLookupResult& pose) const {
        if (!std::isfinite(timestamp) || samples.empty())
            return false;

        const ORB_SLAM3::System::TrajectorySample* before = nullptr;
        const ORB_SLAM3::System::TrajectorySample* after = nullptr;
        const ORB_SLAM3::System::TrajectorySample* nearest = nullptr;
        double nearest_dt = numeric_limits<double>::infinity();
        for (const auto& sample : samples) {
            if (!std::isfinite(sample.timestamp))
                continue;

            if (sample.timestamp <= timestamp &&
                (!before || sample.timestamp > before->timestamp)) {
                before = &sample;
            }
            if (sample.timestamp >= timestamp &&
                (!after || sample.timestamp < after->timestamp)) {
                after = &sample;
            }

            const double dt = std::fabs(sample.timestamp - timestamp);
            if (dt < nearest_dt) {
                nearest_dt = dt;
                nearest = &sample;
            }
        }

        pose = PoseLookupResult();
        if (before && after) {
            const double span = after->timestamp - before->timestamp;
            if (before->id == after->id || std::fabs(span) < 1e-9) {
                pose.valid = true;
                pose.Twc = before->Twc;
                pose.frame_id = before->id;
                return true;
            }

            const float alpha = static_cast<float>((timestamp - before->timestamp) / span);
            pose.valid = true;
            pose.Twc = interpolateTwc(before->Twc, after->Twc, alpha);
            pose.frame_id = before->id;
            pose.frame_id_after = after->id;
            pose.interpolated = true;
            pose.alpha = std::max(0.0f, std::min(1.0f, alpha));
            return true;
        }

        if (!nearest)
            return false;
        pose.valid = true;
        pose.Twc = nearest->Twc;
        pose.frame_id = nearest->id;
        return true;
    }

    const Sophus::SE3f* poseAtOrBeforeFrame(
        const unordered_map<uint64_t, Sophus::SE3f>& Twc_raw_by_frame,
        uint64_t frame_id,
        uint64_t* pose_frame_id = nullptr) const {
        const auto exact_it = Twc_raw_by_frame.find(frame_id);
        if (exact_it != Twc_raw_by_frame.end()) {
            if (pose_frame_id)
                *pose_frame_id = exact_it->first;
            return &exact_it->second;
        }

        const Sophus::SE3f* best_pose = nullptr;
        uint64_t best_frame_id = 0;
        for (const auto& kv : Twc_raw_by_frame) {
            if (kv.first >= frame_id)
                continue;
            if (!best_pose || kv.first > best_frame_id) {
                best_pose = &kv.second;
                best_frame_id = kv.first;
            }
        }

        if (best_pose && pose_frame_id)
            *pose_frame_id = best_frame_id;
        return best_pose;
    }

    bool poseForCustomDetection(
        const CustomDetectionObservation& obs,
        const unordered_map<uint64_t, Sophus::SE3f>& Twc_raw_by_frame,
        const vector<ORB_SLAM3::System::TrajectorySample>& samples,
        PoseLookupResult& pose) const {
        const auto exact_it = Twc_raw_by_frame.find(obs.frame_id);
        if (exact_it != Twc_raw_by_frame.end()) {
            pose = PoseLookupResult();
            pose.valid = true;
            pose.Twc = exact_it->second;
            pose.frame_id = exact_it->first;
            return true;
        }

        if (obs.has_capture_timestamp && poseByTimestamp(samples, obs.timestamp, pose))
            return true;

        uint64_t pose_frame_id = 0;
        const Sophus::SE3f* Twc =
            poseAtOrBeforeFrame(Twc_raw_by_frame, obs.frame_id, &pose_frame_id);
        if (!Twc)
            return false;

        pose = PoseLookupResult();
        pose.valid = true;
        pose.Twc = *Twc;
        pose.frame_id = pose_frame_id;
        return true;
    }

    json portalFrameDetectionsToJson(
        const vector<PortalCornerObservation>& observations,
        const unordered_map<uint64_t, Sophus::SE3f>& Twc_raw_by_frame) const {
        json rows = json::array();
        for (const auto& obs : observations) {
            if (obs.image_points.size() != 4)
                continue;
            uint64_t pose_frame_id = 0;
            const bool has_pose_frame =
                poseAtOrBeforeFrame(Twc_raw_by_frame, obs.frame_id, &pose_frame_id) != nullptr;
            for (size_t i = 0; i < obs.image_points.size(); ++i) {
                const cv::Point2f& p = obs.image_points[i];
                json row = {
                    {"frame_index", obs.frame_id},
                    {"type", "portal_corner"},
                    {"id", obs.short_id + "_" + to_string(i)},
                    {"pixel", {p.x, p.y}},
                };
                if (has_pose_frame)
                    row["pose_frame_index"] = pose_frame_id;
                rows.push_back(row);
            }
        }
        return rows;
    }

    json customFrameDetectionsToJson(
        const vector<CustomDetectionObservation>& observations,
        const unordered_map<uint64_t, Sophus::SE3f>& Twc_raw_by_frame,
        const vector<ORB_SLAM3::System::TrajectorySample>& samples) const {
        json rows = json::array();
        for (const auto& obs : observations) {
            PoseLookupResult pose;
            const bool has_pose_frame =
                poseForCustomDetection(obs, Twc_raw_by_frame, samples, pose);
            json row = {
                {"frame_index", obs.frame_id},
                {"type", obs.type},
                {"id", obs.id},
                {"pixel", {obs.pixel.x, obs.pixel.y}},
            };
            if (has_pose_frame) {
                row["pose_frame_index"] = pose.frame_id;
                if (pose.interpolated) {
                    row["pose_frame_index_after"] = pose.frame_id_after;
                    row["pose_alpha"] = pose.alpha;
                }
            }
            rows.push_back(row);
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

    Eigen::Vector3f cameraRayFromPixel(const vector<float>& intrinsics,
                                       const cv::Point2f& pixel) const {
        const float x = (pixel.x - intrinsics[2]) / intrinsics[0];
        const float y = (pixel.y - intrinsics[3]) / intrinsics[1];
        return Eigen::Vector3f(x, y, 1.0f).normalized();
    }

    bool triangulateRays(const vector<Eigen::Vector3f>& origins,
                         const vector<Eigen::Vector3f>& directions,
                         Eigen::Vector3f& point) const {
        if (origins.size() < 2 || origins.size() != directions.size())
            return false;

        Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
        Eigen::Vector3f b = Eigen::Vector3f::Zero();
        for (size_t i = 0; i < origins.size(); ++i) {
            const Eigen::Vector3f d = directions[i].normalized();
            const Eigen::Matrix3f P = Eigen::Matrix3f::Identity() - d * d.transpose();
            A += P;
            b += P * origins[i];
        }

        Eigen::LDLT<Eigen::Matrix3f> solver(A);
        if (solver.info() != Eigen::Success)
            return false;

        point = solver.solve(b);
        if (!point.allFinite())
            return false;

        int positive_depths = 0;
        for (size_t i = 0; i < origins.size(); ++i) {
            const float depth = directions[i].normalized().dot(point - origins[i]);
            if (std::isfinite(depth) && depth > 0.0f)
                positive_depths++;
        }
        return positive_depths >= 2;
    }

    bool intersectRayTriangle(const Eigen::Vector3f& origin,
                              const Eigen::Vector3f& direction,
                              const ORB_SLAM3::MapDrawer::MeshTriangle& triangle,
                              float& t) const {
        const Eigen::Vector3f v0(
            triangle.vertices[0][0], triangle.vertices[0][1], triangle.vertices[0][2]);
        const Eigen::Vector3f v1(
            triangle.vertices[1][0], triangle.vertices[1][1], triangle.vertices[1][2]);
        const Eigen::Vector3f v2(
            triangle.vertices[2][0], triangle.vertices[2][1], triangle.vertices[2][2]);

        constexpr float kEpsilon = 1e-6f;
        const Eigen::Vector3f edge1 = v1 - v0;
        const Eigen::Vector3f edge2 = v2 - v0;
        const Eigen::Vector3f pvec = direction.cross(edge2);
        const float det = edge1.dot(pvec);
        if (std::fabs(det) < kEpsilon)
            return false;

        const float inv_det = 1.0f / det;
        const Eigen::Vector3f tvec = origin - v0;
        const float u = tvec.dot(pvec) * inv_det;
        if (u < 0.0f || u > 1.0f)
            return false;

        const Eigen::Vector3f qvec = tvec.cross(edge1);
        const float v = direction.dot(qvec) * inv_det;
        if (v < 0.0f || u + v > 1.0f)
            return false;

        t = edge2.dot(qvec) * inv_det;
        return std::isfinite(t) && t > 1e-4f;
    }

    bool raycastOcclusionMesh(const Eigen::Vector3f& origin_domain,
                              const Eigen::Vector3f& direction_domain,
                              Eigen::Vector3f& hit_domain) const {
        if (config_.occlusion_mesh.triangles.empty() ||
            !origin_domain.allFinite() || !direction_domain.allFinite()) {
            return false;
        }

        const Eigen::Vector3f direction = direction_domain.normalized();
        if (!direction.allFinite())
            return false;

        bool hit = false;
        float closest_t = numeric_limits<float>::infinity();
        for (const auto& triangle : config_.occlusion_mesh.triangles) {
            float t = 0.0f;
            if (intersectRayTriangle(origin_domain, direction, triangle, t) &&
                t < closest_t) {
                closest_t = t;
                hit = true;
            }
        }
        if (!hit)
            return false;

        hit_domain = origin_domain + closest_t * direction;
        return hit_domain.allFinite();
    }

    bool meshHitForCustomDetection(
        const CustomDetectionObservation& obs,
        const unordered_map<uint64_t, Sophus::SE3f>& Twc_raw_by_frame,
        const vector<ORB_SLAM3::System::TrajectorySample>& samples,
        const AlignmentSnapshot& alignment,
        Eigen::Vector3f& hit_domain) const {
        PoseLookupResult pose;
        if (!poseForCustomDetection(obs, Twc_raw_by_frame, samples, pose) ||
            obs.intrinsics.size() < 4) {
            return false;
        }

        const Eigen::Vector3f ray_cam =
            cameraRayFromPixel(obs.intrinsics, obs.pixel);
        const Eigen::Vector3f origin_slam = pose.Twc.translation();
        const Eigen::Vector3f direction_slam =
            (pose.Twc.so3().matrix() * ray_cam).normalized();
        const Eigen::Vector3f origin_domain =
            alignment.scale * (alignment.R_domain_slam * origin_slam) +
            alignment.t_domain_slam;
        const Eigen::Vector3f direction_domain =
            (alignment.R_domain_slam * direction_slam).normalized();

        return raycastOcclusionMesh(origin_domain, direction_domain, hit_domain);
    }

    bool placeDetectionGroupWithMesh(
        const vector<const CustomDetectionObservation*>& observations,
        const unordered_map<uint64_t, Sophus::SE3f>& Twc_raw_by_frame,
        const vector<ORB_SLAM3::System::TrajectorySample>& samples,
        const AlignmentSnapshot& alignment,
        Eigen::Vector3f& point_domain) const {
        if (!config_.occlusion_mesh.hasTriangles() ||
            !alignment.enabled || !alignment.has_transform ||
            !isValidScale(alignment.scale)) {
            return false;
        }

        if (observations.empty() || !observations.back())
            return false;

        return meshHitForCustomDetection(
            *observations.back(), Twc_raw_by_frame, samples, alignment, point_domain);
    }

    vector<ORB_SLAM3::MapDrawer::CustomPoint> computeCustomPlacedPoints(
        const vector<CustomDetectionObservation>& observations,
        const unordered_map<uint64_t, Sophus::SE3f>& Twc_raw_by_frame,
        const vector<ORB_SLAM3::System::TrajectorySample>& samples,
        const AlignmentSnapshot& alignment) const {
        vector<ORB_SLAM3::MapDrawer::CustomPoint> points;

        struct DetectionGroup {
            string type;
            string id;
            vector<const CustomDetectionObservation*> observations;
        };

        unordered_map<string, DetectionGroup> groups;
        vector<string> order;
        for (const auto& obs : observations) {
            if (obs.type.empty() || obs.id.empty() || obs.intrinsics.size() < 4)
                continue;
            const string key = obs.type + "\n" + obs.id;
            if (groups.find(key) == groups.end()) {
                DetectionGroup group;
                group.type = obs.type;
                group.id = obs.id;
                groups[key] = group;
                order.push_back(key);
            }
            groups[key].observations.push_back(&obs);
        }

        for (const string& key : order) {
            const DetectionGroup& group = groups[key];
            Eigen::Vector3f p_out;
            bool placed = placeDetectionGroupWithMesh(
                group.observations, Twc_raw_by_frame, samples, alignment, p_out);

            vector<Eigen::Vector3f> origins;
            vector<Eigen::Vector3f> directions;
            if (!placed) {
                for (const CustomDetectionObservation* obs : group.observations) {
                    PoseLookupResult pose;
                    if (!obs || !poseForCustomDetection(*obs, Twc_raw_by_frame, samples, pose))
                        continue;

                    const Eigen::Vector3f ray_cam =
                        cameraRayFromPixel(obs->intrinsics, obs->pixel);
                    origins.push_back(pose.Twc.translation());
                    directions.push_back((pose.Twc.so3().matrix() * ray_cam).normalized());
                }

                Eigen::Vector3f p_slam;
                if (!triangulateRays(origins, directions, p_slam))
                    continue;

                p_out = (alignment.enabled && alignment.has_transform)
                    ? alignment.scale * (alignment.R_domain_slam * p_slam) +
                          alignment.t_domain_slam
                    : p_slam;
                placed = true;
            }

            ORB_SLAM3::MapDrawer::CustomPoint point;
            point.type = group.type;
            point.id = group.id;
            point.point = {p_out.x(), p_out.y(), p_out.z()};
            point.diameter_m = 0.1f;
            points.push_back(point);
        }
        return points;
    }

    json customPlacedDataToJson(
        const vector<ORB_SLAM3::MapDrawer::CustomPoint>& points) const {
        json rows = json::array();
        for (const auto& point : points) {
            rows.push_back({
                {"type", point.type},
                {"id", point.id},
                {"point", {point.point[0], point.point[1], point.point[2]}},
            });
        }
        return rows;
    }

    void appendTopographyMeshPlacedData(json& rows, const AlignmentSnapshot& alignment) const {
        if (config_.occlusion_mesh.obj_content.empty() ||
            !alignment.enabled || !alignment.has_transform)
            return;
        rows.push_back({
            {"type", "topography_mesh_obj"},
            {"obj", config_.occlusion_mesh.obj_content},
        });
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
                if (slam_dist > kMinSlamLocalPointPairDistance && metric_dist > 1e-4f)
                    ratios.push_back(metric_dist / slam_dist);
            }
        }

        if (ratios.empty() && slam_cam_points.size() == 1) {
            const float slam_dist = slam_cam_points.front().norm();
            const float metric_dist = metric_cam_points.front().norm();
            if (slam_dist > kMinSlamLocalPointPairDistance && metric_dist > 1e-4f)
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

    void syncActiveMapAlignmentState(unsigned long current_map_id) {
        lock_guard<mutex> lock(alignment_mutex_);
        if (last_synced_map_id_ == current_map_id)
            return;

        const unsigned long previous_map_id = last_synced_map_id_;
        last_synced_map_id_ = current_map_id;

        if (previous_map_id != numeric_limits<unsigned long>::max() &&
            !overlay_graph_.isMapPinned(current_map_id)) {
            overlay_graph_.clearMapAlignment(current_map_id);
            map_alignments_.erase(current_map_id);
        }

        const auto align_it = map_alignments_.find(current_map_id);
        if (align_it != map_alignments_.end() && align_it->second.has_transform) {
            alignment_ = align_it->second;
            alignment_.enabled = !portals_.empty();
            const overlay::MapSim3Pin overlay_pin = overlay_graph_.mapPin(current_map_id);
            if (overlay_pin.pinned) {
                alignment_.portal_observations = overlay_pin.portal_corner_factor_count / 4;
            }
            return;
        }

        const overlay::MapSim3Pin overlay_pin = overlay_graph_.mapPin(current_map_id);
        if (overlay_pin.pinned && overlay_pin.seeded) {
            alignment_.enabled = !portals_.empty();
            alignment_.has_transform = true;
            alignment_.R_domain_slam = overlay_pin.R_domain_slam;
            alignment_.t_domain_slam = overlay_pin.t_domain_slam;
            alignment_.scale = overlay_pin.scale;
            alignment_.scale_source = overlay_pin.scale_source;
            alignment_.last_portal_short_id = overlay_pin.last_portal_short_id;
            alignment_.last_reproj_px = overlay_pin.last_reproj_px;
            alignment_.portal_observations = overlay_pin.portal_corner_factor_count / 4;
            alignment_.status = "aligned_refining";
            map_alignments_[current_map_id] = alignment_;
            return;
        }

        alignment_.enabled = !portals_.empty();
        alignment_.has_transform = false;
        alignment_.R_domain_slam = Eigen::Matrix3f::Identity();
        alignment_.t_domain_slam = Eigen::Vector3f::Zero();
        alignment_.scale = 0.0f;
        alignment_.has_scale_update_observation = false;
        alignment_.last_scale_update_p_slam = Eigen::Vector3f::Zero();
        alignment_.last_scale_update_p_domain = Eigen::Vector3f::Zero();
        alignment_.scale_source = "none";
        alignment_.last_portal_short_id.clear();
        alignment_.last_reproj_px = 0.0f;
        alignment_.last_local_points_used = 0;
        alignment_.portal_observations = 0;
        alignment_.status = portals_.empty()
            ? "disabled"
            : "portal_manifest_loaded_waiting_for_detection";
    }

    AlignmentSnapshot mapAlignmentSnapshot(unsigned long map_id, bool request_aligned) const {
        lock_guard<mutex> lock(alignment_mutex_);
        AlignmentSnapshot snapshot;
        const auto it = map_alignments_.find(map_id);
        if (it != map_alignments_.end())
            snapshot = it->second;
        if (!request_aligned) {
            snapshot.enabled = false;
            snapshot.has_transform = false;
            snapshot.status = portals_.empty() ? "disabled" : "raw_requested";
        }
        return snapshot;
    }

    void clearDomainAlignmentForMap(unsigned long map_id) {
        overlay_graph_.clearMapAlignment(map_id);
        lock_guard<mutex> lock(alignment_mutex_);
        map_alignments_.erase(map_id);
        if (last_synced_map_id_ != map_id)
            return;

        alignment_.enabled = !portals_.empty();
        alignment_.has_transform = false;
        alignment_.R_domain_slam = Eigen::Matrix3f::Identity();
        alignment_.t_domain_slam = Eigen::Vector3f::Zero();
        alignment_.scale = 0.0f;
        alignment_.has_scale_update_observation = false;
        alignment_.last_scale_update_p_slam = Eigen::Vector3f::Zero();
        alignment_.last_scale_update_p_domain = Eigen::Vector3f::Zero();
        alignment_.scale_source = "none";
        alignment_.last_portal_short_id.clear();
        alignment_.last_reproj_px = 0.0f;
        alignment_.last_local_points_used = 0;
        alignment_.portal_observations = 0;
        alignment_.status = portals_.empty()
            ? "disabled"
            : "tracking_lost_domain_cleared";
    }

    void applyOverlayRefineToAlignment(unsigned long map_id,
                                       const overlay::RefineResult& refine) {
        if (!refine.published)
            return;

        lock_guard<mutex> lock(alignment_mutex_);
        alignment_.enabled = !portals_.empty();
        alignment_.has_transform = true;
        alignment_.R_domain_slam = refine.pin.R_domain_slam;
        alignment_.t_domain_slam = refine.pin.t_domain_slam;
        alignment_.scale = refine.pin.scale;
        alignment_.scale_source = refine.pin.scale_source;
        alignment_.status = refine.status;
        alignment_.has_scale_update_observation = true;
        alignment_.portal_observations = refine.pin.portal_corner_factor_count / 4;
        map_alignments_[map_id] = alignment_;

        if (slam_system_) {
            slam_system_->SetViewerAlignment(
                refine.pin.R_domain_slam, refine.pin.t_domain_slam, refine.pin.scale);
        }
    }

    MultiMapPublishPlan buildMultiMapPublishPlan(unsigned long active_map_id) const {
        MultiMapPublishPlan plan;
        plan.active_map_id = active_map_id;

        lock_guard<mutex> lock(alignment_mutex_);
        for (const auto& kv : map_alignments_) {
            if (mapAlignmentIsDomainPinned(kv.second))
                plan.pinned_map_ids.push_back(kv.first);
        }
        sort(plan.pinned_map_ids.begin(), plan.pinned_map_ids.end());

        const auto active_it = map_alignments_.find(active_map_id);
        plan.active_map_domain_aligned =
            active_it != map_alignments_.end() &&
            mapAlignmentIsDomainPinned(active_it->second);

        if (!plan.active_map_domain_aligned) {
            plan.policy = "active_only";
            plan.included_map_ids = {active_map_id};
            for (unsigned long pinned_id : plan.pinned_map_ids) {
                if (pinned_id != active_map_id)
                    plan.withheld_map_ids.push_back(pinned_id);
            }
            return plan;
        }

        plan.policy = "domain_pinned_merge";
        plan.included_map_ids = plan.pinned_map_ids;
        return plan;
    }

    void updatePortalAlignment(const PortalPnPResult& pnp,
                               const Sophus::SE3f& Tcw_slam,
                               const LocalScaleEstimate& local_scale,
                               uint64_t frame_id,
                               double timestamp,
                               unsigned long map_id,
                               const vector<float>& intrinsics) {
        syncActiveMapAlignmentState(map_id);

        auto portal_it = portals_.find(pnp.short_id);
        if (portal_it == portals_.end() || intrinsics.size() < 4)
            return;

        const PortalDefinition& portal = portal_it->second;
        const Sophus::SE3f Twc_slam = Tcw_slam.inverse();
        const Sophus::SE3f Twc_domain = pnp.T_cam_domain.inverse();
        const Eigen::Matrix3f R_domain_slam =
            Twc_domain.so3().matrix() * Twc_slam.so3().matrix().transpose();

        overlay::PortalSightInput sight;
        sight.frame_id = frame_id;
        sight.timestamp = timestamp;
        sight.short_id = pnp.short_id;
        sight.Twc_slam = Twc_slam;
        sight.Twc_domain = Twc_domain;
        sight.R_domain_slam = R_domain_slam;
        sight.fx = intrinsics[0];
        sight.fy = intrinsics[1];
        sight.cx = intrinsics[2];
        sight.cy = intrinsics[3];
        sight.reproj_px = pnp.mean_reproj_px;
        sight.physical_size_m = pnp.physical_size_m;
        sight.metric_depth_m = pnp.T_cam_portal.translation().norm();

        const vector<cv::Point3f> object_points = portalObjectPoints(portal.physical_size_m);
        const Eigen::Matrix3f R_domain_portal = portal.T_domain_portal.so3().matrix();
        const Eigen::Vector3f t_domain_portal = portal.T_domain_portal.translation();
        for (size_t i = 0; i < 4 && i < object_points.size(); ++i) {
            const cv::Point3f& obj = object_points[i];
            const Eigen::Vector3f p_portal(obj.x, obj.y, obj.z);
            sight.corners_domain[i] = R_domain_portal * p_portal + t_domain_portal;
            if (i < pnp.image_points.size()) {
                sight.corners_pixel[i] =
                    Eigen::Vector2f(pnp.image_points[i].x, pnp.image_points[i].y);
            }
        }

        cout << "[portal] sight frame=" << frame_id
             << " id=" << sight.short_id
             << " reproj_px=" << sight.reproj_px
             << " local_scale_valid=" << (local_scale.valid ? 1 : 0)
             << " local_scale=" << local_scale.scale
             << endl;

        overlay::RefineResult refine = overlay_graph_.ingestPortalSight(map_id, sight);

        bool publish_viewer_alignment = false;
        Eigen::Matrix3f viewer_R_domain_slam = Eigen::Matrix3f::Identity();
        Eigen::Vector3f viewer_t_domain_slam = Eigen::Vector3f::Zero();
        float viewer_scale = 1.0f;

        {
            lock_guard<mutex> lock(alignment_mutex_);
            alignment_.enabled = true;
            alignment_.last_portal_short_id = sight.short_id;
            alignment_.last_reproj_px = sight.reproj_px;
            alignment_.last_local_points_used = local_scale.points_used;
            alignment_.portal_observations = refine.pin.portal_corner_factor_count / 4;

            if (!refine.published) {
                alignment_.has_transform = false;
                alignment_.scale = 0.0f;
                alignment_.scale_source = "none";
                alignment_.status = refine.status;
                map_alignments_[map_id] = alignment_;
                cout << "[portal] overlay accumulating via " << sight.short_id
                     << " corner_factors=" << refine.pin.portal_corner_factor_count
                     << " live_edges=" << refine.pin.live_relative_edge_count
                     << " status=" << refine.status
                     << " scale_update=" << refine.scale_update_status << endl;
            } else {
                alignment_.has_transform = true;
                alignment_.R_domain_slam = refine.pin.R_domain_slam;
                alignment_.t_domain_slam = refine.pin.t_domain_slam;
                alignment_.scale = refine.pin.scale;
                alignment_.scale_source = refine.pin.scale_source;
                alignment_.status = refine.status;
                alignment_.has_scale_update_observation = true;
                alignment_.last_scale_update_p_slam = Twc_slam.translation();
                alignment_.last_scale_update_p_domain = Twc_domain.translation();
                map_alignments_[map_id] = alignment_;

                publish_viewer_alignment = true;
                viewer_R_domain_slam = refine.pin.R_domain_slam;
                viewer_t_domain_slam = refine.pin.t_domain_slam;
                viewer_scale = refine.pin.scale;

                cout << "[portal] overlay " << refine.status << " via " << sight.short_id
                     << " reproj_px=" << sight.reproj_px
                     << " scale=" << refine.pin.scale
                     << " scale_source=" << refine.pin.scale_source
                     << " scale_update=" << refine.scale_update_status
                     << " corner_factors=" << refine.pin.portal_corner_factor_count
                     << " live_edges=" << refine.pin.live_relative_edge_count
                     << " corner_rmse_px=" << refine.pin.last_corner_rmse_px
                     << " opt_passes=" << refine.pin.optimization_passes << endl;
                cout << "[overlay] map " << map_id << " pinned via portal " << sight.short_id
                     << " scale=" << refine.pin.scale
                     << " scale_source=" << refine.pin.scale_source
                     << " portal_corner_factors=" << refine.pin.portal_corner_factor_count
                     << " reproj_px=" << sight.reproj_px << endl;
                if (prior_pack_.loaded) {
                    last_prior_seed_image_id_ =
                        prior_soft::PriorSoftEdgeMatcher::nearestPriorImageId(prior_pack_, Twc_domain);
                    cout << "[prior] covis seed image=" << last_prior_seed_image_id_
                         << " after portal PnP" << endl;
                }
            }
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
                "Content-Type, X-Intrinsics, X-Client-ID, X-Capture-Timestamp, X-Frame-Id, X-Run-Detector");
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
            // Reject before JPEG decode when the tracking queue is already full.
            // Unbounded phone clients + ngrok otherwise burn HTTP worker threads decoding
            // frames that will only be dropped, which makes the stream UI appear frozen.
            {
                lock_guard<mutex> lock(queue_mutex_);
                if (image_queue_.size() >= static_cast<size_t>(config_.max_queue_size)) {
                    response["success"] = false;
                    response["error"] = "Server queue full, try again later";
                    res.set_content(response.dump(), "application/json");
                    res.status = 503;
                    return;
                }
            }

            // Parse request
            string parse_error;
            auto image_req = parseImageRequest(req, &parse_error);
            if (!image_req) {
                response["success"] = false;
                response["error"] = parse_error.empty() ? "Failed to parse image request" : parse_error;
                res.set_content(response.dump(), "application/json");
                res.status = 400;
                return;
            }

            const uint64_t accepted_frame_index = total_requests_.load() + 1;
            image_req->accepted_index = accepted_frame_index;
            cout << "[track] incoming frame " << image_req->frame_id
                 << " image_kib=" << formatKiB(image_req->encoded_image_bytes)
                 << " decoded=" << image_req->image.cols << "x" << image_req->image.rows
                 << " timestamp=" << image_req->timestamp
                 << " timestamp_source=" << (image_req->has_capture_timestamp ? "client" : "server")
                 << " content_type=" << image_req->content_type
                 << endl;
            const bool should_queue_detector = shouldRunCustomDetection(image_req);
            bool detector_queued = false;
            const bool detector_only_still = shouldTreatAsDetectorOnlyStill(image_req);
            if (detector_only_still) {
                detector_queued = enqueueCustomDetection(image_req);
                total_requests_++;
                skipped_tracks_++;

                response["success"] = detector_queued;
                response["message"] = detector_queued
                    ? "Detector-only still accepted"
                    : "Detector-only still rejected because the detector queue is unavailable";
                response["frame_id"] = image_req->frame_id;
                response["timestamp"] = image_req->timestamp;
                response["tracked"] = false;
                response["detector_only"] = true;
                response["detector_queued"] = detector_queued;
                if (!detector_queued)
                    res.status = 409;

                res.set_content(response.dump(), "application/json");
                return;
            }

            if (!shouldRunSlamForAcceptedFrame(accepted_frame_index)) {
                detector_queued = should_queue_detector && enqueueCustomDetection(image_req);
                total_requests_++;
                skipped_tracks_++;

                response["success"] = true;
                response["message"] = "Image accepted but skipped by SLAM frame interval";
                response["frame_id"] = image_req->frame_id;
                response["timestamp"] = image_req->timestamp;
                response["tracked"] = false;
                response["slam_interval_nth"] = config_.slam_interval_nth;
                response["detector_queued"] = detector_queued;

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
            detector_queued = should_queue_detector && enqueueCustomDetection(image_req);
            
            // For now, return immediate response
            // TODO: Implement synchronous processing or result polling
            response["success"] = true;
            response["message"] = "Image queued for processing";
            response["frame_id"] = image_req->frame_id;
            response["timestamp"] = image_req->timestamp;
            response["tracked"] = true;
            response["detector_queued"] = detector_queued;
            
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
        response["custom_detect_every"] = config_.custom_detect_every;
        response["custom_detection_enqueued"] = custom_detection_enqueued_.load();
        response["custom_detection_dropped"] = custom_detection_dropped_.load();
        response["custom_detection_processed"] = custom_detection_processed_.load();
        {
            lock_guard<mutex> lock(custom_detection_queue_mutex_);
            response["custom_detection_queue_size"] = custom_detection_queue_.size();
        }
        response["avg_processing_time_ms"] = avg_processing_time_.load() * 1000.0;
        response["timing"] = stage_timing_.toJson();
        
        if (slam_system_) {
            response["tracking_state"] = slam_system_->GetTrackingState();
            response["is_lost"] = slam_system_->isLost();
        }

        response["alignment"] = alignmentSnapshotToJson(getAlignmentSnapshot(true));

        unsigned long active_map_id = 0;
        if (slam_system_) {
            lock_guard<mutex> slam_lock(slam_mutex_);
            active_map_id = slam_system_->GetCurrentMapId();
            syncActiveMapAlignmentState(active_map_id);
            response["multi_map_publish"] = multiMapPublishToJson(
                buildMultiMapPublishPlan(active_map_id));
            response["map_id"] = active_map_id;
        }

        response["overlay"] = overlay_graph_.statusJson(active_map_id);

        const LatestSlamFrame latest = latestSlamFrameSnapshot();
        if (latest.valid) {
            const overlay::NavPoseResult nav =
                overlay_graph_.navPoseFromSlam(active_map_id, latest.Tcw_slam.inverse());
            if (nav.valid) {
                response["nav_pose"] = TwcToClientJson(
                    nav.Twc, latest.timestamp, latest.frame_id, false);
                response["pose_source"] = nav.pose_source;
            }
        }

        response["prior_pack"] = prior_pack::prior_pack_status_json(prior_pack_);
        
        res.set_content(response.dump(), "application/json");
    }
    
    void handleTrajectoryRequest(const httplib::Request& req, httplib::Response& res) {
        json response;
        json trajectory_json = json::array();

        const bool dense = trajectoryUseDense(req);
        const bool request_aligned = !requestParamIsFalse(req, "aligned");
        const bool all_maps =
            req.has_param("all_maps") && !requestParamIsFalse(req, "all_maps");

        unsigned long current_map_id = 0;
        MultiMapPublishPlan publish_plan;
        vector<ORB_SLAM3::System::TrajectorySample> active_samples;
        unordered_map<uint64_t, Sophus::SE3f> Twc_raw_by_frame;

        if (slam_system_) {
            lock_guard<mutex> slam_lock(slam_mutex_);
            current_map_id = slam_system_->GetCurrentMapId();
            syncActiveMapAlignmentState(current_map_id);
            publish_plan = buildMultiMapPublishPlan(current_map_id);

            vector<unsigned long> map_ids = publish_plan.included_map_ids;
            if (all_maps) {
                map_ids = slam_system_->GetAtlasMapIds();
                // Eval export: include every atlas map even when publish policy withholds them.
            }

            for (unsigned long map_id : map_ids) {
                vector<ORB_SLAM3::System::TrajectorySample> samples;
                const bool is_active = map_id == current_map_id;
                if (is_active) {
                    if (dense)
                        slam_system_->GetActiveMapDenseTrajectory(samples);
                    else
                        slam_system_->GetActiveMapTrajectory(samples, true);
                } else {
                    if (dense)
                        slam_system_->GetMapDenseTrajectory(map_id, samples);
                    else
                        slam_system_->GetMapKeyFrameTrajectory(map_id, samples);
                }

                const AlignmentSnapshot map_alignment =
                    request_aligned ? mapAlignmentSnapshot(map_id, true) : mapAlignmentSnapshot(map_id, false);

                for (const auto& sample : samples) {
                    if (is_active)
                        Twc_raw_by_frame[sample.id] = sample.Twc;

                    // all_maps=1 is an eval export: apply each map's pin when present.
                    // Live publish policy still withholds unpinned maps unless active_only.
                    const bool apply_domain =
                        request_aligned && mapAlignmentIsDomainPinned(map_alignment) &&
                        (all_maps || publish_plan.policy == "domain_pinned_merge" ||
                         (publish_plan.policy == "active_only" && is_active));
                    const Sophus::SE3f Twc = apply_domain
                        ? overlay_graph_.applyPinToTwc(map_id, sample.Twc)
                        : sample.Twc;

                    json pose_json = TwcToClientJson(
                        Twc, sample.timestamp, sample.id, sample.is_keyframe);
                    pose_json["map_id"] = map_id;
                    pose_json["pose_source"] = apply_domain ? "overlay" : "slam";
                    trajectory_json.push_back(pose_json);
                }
            }

            if (dense)
                slam_system_->GetActiveMapDenseTrajectory(active_samples);
            else
                slam_system_->GetActiveMapTrajectory(active_samples, true);
            if (Twc_raw_by_frame.empty()) {
                for (const auto& sample : active_samples)
                    Twc_raw_by_frame[sample.id] = sample.Twc;
            }
        }

        const AlignmentSnapshot alignment = getAlignmentSnapshot(request_aligned);

        const vector<PortalCornerObservation> portal_corner_observations =
            portalCornerObservationsSnapshot();
        const vector<CustomDetectionObservation> custom_detection_observations =
            customDetectionObservationsSnapshot();
        const vector<ORB_SLAM3::MapDrawer::PortalSquare> portal_squares =
            computePortalSquares(portal_corner_observations, Twc_raw_by_frame, alignment);
        const vector<ORB_SLAM3::MapDrawer::CustomPoint> custom_points =
            computeCustomPlacedPoints(
                custom_detection_observations, Twc_raw_by_frame, active_samples, alignment);
        json placed_data = placedDataToJson(portal_squares);
        for (const auto& row : customPlacedDataToJson(custom_points)) {
            placed_data.push_back(row);
        }
        appendTopographyMeshPlacedData(placed_data, alignment);

        json frame_detections =
            portalFrameDetectionsToJson(portal_corner_observations, Twc_raw_by_frame);
        for (const auto& row :
             customFrameDetectionsToJson(
                 custom_detection_observations, Twc_raw_by_frame, active_samples)) {
            frame_detections.push_back(row);
        }

        response["trajectory"] = trajectory_json;
        response["count"] = trajectory_json.size();
        response["placed_data"] = placed_data;
        response["frame_detections"] = frame_detections;
        response["map_id"] = current_map_id;
        response["multi_map_publish"] = multiMapPublishToJson(publish_plan);
        response["dense"] = dense;
        response["all_maps"] = all_maps;
        if (all_maps) {
            response["source"] = dense
                ? "all_atlas_maps_dense_frames"
                : "all_atlas_maps_keyframes";
        } else if (publish_plan.policy == "domain_pinned_merge") {
            response["source"] = dense
                ? "multi_map_domain_pinned_merge_dense_frames"
                : "multi_map_domain_pinned_merge_keyframes";
        } else {
            response["source"] = dense ? "active_map_dense_frames" : "active_map_keyframes";
        }
        const bool domain_output =
            request_aligned &&
            (publish_plan.policy == "domain_pinned_merge" ||
             (publish_plan.policy == "active_only" && publish_plan.active_map_domain_aligned));
        response["coordinate_frame"] = domain_output ? "domain_colmap" : "slam";
        response["pose_source"] = domain_output ? "overlay" : "slam";
        response["alignment"] = alignmentSnapshotToJson(alignment);
        response["overlay"] = overlay_graph_.statusJson(current_map_id);
        if (!active_samples.empty()) {
            const overlay::NavPoseResult nav =
                overlay_graph_.navPoseFromSlam(current_map_id, active_samples.back().Twc);
            if (nav.valid) {
                const auto& latest_sample = active_samples.back();
                response["nav_pose"] = TwcToClientJson(
                    nav.Twc,
                    latest_sample.timestamp,
                    latest_sample.id,
                    latest_sample.is_keyframe);
            }
        }
        response["custom_detection"] = {
            {"detect_every", config_.custom_detect_every},
            {"enqueued", custom_detection_enqueued_.load()},
            {"dropped", custom_detection_dropped_.load()},
            {"processed", custom_detection_processed_.load()},
            {"stored_observations", custom_detection_observations.size()},
        };

        if (request_aligned && slam_system_) {
            slam_system_->SetViewerPortalSquares(portal_squares);
            if (custom_points.empty())
                slam_system_->ClearViewerCustomPoints();
            else
                slam_system_->SetViewerCustomPoints(custom_points);
        }
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
            {"POST /api/v1/track", "Submit image for tracking; X-Run-Detector or JSON run_detector queues full-resolution custom detection"},
            {"GET /api/v1/status", "Get server status"},
            {"GET /api/v1/trajectory", "Trajectory (dense default; ?all_maps=1 every atlas map; ?dense=0 keyframes; ?aligned=0 raw SLAM)"},
            {"GET /api/v1/health", "Health check"}
        };
        response["portal_alignment_enabled"] = !portals_.empty();
        
        res.set_content(response.dump(), "application/json");
    }
    
    shared_ptr<ImageRequest> parseImageRequest(const httplib::Request& req,
                                               string* error_message = nullptr) {
        auto image_req = make_shared<ImageRequest>();
        string decode_error;
        
        // Generate frame ID and timestamp
        image_req->frame_id = total_requests_.load() + 1;
        image_req->timestamp = chrono::duration<double>(
            chrono::high_resolution_clock::now().time_since_epoch()).count();
        
        // Get client ID from headers or generate one
        auto client_header = req.get_header_value("X-Client-ID");
        image_req->client_id = client_header.empty() ? "unknown" : client_header;
        image_req->force_custom_detection = hasRunDetectorHeader(req);
        image_req->content_type = req.get_header_value("Content-Type");
        
        // Parse image from request body
        if (req.get_header_value("Content-Type").find("application/json") != string::npos) {
            // JSON request with base64 encoded image
            try {
                json request_json = json::parse(req.body);
                
                if (request_json.contains("timestamp")) {
                    image_req->timestamp = request_json["timestamp"];
                    image_req->has_capture_timestamp = true;
                }
                
                if (request_json.contains("frame_id")) {
                    image_req->frame_id = request_json["frame_id"];
                }
                
                if (request_json.contains("intrinsics")) {
                    image_req->intrinsics = request_json["intrinsics"].get<vector<float>>();
                }

                if (request_json.contains("run_detector") || request_json.contains("runDetector")) {
                    image_req->force_custom_detection = true;
                }
                
                if (request_json.contains("image_base64")) {
                    string base64_data = request_json["image_base64"];
                    vector<uchar> image_data = base64Decode(base64_data);
                    image_req->encoded_image_bytes = image_data.size();
                    const string image_content_type =
                        request_json.value("content_type", request_json.value("contentType", string()));
                    image_req->content_type = image_content_type;
                    image_req->image = decodeImageBytes(image_data, image_content_type, &decode_error);
                }
                
            } catch (const exception& e) {
                cout << "Error parsing JSON request: " << e.what() << endl;
                if (error_message)
                    *error_message = string("Error parsing JSON request: ") + e.what();
                return nullptr;
            }
        } else {
            // Direct image upload
            vector<uchar> image_data(req.body.begin(), req.body.end());
            image_req->encoded_image_bytes = image_data.size();
            image_req->image =
                decodeImageBytes(image_data, req.get_header_value("Content-Type"), &decode_error);

            const auto ts_hdr = req.get_header_value("X-Capture-Timestamp");
            if (!ts_hdr.empty()) {
                try {
                    image_req->timestamp = stod(ts_hdr);
                    image_req->has_capture_timestamp = true;
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
            const string message = decode_error.empty()
                ? "Failed to decode image from request"
                : "Failed to decode image from request: " + decode_error;
            cout << message << endl;
            if (error_message)
                *error_message = message;
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

    void processCustomDetections() {
        cout << "Started custom detection thread" << endl;

        while (true) {
            shared_ptr<CustomDetectionRequest> request;

            {
                unique_lock<mutex> lock(custom_detection_queue_mutex_);
                custom_detection_queue_cv_.wait(lock, [this] {
                    return !custom_detection_active_.load() || !custom_detection_queue_.empty();
                });

                if (custom_detection_queue_.empty()) {
                    if (!custom_detection_active_.load())
                        break;
                    continue;
                }

                request = custom_detection_queue_.front();
                custom_detection_queue_.pop_front();
            }

            if (!request)
                continue;

            try {
                const vector<CustomDetection> detections =
                    customDetect(request->image, request->intrinsics, request->frame_id);
                storeCustomDetectionObservations(
                    detections,
                    request->frame_id,
                    request->timestamp,
                    request->has_capture_timestamp,
                    request->intrinsics);
                if (!detections.empty())
                    refreshViewerCustomPoints();

                if (request->run_portal_detection && request->has_associated_slam_pose) {
                    PortalPnPResult pnp;
                    vector<PortalPnPResult> portal_detections;
                    if (solvePortalPnP(request->image, request->intrinsics, pnp, &portal_detections)) {
                        storePortalCornerObservations(
                            portal_detections,
                            request->frame_id,
                            request->timestamp,
                            request->intrinsics);
                        // Main track thread runs updatePortalAlignment with a fresh pose; skip
                        // here to avoid duplicate overlay ingests on stale associated_Tcw_slam.
                        refreshViewerPortalSquares();
                        refreshViewerCustomPoints();
                    }
                }

                custom_detection_processed_++;
            } catch (const exception& e) {
                cerr << "Custom detection error for frame " << request->frame_id
                     << ": " << e.what() << endl;
            }
        }

        cout << "Custom detection thread stopped" << endl;
    }
    
    void processImageRequest(shared_ptr<ImageRequest> request) {
        auto start_time = chrono::high_resolution_clock::now();

        vector<float> slam_intrinsics;
        const auto t_prepare0 = chrono::steady_clock::now();
        const cv::Mat slam_image = prepareSlamImage(request, slam_intrinsics);
        const double prepare_ms = chrono::duration<double, milli>(
            chrono::steady_clock::now() - t_prepare0).count();

        cout << "Frame " << request->frame_id << " image resolution: "
             << request->image.cols << " x " << request->image.rows;
        if (slam_image.data != request->image.data) {
            cout << " (SLAM input " << slam_image.cols << " x " << slam_image.rows << ")";
        }
        cout << endl;
        
        bool success = false;
        Sophus::SE3f pose;
        vector<ORB_SLAM3::MapPoint*> tracked_map_points;
        vector<cv::KeyPoint> tracked_keypoints;
        vector<float> portal_intrinsics;
        vector<cv::KeyPoint> prior_match_kpts;
        cv::Mat prior_match_desc;
        unsigned long current_map_id = 0;
        ORB_SLAM3::FrameTimingMs vo_timing;
        double portal_ms = 0.0;
        double prior_ms = 0.0;
        try {
            {
                lock_guard<mutex> slam_lock(slam_mutex_);

                pose = slam_system_->TrackMonocular(
                    slam_image,
                    request->timestamp,
                    vector<ORB_SLAM3::IMU::Point>(),
                    "",
                    slam_intrinsics
                );
                vo_timing = slam_system_->GetLastFrameTiming();

                success = !pose.matrix().isZero();
                if (success) {
                    successful_tracks_++;
                    current_map_id = slam_system_->GetCurrentMapId();
                    syncActiveMapAlignmentState(current_map_id);
                    const overlay::RefineResult live_refine = overlay_graph_.recordLivePose(
                        current_map_id,
                        request->frame_id,
                        request->timestamp,
                        pose.inverse(),
                        false);
                    if (live_refine.published) {
                        applyOverlayRefineToAlignment(current_map_id, live_refine);
                        cout << "[overlay] live_edge " << live_refine.status
                             << " map=" << current_map_id
                             << " scale=" << live_refine.pin.scale
                             << " scale_update=" << live_refine.scale_update_status
                             << " corner_factors=" << live_refine.pin.portal_corner_factor_count
                             << " live_edges=" << live_refine.pin.live_relative_edge_count
                             << " corner_rmse_px=" << live_refine.pin.last_corner_rmse_px
                             << " live_edge_rmse_m=" << live_refine.pin.last_live_edge_rmse_m
                             << " opt_passes=" << live_refine.pin.optimization_passes
                             << endl;
                    }
                    updateLatestSlamFrame(request->frame_id, request->timestamp, pose);
                    if (shouldRunPortalDetection(request->frame_id)) {
                        tracked_map_points = slam_system_->GetTrackedMapPoints();
                        tracked_keypoints = slam_system_->GetTrackedKeyPointsUn();
                        portal_intrinsics = slam_intrinsics;
                    }
                    if (prior_pack_.loaded && shouldRunPriorMatch(request->frame_id)) {
                        prior_match_kpts = slam_system_->GetTrackedKeyPointsUn();
                        prior_match_desc = slam_system_->GetCurrentFrameDescriptors();
                    }
                } else {
                    failed_tracks_++;
                    const unsigned long lost_map_id = slam_system_->GetCurrentMapId();
                    clearDomainAlignmentForMap(lost_map_id);
                    cout << "[overlay] tracking lost — cleared domain alignment for map "
                         << lost_map_id << endl;
                }
            }

            if (success && shouldRunPortalDetection(request->frame_id)) {
                const auto t_portal0 = chrono::steady_clock::now();
                const vector<float> portal_detect_intrinsics =
                    fullResolutionIntrinsicsForRequest(request);
                const cv::Mat& portal_detect_image = request->image;
                if (portal_detect_intrinsics.size() < 4 || portal_detect_image.empty()) {
                    cerr << "[portal] skipping QR PnP for frame " << request->frame_id
                         << ": no full-resolution image/intrinsics available" << endl;
                } else {
                    PortalPnPResult pnp;
                    vector<PortalPnPResult> portal_detections;
                    if (solvePortalPnP(
                            portal_detect_image, portal_detect_intrinsics, pnp, &portal_detections)) {
                        storePortalCornerObservations(
                            portal_detections,
                            request->frame_id,
                            request->timestamp,
                            portal_detect_intrinsics);

                        PortalPnPResult pnp_for_scale = pnp;
                        vector<float> scale_intrinsics = portal_detect_intrinsics;
                        const cv::Size detect_size(
                            portal_detect_image.cols, portal_detect_image.rows);
                        const cv::Size slam_size(slam_image.cols, slam_image.rows);
                        if (isValidImageSize(slam_size) && isValidImageSize(detect_size) &&
                            !sameImageSize(detect_size, slam_size) &&
                            portal_intrinsics.size() >= 4) {
                            const float sx = static_cast<float>(slam_size.width) /
                                             static_cast<float>(detect_size.width);
                            const float sy = static_cast<float>(slam_size.height) /
                                             static_cast<float>(detect_size.height);
                            for (auto& pt : pnp_for_scale.image_points) {
                                pt.x *= sx;
                                pt.y *= sy;
                            }
                            scale_intrinsics = portal_intrinsics;
                        }

                        const LocalScaleEstimate local_scale = estimateLocalScaleFromPortalRegion(
                            pnp_for_scale,
                            pose,
                            tracked_map_points,
                            tracked_keypoints,
                            scale_intrinsics);
                        updatePortalAlignment(
                            pnp,
                            pose,
                            local_scale,
                            request->frame_id,
                            request->timestamp,
                            current_map_id,
                            portal_detect_intrinsics);
                        refreshViewerPortalSquares();
                        refreshViewerCustomPoints();
                    }
                }
                portal_ms = chrono::duration<double, milli>(
                    chrono::steady_clock::now() - t_portal0).count();
            }

            if (success && !prior_match_desc.empty()) {
                const auto t_prior0 = chrono::steady_clock::now();
                vector<float> feature_intrinsics = slam_intrinsics;
                if (isValidImageSize(slam_input_size_) && isValidImageSize(feature_image_size_) &&
                    !sameImageSize(slam_input_size_, feature_image_size_)) {
                    feature_intrinsics =
                        scaleIntrinsicsToSize(slam_intrinsics, slam_input_size_, feature_image_size_);
                }
                tryPriorSoftEdgesAfterTrack(
                    current_map_id,
                    request->frame_id,
                    request->timestamp,
                    pose,
                    prior_match_kpts,
                    prior_match_desc,
                    feature_intrinsics,
                    feature_image_size_.width,
                    feature_image_size_.height);
                prior_ms = chrono::duration<double, milli>(
                    chrono::steady_clock::now() - t_prior0).count();
            }

        } catch (const exception& e) {
            cerr << "SLAM processing error: " << e.what() << endl;
            failed_tracks_++;
        }
        
        auto end_time = chrono::high_resolution_clock::now();
        const double processing_time =
            chrono::duration<double>(end_time - start_time).count();
        const double total_ms = processing_time * 1000.0;

        stage_timing_.record(
            request->frame_id, prepare_ms, vo_timing, portal_ms, prior_ms, total_ms);
        if (total_ms > 100.0) {
            cerr << fixed << setprecision(1)
                 << "[timing] spike frame " << request->frame_id
                 << " total=" << total_ms
                 << "ms prepare=" << prepare_ms
                 << " extract=" << vo_timing.extract_ms
                 << " pose=" << vo_timing.pose_pred_ms
                 << " mm=" << vo_timing.pose_mm_ms
                 << " mm_match=" << vo_timing.pose_mm_match_ms
                 << " mm_opt=" << vo_timing.pose_mm_opt_ms
                 << " lg_prep=" << vo_timing.lg_prep_ms
                 << " lg_fwd=" << vo_timing.lg_fwd_ms
                 << " lg_drain=" << vo_timing.lg_drain_ms
                 << " lg_post=" << vo_timing.lg_post_ms
                 << " lg_calls=" << vo_timing.lg_calls
                 << " lg_kpts=" << vo_timing.lg_kpts0 << "/" << vo_timing.lg_kpts1
                 << " refkf=" << vo_timing.pose_refkf_ms
                 << " reloc=" << vo_timing.pose_reloc_ms
                 << " local_map=" << vo_timing.local_map_ms
                 << " new_kf=" << vo_timing.new_kf_ms
                 << " portal=" << portal_ms
                 << " prior=" << prior_ms
                 << " local_pts=" << vo_timing.local_map_points
                 << " inliers=" << vo_timing.matches_inliers
                 << endl;
        }
        
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
    cout << "  --max-queue <number>   Maximum queue size (default: 100)" << endl;
    cout << "  --slam-interval-nth <N> Run SLAM on one out of every N accepted frames (default: 1)" << endl;
    cout << "  --custom-detect-every <N> Run custom detection every N accepted frames (default: disabled)" << endl;
    cout << "  --min-buffer <number>  Min frames in queue before tracking; oldest timestamp first (default: 3)" << endl;
    cout << "  --portal-manifest <path>  refined_manifest.json with portals[].pose and physicalSize (overrides pack portals)" << endl;
    cout << "  --prior-pack <path>       prior pack root; pack portals[] enable alignment when manifest unset" << endl;
    cout << "  --prior-match-every <N>   Run prior soft-edge match every N frames (default: 5)" << endl;
    cout << "  --portal-detect-every <N> Run portal QR detection every N accepted frames (default: 1)" << endl;
    cout << "  --portal-max-reproj-px <N> Reject portal PnP above this mean reprojection error (default: 8)" << endl;
    cout << "  --occlusion-mesh <path> OBJ mesh in domain coordinates for barcode ray placement and viewer rendering" << endl;
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
        } else if (arg == "--custom-detect-every" && i + 1 < argc) {
            config.custom_detect_every = static_cast<size_t>(stoul(argv[++i]));
        } else if (arg == "--min-buffer" && i + 1 < argc) {
            config.min_buffer_frames = static_cast<size_t>(stoul(argv[++i]));
        } else if (arg == "--portal-manifest" && i + 1 < argc) {
            config.portal_manifest_path = argv[++i];
        } else if (arg == "--prior-pack" && i + 1 < argc) {
            config.prior_pack_path = argv[++i];
        } else if (arg == "--prior-match-every" && i + 1 < argc) {
            config.prior_match_every_nth = static_cast<size_t>(stoul(argv[++i]));
            if (config.prior_match_every_nth < 1)
                config.prior_match_every_nth = 1;
        } else if (arg == "--portal-detect-every" && i + 1 < argc) {
            config.portal_detect_every = static_cast<size_t>(stoul(argv[++i]));
            if (config.portal_detect_every < 1)
                config.portal_detect_every = 1;
        } else if (arg == "--portal-max-reproj-px" && i + 1 < argc) {
            config.portal_max_reproj_px = stof(argv[++i]);
        } else if (arg == "--occlusion-mesh" && i + 1 < argc) {
            config.occlusion_mesh_path = argv[++i];
        } else if (arg == "--viewer") {
            config.use_viewer = true;
        } else {
            cerr << "Unknown argument: " << arg << endl;
            printUsage();
            return 1;
        }
    }

    if (config.prior_pack_path.empty())
        config.prior_pack_path = loadPriorPackPathFromSettings(config.settings_path);
    
    if (!config.occlusion_mesh_path.empty()) {
        string error;
        if (!loadObjMesh(config.occlusion_mesh_path, config.occlusion_mesh, error)) {
            cerr << "Failed to load --occlusion-mesh: " << error << endl;
            return 1;
        }
        if (!config.occlusion_mesh.hasTriangles()) {
            cerr << "Warning: --occlusion-mesh loaded but no OBJ face triangles were parsed: "
                 << config.occlusion_mesh_path << endl;
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