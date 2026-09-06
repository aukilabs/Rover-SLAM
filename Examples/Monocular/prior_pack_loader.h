/**
 * Prior pack v1 loader for slam_server (hybrid prior-map overlay).
 * On-disk layout: scripts/prior_pack/schema.md
 */

#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include "../../include/third_party/nlohmann/json.hpp"

namespace prior_pack {

using json = nlohmann::json;

struct Vec3 {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
};

struct Quat {
    float w = 1.f;
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
};

struct Intrinsics {
    float fx = 0.f;
    float fy = 0.f;
    float cx = 0.f;
    float cy = 0.f;
};

struct Pose {
    Vec3 position;
    Quat rotation;
};

struct ImageFeatures {
    std::vector<std::pair<float, float>> keypoints;
    std::vector<std::vector<float>> descriptors;
    std::vector<float> scores;
};

struct ImageRecord {
    int id = 0;
    std::string path;
    Intrinsics intrinsics;
    Pose pose;
    ImageFeatures features;
};

struct PortalRecord {
    std::string short_id;
    float physical_size = 0.f;
    Pose pose;
};

struct CovisEdge {
    int i = 0;
    int j = 0;
    int shared_tracks = 0;
};

struct TrackObservation {
    int image_id = 0;
    int keypoint_index = 0;
};

struct TrackRecord {
    int id = 0;
    Vec3 position;
    std::vector<TrackObservation> observations;
};

struct PriorPack {
    bool loaded = false;
    std::string pack_root;
    int format_version = 0;
    std::string coordinate_frame_name;
    std::string coordinate_frame_notes;
    std::string descriptor_type;
    std::string descriptor_model;
    int descriptor_dim = 0;
    std::vector<ImageRecord> images;
    std::vector<PortalRecord> portals;
    std::vector<CovisEdge> covis_edges;
    std::vector<TrackRecord> tracks;

    size_t image_count() const { return images.size(); }
    size_t track_count() const { return tracks.size(); }
    size_t portal_count() const { return portals.size(); }
    size_t covis_edge_count() const { return covis_edges.size(); }
    size_t total_keypoints() const;
};

class PriorPackError : public std::runtime_error {
public:
    explicit PriorPackError(const std::string& message) : std::runtime_error(message) {}
};

/** Load and validate a prior pack directory. Throws PriorPackError on failure. */
PriorPack load_prior_pack(const std::string& pack_root_path);

/** Status fragment for GET /api/v1/status prior_pack field. */
json prior_pack_status_json(const PriorPack& pack);

} // namespace prior_pack
