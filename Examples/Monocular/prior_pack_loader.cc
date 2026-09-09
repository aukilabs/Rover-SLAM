#include "prior_pack_loader.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_set>

#include "cnpy.h"

#ifdef _WIN32
#include <direct.h>
#include <sys/stat.h>
#else
#include <sys/stat.h>
#endif

namespace prior_pack {

namespace {

constexpr int kMinFormatVersion = 1;
constexpr int kMaxFormatVersion = 2;

bool path_is_separator(char c) { return c == '/' || c == '\\'; }

bool file_exists(const std::string& path) {
    struct stat st {};
    return stat(path.c_str(), &st) == 0 && (st.st_mode & S_IFREG);
}

bool directory_exists(const std::string& path) {
    struct stat st {};
    return stat(path.c_str(), &st) == 0 && (st.st_mode & S_IFDIR);
}

std::string join_path(const std::string& dir, const std::string& leaf) {
    if (dir.empty())
        return leaf;
    if (path_is_separator(dir.back()))
        return dir + leaf;
    return dir + "/" + leaf;
}

json load_json_file(const std::string& path) {
    std::ifstream in(path);
    if (!in.good())
        throw PriorPackError("cannot open " + path);
    json data;
    try {
        in >> data;
    } catch (const std::exception& e) {
        throw PriorPackError(std::string("invalid JSON in ") + path + ": " + e.what());
    }
    return data;
}

void require_keys(const json& obj, const std::vector<const char*>& keys, const char* label) {
    for (const char* key : keys) {
        if (!obj.contains(key))
            throw PriorPackError(std::string(label) + " missing required field: " + key);
    }
}

Vec3 parse_vec3(const json& v, const char* label) {
    if (!v.is_object())
        throw PriorPackError(std::string(label) + " must be an object");
    require_keys(v, {"x", "y", "z"}, label);
    Vec3 out;
    out.x = v["x"].get<float>();
    out.y = v["y"].get<float>();
    out.z = v["z"].get<float>();
    return out;
}

Quat parse_quat(const json& v, const char* label) {
    if (!v.is_object())
        throw PriorPackError(std::string(label) + " must be an object");
    require_keys(v, {"w", "x", "y", "z"}, label);
    Quat out;
    out.w = v["w"].get<float>();
    out.x = v["x"].get<float>();
    out.y = v["y"].get<float>();
    out.z = v["z"].get<float>();
    return out;
}

bool parse_optional_track_rgb(const json& track, const char* label, TrackRecord& rec) {
    if (!track.contains("rgb"))
        return false;
    const json& rgb = track["rgb"];
    if (!rgb.is_array() || rgb.size() != 3)
        throw PriorPackError(std::string(label) + ".rgb must be [r, g, b]");
    for (size_t i = 0; i < 3; ++i) {
        if (!rgb[i].is_number_integer())
            throw PriorPackError(std::string(label) + ".rgb[" + std::to_string(i) + "] must be int");
        const int channel = rgb[i].get<int>();
        if (channel < 0 || channel > 255)
            throw PriorPackError(
                std::string(label) + ".rgb[" + std::to_string(i) + "] must be in 0-255");
        rec.rgb[i] = static_cast<uint8_t>(channel);
    }
    rec.has_rgb = true;
    return true;
}

Pose parse_pose(const json& v, const char* label) {
    if (!v.is_object())
        throw PriorPackError(std::string(label) + " must be an object");
    require_keys(v, {"position", "rotation"}, label);
    Pose out;
    out.position = parse_vec3(v["position"], (std::string(label) + ".position").c_str());
    out.rotation = parse_quat(v["rotation"], (std::string(label) + ".rotation").c_str());
    return out;
}

Intrinsics parse_intrinsics(const json& v, const char* label) {
    if (!v.is_object())
        throw PriorPackError(std::string(label) + " must be an object");
    require_keys(v, {"fx", "fy", "cx", "cy"}, label);
    Intrinsics out;
    out.fx = v["fx"].get<float>();
    out.fy = v["fy"].get<float>();
    out.cx = v["cx"].get<float>();
    out.cy = v["cy"].get<float>();
    return out;
}

ImageFeatures load_features_json(const std::string& path, int desc_dim) {
    const json feat = load_json_file(path);
    if (!feat.contains("keypoints") || !feat.contains("descriptors"))
        throw PriorPackError("features file missing keypoints or descriptors: " + path);

    const json& kpts = feat["keypoints"];
    const json& descs = feat["descriptors"];
    if (!kpts.is_array() || !descs.is_array())
        throw PriorPackError("features keypoints/descriptors must be arrays: " + path);
    if (kpts.size() != descs.size())
        throw PriorPackError("features keypoints/descriptors length mismatch: " + path);

    ImageFeatures out;
    for (size_t i = 0; i < kpts.size(); ++i) {
        const json& kp = kpts[i];
        if (!kp.is_array() || kp.size() < 2)
            throw PriorPackError("feature keypoint must be [x,y]: " + path);
        out.keypoints.emplace_back(kp[0].get<float>(), kp[1].get<float>());

        const json& d = descs[i];
        if (!d.is_array())
            throw PriorPackError("feature descriptor must be an array: " + path);
        if (static_cast<int>(d.size()) != desc_dim)
            throw PriorPackError("feature descriptor dim mismatch in " + path);
        std::vector<float> row;
        for (const auto& v : d)
            row.push_back(v.get<float>());
        out.descriptors.push_back(std::move(row));
    }

    if (feat.contains("scores") && feat["scores"].is_array()) {
        for (const auto& s : feat["scores"])
            out.scores.push_back(s.get<float>());
    }
    return out;
}

ImageFeatures load_features_npz(const std::string& path, int desc_dim) {
    cnpy::npz_t npz;
    try {
        npz = cnpy::npz_load(path);
    } catch (const std::exception& e) {
        throw PriorPackError(std::string("cannot load npz features: ") + path + ": " + e.what());
    }

    const auto it_k = npz.find("keypoints");
    const auto it_d = npz.find("descriptors");
    if (it_k == npz.end() || it_d == npz.end())
        throw PriorPackError("features npz missing keypoints or descriptors: " + path);

    const cnpy::NpyArray& karr = it_k->second;
    const cnpy::NpyArray& darr = it_d->second;
    if (karr.word_size != sizeof(float) || darr.word_size != sizeof(float))
        throw PriorPackError("features npz must use float32 arrays: " + path);
    if (karr.shape.size() != 2 || karr.shape[1] != 2)
        throw PriorPackError("features npz keypoints must be shape N×2: " + path);
    if (darr.shape.size() != 2 || static_cast<int>(darr.shape[1]) != desc_dim)
        throw PriorPackError("features npz descriptors must be shape N×" + std::to_string(desc_dim) +
                             ": " + path);
    if (karr.shape[0] != darr.shape[0])
        throw PriorPackError("features npz keypoints/descriptors row count mismatch: " + path);

    const size_t n = karr.shape[0];
    const float* kpts = karr.data<float>();
    const float* desc = darr.data<float>();

    ImageFeatures out;
    out.keypoints.reserve(n);
    out.descriptors.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        out.keypoints.emplace_back(kpts[i * 2], kpts[i * 2 + 1]);
        const float* row = desc + i * static_cast<size_t>(desc_dim);
        out.descriptors.emplace_back(row, row + desc_dim);
    }

    const auto it_s = npz.find("scores");
    if (it_s != npz.end()) {
        const cnpy::NpyArray& sarr = it_s->second;
        if (sarr.word_size == sizeof(float) && sarr.shape.size() == 1 &&
            sarr.shape[0] == n) {
            const float* scores = sarr.data<float>();
            out.scores.assign(scores, scores + n);
        }
    }
    return out;
}

ImageFeatures load_features_for_image(
    const std::string& features_dir, int image_id, int desc_dim) {
    const std::string json_path = join_path(features_dir, std::to_string(image_id) + ".json");
    if (file_exists(json_path))
        return load_features_json(json_path, desc_dim);

    const std::string npz_path = join_path(features_dir, std::to_string(image_id) + ".npz");
    if (file_exists(npz_path))
        return load_features_npz(npz_path, desc_dim);

    throw PriorPackError(
        "missing features for image id " + std::to_string(image_id) + ": expected " +
        join_path(features_dir, std::to_string(image_id) + ".json"));
}

} // namespace

size_t PriorPack::total_keypoints() const {
    size_t n = 0;
    for (const auto& img : images)
        n += img.features.keypoints.size();
    return n;
}

PriorPack load_prior_pack(const std::string& pack_root_path) {
    if (!directory_exists(pack_root_path))
        throw PriorPackError("pack root is not a directory: " + pack_root_path);

    const std::string manifest_path = join_path(pack_root_path, "manifest.json");
    if (!file_exists(manifest_path))
        throw PriorPackError("missing required file: manifest.json");

    const json manifest = load_json_file(manifest_path);
    if (!manifest.is_object())
        throw PriorPackError("manifest.json must be a JSON object");

    require_keys(
        manifest,
        {"format_version", "coordinate_frame", "descriptor", "files", "counts"},
        "manifest");

    const int version = manifest["format_version"].get<int>();
    if (version < kMinFormatVersion || version > kMaxFormatVersion)
        throw PriorPackError(
            "unsupported format_version " + std::to_string(version) + " (expected " +
            std::to_string(kMinFormatVersion) + "–" + std::to_string(kMaxFormatVersion) + ")");

    const json& coord = manifest["coordinate_frame"];
    if (!coord.is_object() || !coord.contains("name") || !coord.contains("notes"))
        throw PriorPackError("manifest.coordinate_frame requires name and notes");

    const json& descriptor = manifest["descriptor"];
    if (!descriptor.is_object() || !descriptor.contains("dim"))
        throw PriorPackError("manifest.descriptor missing dim");
    const int desc_dim = descriptor["dim"].get<int>();
    if (desc_dim <= 0)
        throw PriorPackError("manifest.descriptor.dim must be positive");

    const json& files = manifest["files"];
    if (!files.is_object())
        throw PriorPackError("manifest.files must be an object");
    require_keys(
        files,
        {"images", "portals", "covisibility", "tracks", "features_dir"},
        "manifest.files");

    const json& counts = manifest["counts"];
    if (!counts.is_object())
        throw PriorPackError("manifest.counts must be an object");
    require_keys(counts, {"images", "tracks", "portals"}, "manifest.counts");

    const std::string images_rel = files["images"].get<std::string>();
    const std::string portals_rel = files["portals"].get<std::string>();
    const std::string covis_rel = files["covisibility"].get<std::string>();
    const std::string tracks_rel = files["tracks"].get<std::string>();
    const std::string features_rel = files["features_dir"].get<std::string>();

    const std::string images_path = join_path(pack_root_path, images_rel);
    const std::string portals_path = join_path(pack_root_path, portals_rel);
    const std::string covis_path = join_path(pack_root_path, covis_rel);
    const std::string tracks_path = join_path(pack_root_path, tracks_rel);
    const std::string features_dir = join_path(pack_root_path, features_rel);

    for (const auto& label_path :
         {std::pair<const char*, std::string>{"images", images_path},
          {"portals", portals_path},
          {"covisibility", covis_path},
          {"tracks", tracks_path}}) {
        if (!file_exists(label_path.second))
            throw PriorPackError(
                std::string("missing required file: ") + label_path.first + " (" + label_path.second +
                ")");
    }
    if (!directory_exists(features_dir))
        throw PriorPackError("missing required directory: features_dir (" + features_dir + ")");

    PriorPack pack;
    pack.loaded = true;
    pack.pack_root = pack_root_path;
    pack.format_version = version;
    pack.coordinate_frame_name = coord["name"].get<std::string>();
    pack.coordinate_frame_notes = coord["notes"].get<std::string>();
    pack.descriptor_type = descriptor.value("type", std::string());
    pack.descriptor_model = descriptor.value("model", std::string());
    pack.descriptor_dim = desc_dim;

    const json images_doc = load_json_file(images_path);
    if (!images_doc.is_object() || !images_doc.contains("images"))
        throw PriorPackError("images.json must be an object with an images array");
    const json& images = images_doc["images"];
    if (!images.is_array())
        throw PriorPackError("images.json images must be an array");

  std::vector<int> image_ids;
    for (size_t idx = 0; idx < images.size(); ++idx) {
        const std::string label = "images[" + std::to_string(idx) + "]";
        const json& img = images[idx];
        if (!img.is_object())
            throw PriorPackError(label + " must be an object");
        require_keys(img, {"id", "path", "intrinsics", "pose"}, label.c_str());

        ImageRecord rec;
        rec.id = img["id"].get<int>();
        rec.path = img["path"].get<std::string>();
        rec.intrinsics = parse_intrinsics(img["intrinsics"], (label + ".intrinsics").c_str());
        rec.pose = parse_pose(img["pose"], (label + ".pose").c_str());

        if (rec.path.empty())
            throw PriorPackError(label + ".path must be a non-empty string");
        const std::string blob_path = join_path(pack_root_path, rec.path);
        if (!file_exists(blob_path))
            throw PriorPackError(
                label + " image blob not found: " + rec.path + " (resolved " + blob_path + ")");

        image_ids.push_back(rec.id);
        pack.images.push_back(std::move(rec));
    }

    for (size_t i = 0; i < image_ids.size(); ++i) {
        if (image_ids[i] != static_cast<int>(i))
            throw PriorPackError("image ids must be contiguous from 0");
    }

    for (auto& img : pack.images)
        img.features = load_features_for_image(features_dir, img.id, desc_dim);

    const json portals_doc = load_json_file(portals_path);
    if (!portals_doc.is_object() || !portals_doc.contains("portals"))
        throw PriorPackError("portals.json must be an object with portals array");
    const json& portals = portals_doc["portals"];
    if (!portals.is_array())
        throw PriorPackError("portals.json portals must be an array");

    for (size_t idx = 0; idx < portals.size(); ++idx) {
        const std::string label = "portals[" + std::to_string(idx) + "]";
        const json& portal = portals[idx];
        if (!portal.is_object())
            throw PriorPackError(label + " must be an object");
        require_keys(portal, {"shortId", "physicalSize", "pose"}, label.c_str());

        PortalRecord rec;
        rec.short_id = portal["shortId"].get<std::string>();
        rec.physical_size = portal["physicalSize"].get<float>();
        rec.pose = parse_pose(portal["pose"], (label + ".pose").c_str());
        if (rec.short_id.empty())
            throw PriorPackError(label + ".shortId must be a non-empty string");
        if (!std::isfinite(rec.physical_size) || rec.physical_size <= 0.f)
            throw PriorPackError(label + ".physicalSize must be positive");
        pack.portals.push_back(std::move(rec));
    }

    const std::unordered_set<int> image_id_set(image_ids.begin(), image_ids.end());

    const json covis_doc = load_json_file(covis_path);
    if (!covis_doc.is_object() || !covis_doc.contains("edges"))
        throw PriorPackError("covisibility.json must be an object with edges array");
    const json& edges = covis_doc["edges"];
    if (!edges.is_array())
        throw PriorPackError("covisibility.json edges must be an array");

    for (size_t idx = 0; idx < edges.size(); ++idx) {
        const std::string label = "covisibility.edges[" + std::to_string(idx) + "]";
        const json& edge = edges[idx];
        if (!edge.is_object())
            throw PriorPackError(label + " must be an object");
        require_keys(edge, {"i", "j", "shared_tracks"}, label.c_str());

        CovisEdge rec;
        rec.i = edge["i"].get<int>();
        rec.j = edge["j"].get<int>();
        rec.shared_tracks = edge["shared_tracks"].get<int>();
        if (rec.i >= rec.j)
            throw PriorPackError(label + ": require i < j");
        if (!image_id_set.count(rec.i) || !image_id_set.count(rec.j))
            throw PriorPackError(label + ": image id out of range");
        if (rec.shared_tracks < 0)
            throw PriorPackError(label + ".shared_tracks must be non-negative int");
        pack.covis_edges.push_back(rec);
    }

    const json tracks_doc = load_json_file(tracks_path);
    if (!tracks_doc.is_object() || !tracks_doc.contains("tracks"))
        throw PriorPackError("tracks.json must be an object with tracks array");
    const json& tracks = tracks_doc["tracks"];
    if (!tracks.is_array())
        throw PriorPackError("tracks.json tracks must be an array");

    std::vector<int> track_ids;
    for (size_t idx = 0; idx < tracks.size(); ++idx) {
        const std::string label = "tracks[" + std::to_string(idx) + "]";
        const json& track = tracks[idx];
        if (!track.is_object())
            throw PriorPackError(label + " must be an object");
        require_keys(track, {"id", "position", "observations"}, label.c_str());

        TrackRecord rec;
        rec.id = track["id"].get<int>();
        rec.position = parse_vec3(track["position"], (label + ".position").c_str());
        parse_optional_track_rgb(track, label.c_str(), rec);
        const json& obs = track["observations"];
        if (!obs.is_array() || obs.empty())
            throw PriorPackError(label + ".observations must be a non-empty array");

        for (size_t oi = 0; oi < obs.size(); ++oi) {
            const std::string ob_label = label + ".observations[" + std::to_string(oi) + "]";
            const json& ob = obs[oi];
            if (!ob.is_object())
                throw PriorPackError(ob_label + " must be an object");
            require_keys(ob, {"image_id", "keypoint_index"}, ob_label.c_str());

            TrackObservation to;
            to.image_id = ob["image_id"].get<int>();
            to.keypoint_index = ob["keypoint_index"].get<int>();
            if (!image_id_set.count(to.image_id))
                throw PriorPackError(ob_label + ".image_id invalid: " + std::to_string(to.image_id));
            if (to.keypoint_index < 0)
                throw PriorPackError(ob_label + ".keypoint_index must be non-negative int");

            const size_t n_kpts = pack.images[static_cast<size_t>(to.image_id)].features.keypoints.size();
            if (static_cast<size_t>(to.keypoint_index) >= n_kpts)
                throw PriorPackError(
                    ob_label + ".keypoint_index out of range for image " +
                    std::to_string(to.image_id));

            rec.observations.push_back(to);
        }

        track_ids.push_back(rec.id);
        pack.tracks.push_back(std::move(rec));
    }

    bool track_contiguous = true;
    for (size_t i = 0; i < track_ids.size(); ++i) {
        if (track_ids[i] != static_cast<int>(i))
            track_contiguous = false;
    }
    if (!track_contiguous)
        throw PriorPackError("track ids must be contiguous from 0");

    const int expected_images = counts["images"].get<int>();
    const int expected_tracks = counts["tracks"].get<int>();
    const int expected_portals = counts["portals"].get<int>();
    if (static_cast<int>(pack.images.size()) != expected_images)
        throw PriorPackError("manifest.counts.images does not match images.json length");
    if (static_cast<int>(pack.tracks.size()) != expected_tracks)
        throw PriorPackError("manifest.counts.tracks does not match tracks.json length");
    if (static_cast<int>(pack.portals.size()) != expected_portals)
        throw PriorPackError("manifest.counts.portals does not match portals.json length");

    return pack;
}

json prior_pack_status_json(const PriorPack& pack) {
    if (!pack.loaded)
        return json{{"loaded", false}};

    return json{
        {"loaded", true},
        {"path", pack.pack_root},
        {"format_version", pack.format_version},
        {"coordinate_frame",
         {{"name", pack.coordinate_frame_name}, {"notes", pack.coordinate_frame_notes}}},
        {"descriptor",
         {{"type", pack.descriptor_type},
          {"model", pack.descriptor_model},
          {"dim", pack.descriptor_dim}}},
        {"counts",
         {{"images", pack.image_count()},
          {"tracks", pack.track_count()},
          {"portals", pack.portal_count()},
          {"covis_edges", pack.covis_edge_count()},
          {"keypoints", pack.total_keypoints()}}}};
}

} // namespace prior_pack
