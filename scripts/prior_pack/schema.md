# Prior pack on-disk schema (v1–v2)

Offline prior packs supply everything needed for covisible prior matching without
recomputing COLMAP tracks at load time. Produced by `scripts/convert_to_prior_pack.py`.

## Layout

```
<prior_pack_root>/
  manifest.json          # version, coordinate frame, file index, counts
  images.json            # image id, path, intrinsics, pose
  portals.json           # portal short ids, domain poses, physical size
  covisibility.json      # image–image adjacency edges
  tracks.json            # 3D points + observations (image id, keypoint index)
  features/              # per-image ALIKED keypoints + descriptors
    0.npz                # or 0.json for tiny fixtures
  images/                # optional embedded image blobs (paths in images.json)
    000.png
```

## manifest.json

| Field | Type | Required | Notes |
|-------|------|----------|-------|
| `format_version` | int | yes | `1` (legacy) or `2` (optional per-track `rgb`) |
| `coordinate_frame` | object | yes | `name`, `notes` (human-readable frame description) |
| `descriptor` | object | yes | `type` (`aliked`), `model` (e.g. `aliked-n16`), `dim` (128) |
| `files` | object | yes | Relative paths: `images`, `portals`, `covisibility`, `tracks`, `features_dir` |
| `counts` | object | yes | `images`, `tracks`, `portals` tallies |

## images.json

Array wrapper `images[]` with entries:

| Field | Type | Required |
|-------|------|----------|
| `id` | int | yes, unique, contiguous from 0 |
| `path` | string | yes, relative to pack root |
| `intrinsics` | object | yes, `fx`, `fy`, `cx`, `cy` |
| `pose` | object | yes, `position` {x,y,z}, `rotation` {w,x,y,z} quaternion (camera-to-world in domain frame) |

## portals.json

Same shape as `slam_server` portal manifest: `portals[]` with `shortId`, `physicalSize`, `pose`.

## covisibility.json

`edges[]`: `{ "i": int, "j": int, "shared_tracks": int }` with `i < j`.

## tracks.json

`tracks[]`: `{ "id": int, "position": {x,y,z}, "observations": [{ "image_id", "keypoint_index" }], "rgb": [r,g,b] }`.

`rgb` is optional (v2 packs; may be absent on individual tracks). When present, each channel is uint8 in 0–255 (COLMAP point color). v1 packs omit `rgb` entirely; loaders treat missing color as unset (fallback applied at display time).

## features/{id}.npz

NumPy archive (preferred for real packs):

| Array | Shape | Dtype |
|-------|-------|-------|
| `keypoints` | N × 2 | float32, pixel (x, y) |
| `descriptors` | N × D | float32, D = manifest.descriptor.dim |
| `scores` | N | float32, optional |

Tiny fixtures may use `features/{id}.json` instead: `{ "keypoints": [[x,y],...], "descriptors": [[...],...], "scores": [...] }`.

## Reconstruction input layout (converter `--input`)

```
<recon_root>/
  images.json            # same schema as pack images.json
  portals.json           # optional; copied/embedded into pack
  covisibility.json      # same as pack
  tracks.json            # same as pack
  features/              # npz or json per image id
  images/                # image files referenced by images.json paths
```
