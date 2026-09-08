# Initiative: domain-3d-viewer

Status: active  
Tier: feature  
Stage: 6-breakdown  
Skips: _(none)_  
Last updated: 2026-09-08 (§1–§6 locked)

Living document — update in place when later work changes earlier conclusions.  
**§1 Intent (feature) and §4 System design stay distinct — never merge.**

---

## 1. Intent

- **Customer / internal need:** Operators debugging live phone SLAM with `slam_server` already get domain trajectories on the iPhone app / API, but Pangolin `--viewer` stays in raw SLAM until pin and is awkward on remote machines. Need a browser 3D view of the **domain** map.
- **Framing:** Add a standalone HTML page (served by `slam_server` like `index.html`) that renders domain-frame trajectories, portals, and map point cloud in 3D — matching what the iPhone app already shows for trajectory.
- **Why now:** User confirmed API trajectory is correct in domain coords; Pangolin does not meet the “see domain map” need.
- **Success look like:** Open `http://<host>:<port>/viewer` (same host/port as `GET /api/v1/trajectory`; no second static server) while tracking; see domain trajectories (all published maps), portals, and a point cloud updating live; colored points when the server can supply colors (or a clear fallback colormap).
- **Locked:** 2026-09-08 (human confirmed §1; path `/viewer` on slam_server port).

## 2. Alternatives

- **Existing ways:** Pangolin `--viewer` (raw until pin); `index.html` top-down 2D plot; iPhone app 3D; poll API yourself.
- **Comparison:** iPhone app proves data path; need a PC/browser standalone without app. Pangolin insufficient for domain-first viewing.
- **Decision:** **Proceed** (feature tier).

## 3. Priority

- **Rank:** insert as focused viewer feature alongside slam_server demos.
- **Capacity:** small–medium; new static page + map-points (and likely portals catalog) HTTP; MapPoints have no RGB today.
- **Decision:** top-of-queue for this chat.

## 4. System design

- **Decision (architecture):** **Design change needed** — extend `slam_server` HTTP surface + add static viewer page + prior-pack RGB plumbing; no Atlas/Tracking rewrite.
- **Products / codebases:** Rover-SLAM (`slam_server`, `prior_pack_loader`, converter `scripts/convert_to_prior_pack.py` / recon export path); reconstruction-server only as **source of COLMAP RGB** (no recon-server code change if colors already in points3D / intermediate recon artifacts we can read).
- **Serving (locked 2026-09-08 — human):** Same `slam_server` HTTP port as `/api/v1/trajectory`. No second static server. Same-origin `fetch`.
- **Path (locked 2026-09-08 — human):** **`GET /viewer`** → HTML loaded at startup like `index.html` (disk: `Examples/Monocular/viewer.html`).
- **Client (locked 2026-09-08 — human):** **Three.js via CDN**; poll ~1–2 Hz (match `index.html` trajectory poll order of magnitude). OrbitControls; domain Z-up or match trajectory convention already used by iPhone (verify against published `position` frame).
- **Reuse:** `GET /api/v1/trajectory` for camera path(s) — domain when pinned; publish policy / `?all_maps=1` as today.
- **Point cloud source (locked):** **Both** — prior tracks (backdrop) + live Atlas map points (when pin exists for that map; else omit or show in slam frame only if clearly labeled — **default: domain-only for live points**, hide until pinned).
- **Point colors (locked):** Prior RGB from pack; live height colormap cyan→teal (top→bottom). Emit live `rgb` from server after colormap so client stays dumb, **or** client maps height — prefer **server-side colormap** for one wire format.
- **Prior RGB plumbing (locked in-scope):** Extend recon→prior-pack track records with optional `rgb: [r,g,b]` (0–255); bump pack `format_version` if schema is versioned; loader keeps backward compat (missing → fallback gray/height). Rebuild or patch holdout pack used for demos.
- **HTTP APIs (proposed — confirm):**
  - `GET /api/v1/map_points?source=live|prior|both` (default `both`): JSON `{ coordinate_frame, points: [{x,y,z,r,g,b, source}], count, … }` with optional `max_points` downsample (uniform stride) for large priors (~50k+).
  - `GET /api/v1/portals`: catalog `short_id`, domain pose, `physical_size_m` from loaded manifest/pack (draw squares even before detection).
- **Constraints:** No mesh; no Pangolin dependency for this page; don’t block track thread — snapshot under existing locks; cap payload size.
- **Risks:** CDN requires network on viewer machine; large prior dumps need downsample; color missing on old packs until rebuild.
- **Decision:** proceed with above unless human revises proposed APIs / domain-only live rule.
- **Locked:** 2026-09-08 (human confirmed §4 as drafted).

## 5. Verification design

- **Depth (locked 2026-09-08 — human):** **Lean + holdout replay** + **agentic browser visual check** during streaming replay.
- **End-to-end acceptance:**
  1. `GET /viewer` returns HTML (`200`, `text/html`) on the same port as `/api/v1/trajectory`.
  2. Prior pack with plumbed `rgb`: fixture or rebuilt pack; loader exposes colors; `GET /api/v1/map_points?source=prior` returns points with `r,g,b` (or documented fallback when absent).
  3. `GET /api/v1/portals` returns catalog entries (`short_id`, pose, size) when pack/manifest loaded.
  4. After pin during holdout/streaming replay: `GET /api/v1/map_points?source=live` (or `both`) returns domain-frame live points with cyan→teal height colors; before pin, live points omitted (domain-only rule).
  5. Trajectory still correct via existing `/api/v1/trajectory` (no regression in frame/policy fields used by viewer).
  6. **Holdout / streaming replay:** run a short timestamped holdout (or existing replay harness) against `slam_server` with prior pack; confirm seed/pin then map_points populate.
  7. **Agentic browser:** open `http://<host>:<port>/viewer` while replay streams; confirm page loads, orbit/view shows prior cloud (colored), portals, trajectory, and live points after pin — “looks OK” human/agent visual pass (screenshot or noted observations). Not a pixel-diff CI gate.
- **Agent checks:** curl/jq JSON shape + counts; build `slam_server`; optional small Python assert on pack `rgb` presence after convert.
- **Out of scope for v1 verify:** CDN offline; Pangolin parity; dense mesh; CI headless WebGL screenshot automation; full 580-frame metric ATE (viewer is visual/debug, not pose accuracy gate).
- **Decision:** lean API + short replay + browser visual during stream.
- **Locked:** 2026-09-08 (human confirmed §5).

## 6. Work breakdown (AI-oriented)

- **Shape (proposed — confirm):** three implement slices, then verify against §5 (incl. agentic browser during replay).
- **S1 — Prior RGB plumbing**  
  - Extend recon→prior-pack track records with optional `rgb: [r,g,b]`; loader `TrackRecord` + backward compat; update fixture generator; small assert/test that convert preserves color when source has it.  
  - **Depends:** none. **Human check:** no (agent can verify fixture).
- **S2 — HTTP map_points + portals**  
  - `GET /api/v1/map_points` (`source`, `max_points`, domain-only live, server cyan→teal colormap); `GET /api/v1/portals`; wire into `slam_server` routing + status/root listing; docs snippet in `slam_server_api.md`.  
  - **Depends:** S1 for prior colors on wire (can stub gray until S1 merges). **Human check:** no.
- **S3 — `/viewer` page**  
  - `Examples/Monocular/viewer.html` (Three.js CDN): prior+live clouds, portals, trajectory poll; load-at-startup + `GET /viewer` like `index.html`; README one-liner.  
  - **Depends:** S2. **Human check:** yes — visual “looks OK” with §5 browser pass.
- **Stop / escalate:** if RGB only lives in recon-server COLMAP and cannot be read from existing recon export without recon-server changes → bounce §4 (may need recon export patch or dual-read points3D). If live point dump blocks track thread → redesign snapshot.
- **Verify (post S3, not a code slice):** build; curl APIs; short holdout/stream replay; **agentic browser** on `/viewer` during stream (§5 #6–#7).
- **Locked:** 2026-09-08 (human confirmed §6 as drafted: S1→S2→S3).

## 7. Implementation notes

## 8. Verification record

## Gate (stage 11)

- **Decision:**  
- **Notes:**
