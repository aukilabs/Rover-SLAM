# Initiative: Hybrid prior map + pose graph

Status: draft  
Tier: feature  
Stage: 1-intake  
Skips: _(none)_  
Last updated: 2026-09-02

Living document — update in place when later work changes earlier conclusions.  
**§1 Intent (feature) and §4 System design stay distinct — never merge.**

---

## 1. Intent

- **Customer / internal need:** Navigate indoors against a reconstruction-server prior (COLMAP + ALIKED descriptors + placed portals) while still running live SLAM so unmapped or stale regions remain usable.
- **Framing:** Pure atlas/reloc is a poor fit for ALIKED v1 (BoW deferred) and for incomplete priors. Portals are globally unique hard anchors; prior views give soft geometric constraints; live SLAM covers novelty. Treat live odometry/map edges, prior match edges, and portal absolute factors jointly as a **pose-graph** problem (not prior-only localization).
- **Why now:** ALIKED+LightGlue live stack is accepted; next product step is domain-aligned navigation with recon outputs without waiting on EigenPlaces as primary reloc.
- **Success look like:** Offline converter turns recon output into a load-cheap prior pack; `slam_server` (or equivalent) runs live ALIKED SLAM, seeds/re-anchors via portals, adds verified prior constraints when covisible matches succeed, and publishes poses in the domain frame with graceful behavior when the prior is missing or stale. **No mesh occlusion required in v1.**

## 2. Alternatives

_(Fill in `/define-feature` / alternatives pass.)_

## 3. Priority

_(Fill after §1 lock.)_

## 4. System design

_(Distinct from Intent — fill in `/design-system`. Expected themes from intake: offline prior pack + converter; live SLAM always on; portal hard factors; prior Soft edges via covisible COLMAP images + LightGlue; g2o pose graph; no mesh v1; EigenPlaces deferred.)_

## 5. Verification design

_(Fill before slices.)_

## 6. Work breakdown (AI-oriented)

_(Fill after §1–§5.)_

## 7. Implementation notes

- **Approvals:** _(none yet)_

## 8. Verification record

_(empty)_

## Gate (stage 11)

- **Decision:** _(pending)_  
- **Notes:**
