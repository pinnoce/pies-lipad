---
name: feedback-reviews
description: How the user conducts code reviews and what they expect from each round
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 755c71f3-6f41-4146-863e-325b7b333604
---

User regularly asks "Review all files please" / "Review again please" as a distinct workflow. Each round is expected to find something new — don't recycle the same findings.

**Why:** Multiple review rounds have each caught real bugs (wrong formula, unused variable, stale comments, off-by-one array sizes). The user has high quality standards and wants fresh eyes each pass.

**How to apply:** On each review request, re-read every file from scratch. Prioritise logic correctness over style. Fix bugs immediately rather than listing them — the user expects changes, not just a report.

## Bugs caught per review round (record to calibrate thoroughness)

- **Round 1:** `sim_mission.py` used old velocity formula that diverged with calibrated-negative KP_X; `HFOV_DEG` declared but unused (hardcoded `784.7` in pixel model).
- **Round 2:** (same session) `sim_mission.py` — confirmed fix; no new bugs after formula correction.
- **Round 3:** `test_statustext.py:75` — hardcoded IP instead of computed `{ip}`; `sim_calibrate.py:183` — misleading "All orientations converge" footer contradicted table.
- **Round 4:** `autonomous_mission.py:431` — `_statustext()` padded to 127 chars but `LogMessage.text` is `uint8[128]`; would raise `ValueError` at runtime on first STATUSTEXT call.
- **Round 5:** `visual_centering.py:251` — CENTERING log missing `throttle_duration_sec=0.5`; fires at 20 Hz during centering (very spammy). `autonomous_mission.py` equivalent correctly throttled. Also: `sim_calibrate.py:45` comment referenced `_MIN_DISP_PX` but actual constant name is `_MIN_PROJ_PX`.
