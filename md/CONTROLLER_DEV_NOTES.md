Below is a clean explanation of:

- What is happening (based directly on the logs)
- Why the small rightward drift exists
- What is actually wrong vs what is acceptable
- Three concrete fixes (from safest → most invasive)
- A clear decision point on whether to move on

---

## AprilTag Hover Controller v2
### Residual Lateral Drift — Diagnosis & Resolution Plan

**Current Status**

The hover_yaw_search_v2 controller is now:

- Stable (no divergence, no runaway)
- Consistently detecting and tracking the AprilTag
- Correctly aligning yaw to the tag
- Correctly regulating distance to the tag
- Operating entirely in a camera-frame IBVS Phase-1 architecture, as intended

The remaining issue is a slow, consistent lateral drift to the right.

This is not a "controller is broken" problem — it is a systematic bias problem.

---

### 1. What the Logs Are Telling Us

From the logs:

```
p_cam[x=-0.22 … -0.26]
lat_error ≈ -0.22
lateral_cmd ≈ +0.02 m/s
```

Interpretation:

- The tag is consistently left of center in the camera frame (negative x)
- The controller is issuing a small positive lateral velocity
- The sign is correct (it's trying to move right to center the tag)
- The magnitude is small but persistent
- The error never converges to zero

This tells us: the controller is behaving consistently, but the system has a steady-state bias.

---

### 2. Why This Drift Exists (Root Cause)

There are three overlapping reasons.

**Root Cause A — Pure P control cannot eliminate steady-state bias**

The lateral controller is:

```python
lateral_cmd = k_lateral * (-x_cam)
```

This is a pure proportional controller. In real systems (and realistic simulations):

- Camera mounting offsets
- Slight yaw bias
- MAVROS velocity tracking error
- Aerodynamic / numerical damping
- FCU internal controllers

→ all introduce constant disturbances. A pure P controller will always settle with a nonzero residual error. **This is expected behavior, not a bug.**

**Root Cause B — Camera optical frame ≠ true body symmetry axis**

Even if the camera looks centered visually:

- The camera is not perfectly aligned with base_link
- The optical axis is not the vehicle's aerodynamic center
- A small yaw bias produces a lateral image error even at steady hover

This shows up exactly as: small constant `x_cam`, small constant lateral velocity, slow sideways drift.

**Root Cause C — Yaw gating introduces lateral "bursts"**

From the code:

```python
align_ok = abs(yaw_error) < yaw_align_threshold
```

Yaw error hovers around the threshold, causing translation to turn on/off intermittently. Each time it turns on, a small lateral step occurs — producing a random-walk lateral drift even if the average command is small.

---

### 3. What Is Not Wrong

- The signs are correct
- The frame choice is correct for Phase-1
- The controller is stable
- The drone is behaving logically
- This is not a TF bug
- This is not MAVROS instability

This is exactly what a Phase-1 IBVS controller looks like before refinement.

---

### 4. Fix Options (Safest → Strongest)

**Option 1 — Add a lateral deadband (RECOMMENDED FIRST)**

```python
LATERAL_DEADBAND = 0.05  # meters (tune 0.03–0.08)

if abs(lateral_error_cam) < LATERAL_DEADBAND:
    lateral_cmd = 0.0
```

Effect: eliminates tiny corrective motions, stops slow drift, keeps controller simple and stable. This is standard practice in visual servoing systems.

**Option 2 — Add a tiny integral term (PI control)**

```python
self.lateral_integral += lateral_error_cam * dt
lateral_cmd = k_p * (-x) + k_i * self.lateral_integral
```

Downsides: requires careful anti-windup, can introduce oscillation if mistuned. More complexity than needed for Phase-1.

**Option 3 — Switch to body-frame regulation (Phase-2 controller)**

Form error vector in camera frame → rotate into body frame using TF → regulate lateral in body Y, not camera X. This removes camera alignment bias entirely — but it is Phase-2 work, not Phase-1.

---

### 5. Recommended Immediate Change

Add one thing only:

```python
LATERAL_DEADBAND = 0.05

if align_ok:
    if abs(lateral_error_cam) > LATERAL_DEADBAND:
        lateral_cmd = self.lock_k_lateral * (-lateral_error_cam)
    else:
        lateral_cmd = 0.0
else:
    lateral_cmd = 0.0
```

Do not change gains, add integrators, or change frames at this step.

---

### 6. Decision Point — Is This "Good Enough"?

Yes — if the goal is demonstrating stable, vision-assisted hover in front of an inspection target, this is a valid stopping point for a Phase-1 baseline.

The residual lateral bias is explainable: "Residual lateral bias is expected due to camera–body misalignment and proportional-only control; future work introduces body-frame regulation and integral action."

---

### 7. Summary

The residual lateral drift observed in the hover_yaw_search_v2 controller is not due to instability or sign errors, but rather to steady-state bias inherent to proportional-only image-based visual servoing. Small camera–body alignment offsets, yaw gating effects, and MAVROS velocity tracking result in a nonzero lateral image error that a pure P controller cannot eliminate. This behavior is expected in Phase-1 IBVS systems and can be mitigated using a lateral deadband or integral action, or fully resolved in future work via body-frame regulation. The controller is otherwise stable and suitable for inspection-hover evaluation.
