# Camera Calibration Checklist

Step-by-step guide for calibrating camera positions on the robot.
Any team member can do this — no code changes needed.

## What You Need

- Robot powered on, connected to driver station
- Dashboard open (Elastic or SmartDashboard)
- At least 2-3 AprilTags visible from robot position
- Tape measure (for initial setup only)

## When to Calibrate

- After mounting or remounting a camera
- After the robot frame gets hit hard
- If vision pose (purple on field view) doesn't match odometry (blue)
- If `Vision/CameraDisagreementMeters` is consistently > 0.10m

---

## Quick Calibration (AutoApply)

If cameras are already roughly set up and you just need to fine-tune:

1. **Position robot** where 2+ cameras can see AprilTags
2. **Run Camera Validation**: Auto Chooser → "Camera Validation" → Enable
   - Robot will sit still for 3s, rotate 90°, repeat 4 times (~20 seconds)
   - Don't bump the robot during this
3. **Read results** in `CamVal/` on dashboard:
   - `CamVal/Cam0` — PASS/FAIL with error numbers
   - `CamVal/Cam0/Fix` — plain English fix description
4. **Press `CamVal/AutoApply`** (set to true) — corrections are applied automatically
5. **Run validation again** to verify improvement
6. **Press `CamTune/Save`** (set to true) to persist to RoboRIO

Done! Values survive reboots until you Save again.

---

## First-Time Setup (New Camera)

### Step 1: Measure Camera Position

Measure from **robot center** (where the four module axes cross) to the **camera lens**:

| Axis | Direction | Example |
|------|-----------|---------|
| X | Forward = positive, Backward = negative | Camera behind center → negative X |
| Y | Left = positive, Right = negative | Camera on left side → positive Y |
| Z | Up from floor to lens | Always positive |

Use **centimeters**. Write the numbers down.

### Step 2: Measure Camera Angle

| Axis | What It Means | When to Use |
|------|---------------|-------------|
| Yaw | Camera points left/right of straight ahead | Side-facing cameras |
| Pitch | Camera tilts up/down | Camera angled down to see closer tags |
| Roll | Camera is rotated/tilted sideways | Usually 0 |

Use **degrees**. Most cameras only need Yaw and maybe Pitch.

### Step 3: Enter Values on Dashboard

Open `CamTune/` on SmartDashboard or Elastic:

```
CamTune/Cam0/Name     → camera name (read-only)
CamTune/Cam0/X_cm     → your X measurement
CamTune/Cam0/Y_cm     → your Y measurement
CamTune/Cam0/Z_cm     → your Z measurement
CamTune/Cam0/Roll_deg → usually 0
CamTune/Cam0/Pitch_deg → tilt angle (negative = looking down)
CamTune/Cam0/Yaw_deg  → rotation (positive = pointing left)
```

### Step 4: Apply and Verify

1. Set `CamTune/Apply` to true
2. Check `Vision/Camera0/ErrorMeters` on dashboard — should be < 0.15m
3. If error is too high, run the **Quick Calibration** above

### Step 5: Save

Set `CamTune/Save` to true. Values persist on the RoboRIO across reboots.

---

## Which Camera Is Which?

If you don't know which camera index matches which physical camera:

1. Look at `CamID/Cam0`, `CamID/Cam1`, etc. on dashboard
2. Cover one camera with your hand
3. That entry will show "BLOCKED (no tags)"
4. Now you know which index is which

---

## Troubleshooting

| Problem | Check |
|---------|-------|
| "NO DATA" after validation | Move robot closer to tags, check camera connection |
| Camera shows "DISCONNECTED" | Check USB cable, verify camera name in PhotonVision matches config |
| Error gets worse after AutoApply | Undo: re-enter original values in CamTune, or redeploy code to reset |
| Cameras disagree with each other | Calibrate each one separately — cover other cameras during validation |
| Good accuracy but bad heading | Yaw offset — adjust `Yaw_deg` by the amount shown in Fix text |
| "ErrorVariesWithRotation" | Yaw is off — the Fix text will tell you which direction |

## Dashboard Keys Reference

| Key | What It Shows |
|-----|--------------|
| `CamTune/Cam0/X_cm` | Editable camera X position (cm) |
| `CamTune/Apply` | Button: apply current CamTune values |
| `CamTune/Save` | Button: save to RoboRIO (persists across reboots) |
| `CamVal/AutoApply` | Button: apply corrections from last validation |
| `CamVal/Cam0` | Validation result: PASS/FAIL with error |
| `CamVal/Cam0/Fix` | Plain English fix recommendation |
| `Vision/Camera0/ErrorMeters` | Live error vs odometry |
| `Vision/CameraDisagreementMeters` | How much cameras disagree |
| `CamID/Cam0` | Camera name + what it sees right now |
