# Vision Camera Calibration Guide

**Robot:** FUEL (3 cameras) or CORAL (1 camera)
**Time required:** 15-30 minutes
**Equipment:** Robot, laptop with SmartDashboard/Shuffleboard, 2+ visible AprilTags

---

## Overview

Camera transforms tell the robot where each camera is mounted (X/Y/Z offset from robot center, pitch/yaw angle). If these are wrong, vision pose estimation will be inaccurate. This guide walks through identifying, validating, and tuning camera transforms without redeploying code.

### Dashboard Sections Used

| Section | Purpose |
|---------|---------|
| `CamID/` | Identify which physical camera is which name |
| `CamVal/` | Automated validation results |
| `CamTune/` | Live transform adjustment |
| `Vision/Camera*/` | Per-camera pose and error telemetry |

---

## Step 1: Identify Cameras (5 min)

**Goal:** Map each physical camera on the robot to its PhotonVision name.

FUEL cameras: `OV9281-5` (center?), `OV9281-2` (left?), `OV9281-4` (right?)

### Procedure

1. Power on robot, connect to dashboard
2. Open `CamID/` section — you'll see entries like:
   ```
   CamID/Cam0: OV9281-5: sees tags [1, 2] best=ID2 yaw=-3.2°
   CamID/Cam1: OV9281-2: sees tags [3] best=ID3 yaw=22.1°
   CamID/Cam2: OV9281-4: sees tags [6] best=ID6 yaw=-18.5°
   ```
3. **Cover the CENTER camera with your hand**
4. Watch which `CamID/` entry changes to `BLOCKED (no tags)`
5. Write down: CENTER = Cam___ = ___________  (name)
6. Uncover, repeat for LEFT camera
7. Write down: LEFT = Cam___ = ___________
8. Repeat for RIGHT camera
9. Write down: RIGHT = Cam___ = ___________

### Quick Checks Without Covering

- **Yaw value** indicates viewing direction: positive yaw = tags are to camera's left
- **Tag IDs** — a camera seeing only tags on one side of the field is probably facing that direction

### Record Your Results

| Position | Dashboard Index | PhotonVision Name |
|----------|----------------|-------------------|
| Center   | Cam___         |                   |
| Left     | Cam___         |                   |
| Right    | Cam___         |                   |

**Keep this table — you'll need it for Step 3.**

---

## Step 2: Run Validation Auto (2 min)

**Goal:** Check if current camera transforms are accurate.

### Procedure

1. Place robot on the field where cameras can see AprilTags (ideally 2+ tags visible)
2. In Auto Chooser, select **"Camera Validation"**
3. Enable autonomous
4. Robot will:
   - Hold still for 3 seconds (collecting samples at 0°)
   - Rotate 90° and hold (collecting at 90°)
   - Repeat at 180° and 270°
   - Total time: ~20 seconds
5. Read results in `CamVal/` section

### Reading Results

**Per-camera accuracy** (`CamVal/Cam0`, `Cam1`, `Cam2`):
```
OV9281-5: PASS avg=0.042m max=0.067m (45 samples)
OV9281-2: FAIL avg=0.213m max=0.340m (38 samples)
```
- PASS = average error < 15cm (transform is good)
- FAIL = needs adjustment

**Inter-camera consistency** (`CamVal/Consistency`):
```
FAIL avg=0.182m max=0.290m (120 pairs)
```
- Compares cameras against each other (does NOT depend on pose reset accuracy)
- PASS = cameras agree within 10cm
- FAIL = at least one camera's transform is wrong

**Recommended fixes** (`CamVal/Cam1/Fix`):
```
Adjust X by -5.2 cm (camera is further back than measured). Adjust Y by +3.1 cm (camera is further left). | Current: X=-27.6cm Y=12.7cm yaw=30.0deg
```

**Overall result** (`CamVal/Result`):

| Result | Meaning |
|--------|---------|
| `ALL PASSED` | Transforms are good |
| `CONSISTENCY OK, ABSOLUTE ERRORS` | Cameras agree but odometry pose may be wrong — transforms are fine |
| `FAILED — check per-camera Fix` | At least one transform needs adjustment |

### Understanding Error Patterns

| Fix message says | Problem | What to measure |
|------------------|---------|-----------------|
| `Adjust X by +N cm` | Camera is further forward/back than recorded | Re-measure distance from robot center to camera along robot's forward axis |
| `Adjust Y by +N cm` | Camera is further left/right than recorded | Re-measure lateral offset from robot centerline |
| `YAW offset suspected` | Camera angle is wrong | Re-measure the angle the camera is pointed |
| `Adjust yaw by +N deg` | Camera rotation is off | The camera is rotated slightly from where you thought |

---

## Step 3: Tune Transforms (10-20 min)

**Goal:** Adjust camera transforms until all cameras pass validation.

### Option A: Apply Validation Recommendations (Quick)

1. Read the `CamVal/CamX/Fix` recommendation
2. In `CamTune/CamX/`, adjust the suggested values
   - Example: if Fix says "Adjust X by -5.2 cm" and current X_cm is -27.6:
   - Set `CamTune/Cam1/X_cm` to `-32.8` (-27.6 + -5.2)
3. Click **`CamTune/Apply`** = `true`
4. Re-run validation auto
5. Repeat until all cameras PASS

### Option B: Enter Fresh Measurements (Thorough)

Use the camera identification table from Step 1 to know which `CamTune/CamX/` to edit.

**How to measure** (all measurements from robot center point, at floor level):

| Value | How to measure | Units |
|-------|---------------|-------|
| `X_cm` | Distance forward (+) or backward (-) from robot center to camera | cm |
| `Y_cm` | Distance left (+) or right (-) from robot centerline to camera | cm |
| `Z_cm` | Height from floor to camera lens | cm |
| `Pitch_deg` | Camera tilt: 0° = level, negative = tilted down | degrees |
| `Yaw_deg` | Camera rotation: 0° = forward, +30° = looking left, -30° = looking right | degrees |
| `Roll_deg` | Camera roll (usually 0 unless camera is mounted sideways) | degrees |

1. Measure each camera carefully with a tape measure
2. Enter values in `CamTune/CamX/` for the correct camera index
3. Click **`CamTune/Apply`**
4. Run validation auto
5. Fine-tune based on Fix recommendations
6. Repeat until all PASS

### Saving Your Work

- **`CamTune/Apply`** — pushes values to the pose estimator immediately (lost on reboot)
- **`CamTune/Save`** — persists values to RoboRIO Preferences (survives reboots)
- Always click **Save** once you're happy with the results

---

## Step 4: Verify (5 min)

After all cameras pass validation:

1. **Drive test:** Drive the robot around while watching `Vision/CameraDisagreementMeters`
   - Should stay below 0.15m while driving at moderate speed
2. **Pose accuracy test:**
   - Place robot at a known field position
   - Reset pose on dashboard
   - Drive a loop and return to the starting position
   - Check `Vision/PoseX` and `Vision/PoseY` — should match the known position within ~10cm
3. **Multi-tag test:** Drive to where multiple cameras see multiple tags
   - `Vision/CameraDisagreementMeters` should be < 0.10m

---

## Troubleshooting

### "NO DATA (no tags seen)" for a camera

- Camera can't see any AprilTags from that rotation angle
- Move robot closer to tags or to a position with tags in more directions
- Check `CamID/CamX` — if it says `DISCONNECTED`, check camera wiring/USB

### Consistency PASS but Absolute FAIL

- The cameras agree with each other, which means transforms are correct relative to each other
- The absolute error means the pose reset was inaccurate
- This is fine — your transforms are good

### One camera consistently worse than others

- That camera's physical measurements are wrong
- Use the Fix recommendations as a starting point
- Common issues:
  - Y offset: mounting bracket puts camera further left/right than the edge of the bracket suggests
  - Pitch: camera is tilted down more than measured
  - Yaw: camera is rotated a few degrees from the bracket angle

### Error varies wildly between rotation stations

- Indicates a yaw measurement error — the camera angle is wrong
- Fix recommendation will say "YAW offset suspected"
- Re-measure the camera's pointing angle with more care

### Values to update in code (after competition season)

Once tuning is final, update `Robots.java` with the calibrated values so they become the new code defaults:

```java
// In Robots.java, update the camera config:
.camera(new CameraConfig(
    "OV9281-2",
    new Transform3d(
        new Translation3d(
            Units.inchesToMeters(X_INCHES),  // from CamTune X_cm / 2.54
            Units.inchesToMeters(Y_INCHES),  // from CamTune Y_cm / 2.54
            Units.inchesToMeters(Z_INCHES)), // from CamTune Z_cm / 2.54
        new Rotation3d(
            Math.toRadians(ROLL_DEG),        // from CamTune Roll_deg
            Math.toRadians(PITCH_DEG),       // from CamTune Pitch_deg
            Math.toRadians(YAW_DEG)))))      // from CamTune Yaw_deg
```

---

## Quick Reference Card

```
IDENTIFY:   CamID/Cam0..2     Cover camera → see which goes BLOCKED
VALIDATE:   Auto Chooser → "Camera Validation" → enable autonomous
RESULTS:    CamVal/Cam0..2    PASS/FAIL + avg/max error
FIX:        CamVal/Cam0/Fix   Recommended adjustments
TUNE:       CamTune/Cam0/X_cm, Y_cm, Z_cm, Pitch_deg, Yaw_deg
APPLY:      CamTune/Apply     Push changes (instant, temporary)
SAVE:       CamTune/Save      Persist to RoboRIO (survives reboot)
MONITOR:    Vision/CameraDisagreementMeters  (< 0.10m = good)
```
