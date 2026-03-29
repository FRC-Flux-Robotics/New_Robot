# FRC 10413 FLUX — Operations Reference

Detailed reference backing the [Quick Card](quick-card.md). Read this when something goes wrong or you need to understand a setting.

---

## 1. Robot Selection

The robot (FUEL or CORAL) is selected **at boot** from the RoboRIO's comments string.

| Robot | CAN Bus | Pigeon ID | Track Size | Cameras | Container |
|-------|---------|-----------|------------|---------|-----------|
| FUEL | "Drivetrain" | 20 | 22.25" x 22.25" | 3 (center, left, right) | FuelRobotContainer |
| CORAL | "CANdace" | 24 | 23.5" x 23.5" | 1 (front OV9281) | RobotContainer |

**To change robot:** Use the RoboRIO web dashboard (http://10.104.13.2) or WPILib RoboRIO imaging tool to set the comments string. Include "CORAL" for CORAL; anything else defaults to FUEL.

**Verification:** On boot, the Elastic dashboard notification shows:
```
[FUEL] initialized with 3 camera(s)
```

---

## 2. SmartDashboard Controls

### Drive Settings

| Key | Type | Default | What It Does |
|-----|------|---------|--------------|
| `Drive/TunableSens` | Boolean | false | OFF = fixed legacy sensitivity curves. ON = use `Drive_` and `Rot_` dashboard sliders for live tuning |
| `Drive/SlewRate` | Boolean | false | Smooths joystick inputs with acceleration limiting. Uses `driver/accelLimit` preference |
| `Drive/SlowMode` | Boolean | false | When ON, holding right trigger scales speed by `driver/slowModeScale` (default 0.3) |
| `Drive/StickClamp` | Boolean | false | Clamps combined stick X+Y magnitude to unit circle (prevents diagonal speed > 1.0) |
| `Drive/SnapToAngle` | Boolean | false | Enables D-pad snap-to-angle (0/90/180/270 degrees field-centric) |
| `Drive/Deadband` | Number | 0.1 | Joystick deadband. Applied to both translation and rotation. Updates live |
| `Drive/MaxRotRate` | Number | 4.712 | Max rotation rate in rad/s. Default = 0.75 rotations/sec. Reduce if robot spins too fast |

### Sensitivity Tuning (only active when `Drive/TunableSens` = ON)

Two-segment piecewise linear curve. Separate curves for translation (`Drive_`) and rotation (`Rot_`).

| Key | Default | Meaning |
|-----|---------|---------|
| `{prefix}Start_X` | 0.09 | Deadzone end (stick below this = 0 output) |
| `{prefix}Middle_X` | 0.6 | Inflection point (transition between segments) |
| `{prefix}Start_Y` | 0.1 | Output at deadzone edge |
| `{prefix}Middle_Y` | 0.3 / 0.5 | Output at inflection (drive / rot) |
| `{prefix}Max_Y` | 0.8 / 1.0 | Output at full stick (drive / rot) |

### Pose Controls

| Key | Type | Purpose |
|-----|------|---------|
| `Pose/X`, `Pose/Y`, `Pose/Heading` | Number (output) | Current robot pose, read-only |
| `Pose/Preset` | Chooser | Select preset: Origin, Left, Right, HUB |
| `Pose/ApplyPreset` | Boolean | Set to true to apply selected preset. Auto-resets to false |
| `Pose/ResetX`, `ResetY`, `ResetHeading` | Number | Manual pose reset values |
| `Pose/AllianceRelative` | Boolean | If true, manual reset mirrors for red alliance |
| `Pose/ResetTrigger` | Boolean | Set to true to apply manual reset. Auto-resets to false |

### Vision Controls

| Key | Type | Purpose |
|-----|------|---------|
| `Vision/Enable` | Boolean | Master vision on/off. Turn off if vision is causing pose jumps |
| `Vision/UseNewStdDevs` | Boolean | true = distance²/tagCount formula. false = legacy 1+distance²/30 |

---

## 3. Vision Status Indicators

### Aggregate (all cameras)

| Key | Healthy Value | Meaning |
|-----|---------------|---------|
| `Vision/Connected` | true | At least one camera connected |
| `Vision/HasTargets` | true (when tags visible) | At least one camera sees an AprilTag |
| `Vision/BestTagId` | 1-22 | ID of the largest-area tag across all cameras. -1 = no targets |
| `Vision/EstimatedPose` | Near actual position | Last accepted vision pose estimate |

### Per-Camera (`Vision/Camera{i}/`)

| Key | Meaning |
|-----|---------|
| `Connected` | Camera connection status. False after ~1s of no data (50 frames) |
| `HasTargets` | This camera sees at least one AprilTag |
| `TagCount` | Number of tags this camera sees |
| `Rejected` | true = last measurement was rejected |
| `RejectionReason` | Why it was rejected (see table below) |

### Rejection Reasons

| Reason | Threshold | What To Do |
|--------|-----------|------------|
| `TOO_AMBIGUOUS` | Single-tag ambiguity > 30% | Clean lens, get closer, or ensure tag not partially occluded |
| `OUT_OF_FIELD` | Pose > 0.5m outside field bounds | Camera calibration issue or bad pose — recalibrate |
| `Z_ERROR` | Estimated Z > 0.75m from floor | Camera mount loose or bumped |
| `ANGULAR_VEL_TOO_HIGH` | Robot spinning > 120 deg/s | Normal during fast turns, no action needed |
| `MOVING_TOO_FAST` | Robot moving > 3.0 m/s | Normal during sprints, no action needed |

**Rejection counts** persist per camera per reason at `Vision/Camera{i}/Rejections/{REASON}`. High `TOO_AMBIGUOUS` or `Z_ERROR` counts indicate hardware issues.

---

## 4. Auto Routines

| Name | Description | Duration | Notes |
|------|-------------|----------|-------|
| None | No-op | — | Default. Robot sits still |
| Drive Forward | 1 m/s forward | 2s | Basic mobility check |
| Forward-Turn-Back | Forward → 90° turn → forward | ~4s | Tests translation + rotation |
| PathPlanner Test | Triangle path via pathfinding | ~15s | Tests PathPlanner integration |
| Precision Square | 2m square, logs error | ~20s | Check `PrecisionTest/ErrorMeters` and `ErrorDegrees` after |
| Drive to Nearest Tag | Approach nearest AprilTag | 10s timeout | Vision required. Uses P-controller |

---

## 5. Driver Preferences (Persistent)

Stored on RoboRIO. Survive redeploys and power cycles. Change via Preferences widget in Shuffleboard.

| Key | Default | Range | Effect |
|-----|---------|-------|--------|
| `driver/maxSpeedScale` | 1.0 | 0.0–1.0 | Scales max drive speed |
| `driver/maxRotationScale` | 1.0 | 0.0–1.0 | Scales max rotation speed |
| `driver/accelLimit` | 3.0 | 0.5–20.0 | Slew rate limit for translation (m/s²) |
| `driver/rotAccelLimit` | 3.0 | 0.5–20.0 | Slew rate limit for rotation (rad/s²) |
| `driver/slowModeScale` | 0.3 | 0.05–0.8 | Speed multiplier when slow mode trigger held |
| `driver/decelMultiplier` | 2.0 | 1.0–5.0 | How much faster deceleration is than acceleration |

---

## 6. Current Limits

These are safety-critical. Do not change without understanding the consequences.

| Motor | Stator Limit | Supply Limit | Why |
|-------|-------------|--------------|-----|
| Drive (TalonFX) | 60A | 120A | Prevents brownouts. Original 40A/35A caused brownouts |
| Steer (TalonFX) | 60A | — | Prevents brownouts during module rotation |
| Slip threshold | 120A | — | Wheel slip detection |

FUEL mechanism motors (intake, shooter, etc.) have per-mechanism limits defined in `MechanismConfigs.java`.

---

## 7. Controller Mapping

### Port 0 — Driver (Xbox)

| Input | Action |
|-------|--------|
| Left Stick | Translation (field-centric) |
| Right Stick X | Rotation |
| Right Trigger | Slow mode (when `Drive/SlowMode` ON) |
| X Button (hold) | X-pattern wheel brake |
| Y Button | Drive to tag (vision) / Reset heading (no vision) |
| Right Bumper | Reset heading |
| Left Bumper | Reset heading |
| Start | Reset pose to origin |
| D-Pad | Snap to angle (when `Drive/SnapToAngle` ON) |

### Port 0 — Driver Overrides (FUEL only)

| Input | Action |
|-------|--------|
| Y Button | Reset heading (overrides drive-to-tag) |
| Right Bumper | Shoot (spin up + feed when at speed) |
| Left Bumper | Index forward |
| Right Trigger | Intake in |
| Left Trigger | Intake reverse |
| A Button | Tilt to stow position |
| B Button | Tilt to score position |
| D-Pad Up/Down | Jog tilt up/down |

### Port 1 — SysId Controller (Xbox, testing only)

| Input | Action |
|-------|--------|
| A | SysId Dynamic Forward |
| B | SysId Dynamic Reverse |
| X | SysId Quasistatic Forward |
| Y | SysId Quasistatic Reverse |
| Left Bumper | Wheel Radius Characterization |

---

## 8. Post-Match Log Collection

### Where Logs Live

AdvantageKit writes `.wpilog` files to the RoboRIO filesystem:
```
/home/lvuser/logs/
```

Each match produces one log file. File names are timestamped by AdvantageKit.

### How to Pull Logs

**Option A: USB (fastest, no network needed)**
1. Connect USB-A cable from laptop to RoboRIO USB port
2. SSH or SFTP to `10.104.13.2` (or `172.22.11.2` over USB)
3. Copy files from `/home/lvuser/logs/`

```bash
# From pit laptop
scp lvuser@10.104.13.2:/home/lvuser/logs/*.wpilog ./match-logs/
```

**Option B: Over WiFi/Ethernet**
1. Connect to robot network (10.104.13.X)
2. Same `scp` command as above

**Option C: USB flash drive**
1. Plug USB drive into RoboRIO USB port
2. If configured, AdvantageKit can auto-copy to mounted USB
3. Check `/media/sda1/` on the RoboRIO

### Log Naming Convention

Rename logs after pulling for easy identification:
```
{Event}_{Qual|SF|F}{MatchNum}_{Alliance}{Station}.wpilog
```
Example: `Ventura_Q23_Blue2.wpilog`

### Log Storage Checklist

- [ ] Pull log file immediately after match (before next match overwrites RoboRIO storage)
- [ ] Rename with match info
- [ ] Note in a log sheet:
  - Match number and result
  - Auto routine used
  - Any observed issues (brownout, vision loss, drift, mechanism jam)
  - Battery voltage at start and end
  - Rejection count spikes
- [ ] Copy to team cloud drive before leaving venue

### Viewing Logs

**AdvantageScope** (recommended):
1. Download from [https://github.com/Mechanical-Advantage/AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)
2. File > Open Log > select `.wpilog` file
3. Key signals to check:
   - `Drive/` — pose, velocity, module states
   - `Vision/Camera{i}/` — connected, rejected, rejection reasons
   - `Vision/EstimatedPose` vs `Drive/Pose` — compare vision vs odometry drift
   - `Battery` — voltage sag under load

**Replay Mode** (for debugging):
1. Place `.wpilog` file where `LogFileUtil.findReplayLog()` can find it
2. Set robot to `REPLAY` mode (requires code change or sim launch flag)
3. Run `./gradlew simulateJava`
4. AdvantageKit replays all inputs through the code, producing a `_sim.wpilog`
5. Compare `_sim.wpilog` against original to verify code changes would have improved behavior

### Storage Management

The RoboRIO has limited flash storage (~500MB usable). Logs can fill this up over a competition day.

- **Check free space:** `ssh lvuser@10.104.13.2 'df -h /home/lvuser/logs/'`
- **Clear old logs after pulling:** `ssh lvuser@10.104.13.2 'rm /home/lvuser/logs/*.wpilog'`
- **Warning signs:** Deploy fails with disk full error, robot boot slows down

---

## 9. Troubleshooting

### Robot Won't Drive

| Check | How | Fix |
|-------|-----|-----|
| Battery voltage | Dashboard `Battery` | Swap if < 12.0V |
| CAN bus | Driver Station CAN metrics | Reseat CAN connectors |
| Breakers | Physical inspection | Reset tripped breakers |
| Gyro | Dashboard heading drifting at rest | Power cycle, let gyro calibrate 2-3s |
| Code | Dashboard shows no errors | Redeploy: `./gradlew deploy` |
| Robot selection | Dashboard notification | Verify RoboRIO comments match physical robot |

### Vision Problems

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `Connected` = false | Camera unplugged or coprocessor down | Check USB/Ethernet to coprocessor, restart PhotonVision |
| All measurements rejected | Camera bumped (Z_ERROR) | Re-mount camera, recalibrate |
| High ambiguity rejections | Dirty lens, partial tag occlusion | Clean lens, ensure full tag visibility |
| Pose jumps | Bad camera calibration or wrong transform | Recalibrate camera in PhotonVision, verify `Robots.java` transforms |
| `Vision/Enable` = false | Accidentally disabled | Set `Vision/Enable` = true on dashboard |

### Brownouts

| Symptom | Cause | Fix |
|---------|-------|-----|
| Robot reboots mid-match | Battery voltage < 6.8V | Use fresher battery, reduce aggressive acceleration |
| Motors cut out briefly | Momentary voltage sag | Normal under high load. Current limits should prevent sustained brownout |
| Repeated brownouts | Bad battery or loose connection | Load-test battery, check all power connections |

### Deployment

```bash
# From pit laptop, must be on 10.104.13.X network
cd /home/boris/frc/new
./gradlew deploy
```

| Error | Fix |
|-------|-----|
| "No RoboRIO found" | Check network connection (WiFi or USB) |
| Build fails | Run `./gradlew build` first, fix compile errors |
| Disk full | Pull logs then clear: `ssh lvuser@10.104.13.2 'rm /home/lvuser/logs/*.wpilog'` |
| "Target not found" | Verify team number 10413 in `.wpilib/wpilib_preferences.json` |
