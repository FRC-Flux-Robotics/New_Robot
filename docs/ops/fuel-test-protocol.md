# FUEL Test Protocol — Small Field

**Date:** 2026-03-29
**Robot:** FUEL (CAN bus "Drivetrain", Pigeon 20)
**Equipment needed:** Fully charged batteries (3+), laptop with code, Xbox controllers (2), AprilTags (2-4 printed, mounted at tag height), fuel game pieces, tape measure

---

## Phase 0: Pre-Power Checks (robot OFF)

- [ ] Inspect wiring: CAN chain tight, battery leads secure, breakers seated
- [ ] All 4 swerve modules spin freely by hand (drive + steer)
- [ ] Mechanisms move freely by hand: intake rollers, tilter, hood, indexer, feeder, shooter wheels
- [ ] Cameras mounted securely: center (OV9281-5), left (OV9281-5-left), right (OV9281-5-right)
- [ ] No loose bolts, zip ties, or debris in mechanisms
- [ ] Deploy latest code: `cd /home/boris/frc/new && ./gradlew deploy`

---

## Phase 1: Power-On & Dashboard (5 min)

- [ ] Fresh battery (> 12.5V), breaker ON
- [ ] Dashboard shows: `[FUEL] initialized with 3 camera(s)`
- [ ] `Battery` > 12.0V on dashboard
- [ ] All 3 cameras: `Vision/Camera0/Connected`, `Camera1/Connected`, `Camera2/Connected` = true
- [ ] CAN devices: check Driver Station diagnostic tab — no missing devices, 0% error rate
- [ ] Record battery voltage: ______V

**STOP if any camera disconnected or CAN errors > 1%. Fix before continuing.**

---

## Phase 2: Drivetrain — Wheels on Ground (10 min)

### 2A: Basic Drive

- [ ] Enable, push left stick gently forward — robot drives forward (not sideways/backward)
- [ ] Left stick left — robot strafes left
- [ ] Right stick — robot rotates
- [ ] All 4 modules tracking (no stuttering, grinding, or one module lagging)
- [ ] Release sticks — robot stops cleanly, wheels don't drift

### 2B: Field-Centric Verification

- [ ] Face robot away from you, push stick forward — robot drives away (not toward you)
- [ ] Rotate robot 180°, push stick forward — robot still drives away from you
- [ ] If inverted: Y button to reset heading, retry

### 2C: Heading Reset

- [ ] Press Y — heading resets to 0
- [ ] Verify field-centric correct after reset

### 2D: Brake

- [ ] Hold X — wheels form X-pattern
- [ ] Try to push robot — should resist
- [ ] Release X — normal driving resumes

### 2E: Speed Modes

- [ ] Set `Drive/SlowMode` = true on dashboard
- [ ] Hold right trigger — noticeably slower
- [ ] Release trigger — full speed returns
- [ ] Set `Drive/SlowMode` = false

### 2F: Odometry

- [ ] Reset pose to origin (Start button)
- [ ] Drive forward ~2m by tape measure
- [ ] Check `Pose/X` reads ~2.0 (within 0.3m acceptable)
- [ ] Drive back to start — `Pose/X` near 0 (within 0.3m)
- [ ] Rotate in place 360° — `Pose/Heading` returns near 0° (within 5°)

**Record odometry accuracy: forward ____m (expected 2.0), heading ____° (expected 0)**

---

## Phase 3: Mechanisms — One at a Time (15 min)

**SAFETY: Robot on blocks or held. One person ready at breaker. Test each mechanism individually before combining.**

### 3A: Intake (CAN 1+2, dual motor)

- [ ] Hold RIGHT TRIGGER (driver) — rollers spin inward
- [ ] Release — stops
- [ ] Hold RIGHT BUMPER (driver) — rollers reverse (outward)
- [ ] Release — stops
- [ ] Check both motors spinning (dual motor, should be matched)
- [ ] Current draw reasonable (dashboard, expect < 30A each)

### 3B: Tilter (CAN 20, position)

- [ ] POV LEFT (driver) — tilter jogs down
- [ ] POV RIGHT (driver) — tilter jogs up
- [ ] A button (driver) — 4 sequential jogs down (deploy position)
- [ ] Verify soft limits: tilter stops at 0.0 (forward) and -17.0 (reverse), doesn't slam
- [ ] Motor holds position when released

### 3C: Indexer (CAN 4, velocity)

- [ ] RIGHT BUMPER (operator) — indexer spins
- [ ] Release — stops
- [ ] Direction correct (moves fuel toward shooter)

### 3D: Feeder (CAN 3, velocity)

- [ ] RIGHT TRIGGER (operator) — feeder spins (backward direction per code)
- [ ] Release — stops

### 3E: Shooter (CAN 11+10, dual motor, counter-rotating)

- [ ] START button (operator) — toggle shooter ON at default speed (2600 RPM)
- [ ] Both wheels spin (counter-rotating: one CW, one CCW)
- [ ] Spin-up time: ____s to reach target speed
- [ ] Dashboard `Shooter/AtTarget` = true when at speed
- [ ] START again — toggle OFF, wheels coast down
- [ ] POV RIGHT + LEFT BUMPER (operator) — speed step up
- [ ] POV LEFT + LEFT BUMPER (operator) — speed step down (stops at 0)

### 3F: Hood (CAN 12, position)

- [ ] POV UP (operator) — hood jogs up
- [ ] POV DOWN (operator) — hood jogs down
- [ ] Soft limits hold: stops at 0.0 (reverse) and 20.0 (forward)
- [ ] Motor holds position when released

### 3G: Range Presets (operator)

- [ ] A button — short range preset: shooter speed + hood angle change
- [ ] B button — mid range preset
- [ ] Y button — long range preset
- [ ] Verify hood and shooter respond to each

### 3H: Encoder Reset

- [ ] BACK button (driver) — resets hood and tilter positions to 0
- [ ] Verify both mechanism positions read 0 on dashboard

---

## Phase 4: Vision (10 min)

**Setup: Place 2-4 printed AprilTags around test area at tag height (~1.3m center). Note tag IDs.**

### 4A: Detection

- [ ] Point robot at tag — `Vision/HasTargets` = true
- [ ] `Vision/BestTagId` matches the visible tag ID
- [ ] Point away — `Vision/HasTargets` = false
- [ ] Test each camera individually: block 2 cameras, verify remaining one detects

### 4B: Per-Camera Health

- [ ] `Vision/Camera0/Connected` = true, sees tags
- [ ] `Vision/Camera1/Connected` = true, sees tags
- [ ] `Vision/Camera2/Connected` = true, sees tags
- [ ] No persistent rejection reasons (check `Vision/Camera{i}/RejectionReason`)

### 4C: Pose Estimation

- [ ] Place robot at known position, reset pose
- [ ] Drive around, return to same spot
- [ ] `Vision/EstimatedPose` vs `Pose/X,Y` — should be close (within 0.5m)
- [ ] Compare `Field` widget: vision pose ghost vs odometry pose

### 4D: Rejection Filtering

- [ ] Spin robot fast in place — `ANGULAR_VEL_TOO_HIGH` rejections appear (expected)
- [ ] Drive fast — `MOVING_TOO_FAST` rejections appear (expected)
- [ ] These should clear when robot slows down

### 4E: Enable/Disable

- [ ] Set `Vision/Enable` = false — pose should rely only on odometry
- [ ] Set `Vision/Enable` = true — vision corrections resume

---

## Phase 5: Integrated Sequences (10 min)

**Robot on ground, open area. Run these only after Phases 2-4 pass.**

### 5A: Intake + Index Cycle

- [ ] Driver: RIGHT TRIGGER to intake fuel
- [ ] Fuel enters and reaches indexer area
- [ ] Operator: RIGHT BUMPER to index fuel forward
- [ ] Fuel advances to shooter area

### 5B: Shoot Cycle

- [ ] Operator: START to spin up shooter
- [ ] Wait for `Shooter/AtTarget` = true
- [ ] Operator: RIGHT TRIGGER to feed
- [ ] Fuel launches
- [ ] Operator: START to stop shooter

### 5C: Range Shoot Command

- [ ] Operator: LEFT BUMPER (hold) — triggers RangeShootCmd
- [ ] Shooter spins up, hood adjusts, feeder + indexer engage when at speed
- [ ] Release — all stop
- [ ] **Watch for jams**: if fuel doesn't exit, release immediately

### 5D: Full Cycle While Driving

- [ ] Drive slowly, intake a fuel piece
- [ ] Stop, run shoot cycle
- [ ] Verify driving doesn't interfere with mechanisms

---

## Phase 6: Autonomous (5 min)

### 6A: Drive Forward

- [ ] Select "Drive Forward" from `Auto Chooser`
- [ ] Place robot, enable autonomous
- [ ] Robot drives forward ~2m, stops
- [ ] Measure actual distance: ____m (expected ~2.0)

### 6B: Forward-Turn-Back

- [ ] Select "Forward-Turn-Back"
- [ ] Enable — robot drives forward, turns 90°, drives forward, stops
- [ ] Reasonable path? No oscillation?

### 6C: Precision Square (if space allows)

- [ ] Select "Precision Square"
- [ ] Enable — robot drives 2m square
- [ ] Check `PrecisionTest/ErrorMeters` and `PrecisionTest/ErrorDegrees`
- [ ] Record: position error ____m, heading error ____°

---

## Phase 7: Post-Test

### Log Collection

- [ ] Pull logs: `scp lvuser@10.104.13.2:/home/lvuser/logs/*.wpilog ./test-logs/`
- [ ] Rename: `SmallField_Test_2026-03-29.wpilog`

### Record Results

| Test | Result | Notes |
|------|--------|-------|
| Drive forward/strafe/rotate | PASS / FAIL | |
| Field-centric | PASS / FAIL | |
| Brake | PASS / FAIL | |
| Odometry (2m accuracy) | ____m error | |
| Heading (360° return) | ____° error | |
| Intake in/out | PASS / FAIL | |
| Tilter jog + soft limits | PASS / FAIL | |
| Indexer | PASS / FAIL | |
| Feeder | PASS / FAIL | |
| Shooter spin-up + counter-rotate | PASS / FAIL | Spin-up time: ____s |
| Hood jog + soft limits | PASS / FAIL | |
| Range presets (S/M/L) | PASS / FAIL | |
| Camera 0 detection | PASS / FAIL | |
| Camera 1 detection | PASS / FAIL | |
| Camera 2 detection | PASS / FAIL | |
| Vision pose estimation | PASS / FAIL | Error: ____m |
| Intake→Index→Shoot cycle | PASS / FAIL | |
| RangeShootCmd | PASS / FAIL | |
| Auto: Drive Forward | PASS / FAIL | Distance: ____m |
| Auto: Precision Square | PASS / FAIL | Error: ____m, ____° |

### Issues Found

| # | Description | Severity | Action |
|---|-------------|----------|--------|
| 1 | | | |
| 2 | | | |
| 3 | | | |

### Battery Log

| Battery | Start V | End V | Used For |
|---------|---------|-------|----------|
| 1 | | | |
| 2 | | | |
| 3 | | | |
