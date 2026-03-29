# FRC 10413 FLUX — Field Quick Card

Print and laminate. One card per role.

---

## PIT: Power-On Sequence

- [ ] Battery > 12.5V, secured with strap
- [ ] Main breaker OFF, connect battery
- [ ] Verify CAN wiring (no loose connectors)
- [ ] Main breaker ON
- [ ] Wait for RoboRIO boot (~30s, status LED solid)
- [ ] Verify robot name: Dashboard shows `FUEL initialized` or `CORAL initialized`
- [ ] Verify camera count in notification matches physical cameras (FUEL: 3, CORAL: 1)

## PIT: Health Check

- [ ] `Battery` on dashboard > 12.0V
- [ ] `Vision/Connected` = true (all cameras green)
- [ ] Each `Vision/Camera{i}/Connected` = true
- [ ] Jog each wheel by hand — verify all 4 swerve modules spin freely
- [ ] Breakers not tripped (check 40A breakers per motor)

## PRE-MATCH: Configuration

- [ ] Confirm robot selection (FUEL/CORAL) in dashboard notification
- [ ] Select auto from `Auto Chooser` (default: None)
- [ ] Verify `Vision/Enable` = true
- [ ] Set `Drive/SlowMode` as desired
- [ ] Set `Drive/SnapToAngle` as desired
- [ ] Check `Pose/X`, `Pose/Y`, `Pose/Heading` — reset if needed via `Pose/ApplyPreset`
- [ ] `Drive/Deadband` = 0.1 (default)
- [ ] Battery > 12.3V before queuing

## ON-FIELD: Before Match Start

- [ ] Enable robot, verify wheels respond
- [ ] Disable, position robot on field
- [ ] Verify alliance color on dashboard matches FMS
- [ ] Reset pose to starting position (preset or manual)
- [ ] Verify `Vision/HasTargets` = true if tags visible
- [ ] Driver: test stick inputs respond (enable/disable quickly)

## ON-FIELD: During Match (Driver)

- Slow mode: hold RIGHT TRIGGER (when `Drive/SlowMode` = ON)
- Brake: hold X button
- Reset heading: Y button (no vision) or RIGHT/LEFT BUMPER
- Drive to tag: hold Y button (with vision)
- Snap to angle: D-pad (when `Drive/SnapToAngle` = ON)

## ON-FIELD: During Match (Operator — FUEL only)

- Intake: RIGHT TRIGGER (in, hold) / LEFT TRIGGER (reverse, hold)
- Tilt: D-pad UP/DOWN (jog) / A button (stow) / B button (score position)
- Shoot: RIGHT BUMPER (hold = spin up + feed)
- Index: LEFT BUMPER (hold = index forward)

## POST-MATCH: Log Collection

- [ ] Connect laptop to robot (USB or WiFi)
- [ ] Pull log file from RoboRIO: `/home/lvuser/logs/` (latest `.wpilog`)
- [ ] Name convention: `Match{#}_{Auto}_{Result}.wpilog`
- [ ] Copy to team drive / USB stick
- [ ] Note any issues observed (brownout, vision loss, drift)

## EMERGENCY

| Symptom | Action |
|---------|--------|
| Robot won't move | Check battery > 12V, check breakers, power cycle |
| Wheels jitter/oscillate | Reduce `Drive/MaxRotRate`, check gyro calibration |
| Vision rejected constantly | Check `Vision/Camera{i}/RejectionReason`, clean lenses |
| Brownout mid-match | Battery < 7V under load, swap battery |
| Code crash | Redeploy: `./gradlew deploy` from pit laptop |
