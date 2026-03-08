# FLUX Robotics - CORAL Driver Cheat Sheet

**Robot: CORAL | Season: 2026 | Config-Driven Swerve Drivetrain**

---

## Pre-Run Checklist

### Driver Station Setup
1. Connect to robot WiFi (10.104.13.X network)
2. Open Driver Station and verify:
   - Robot communication (green)
   - Joystick detected (Port 0)
   - Battery voltage > 12V
3. Check SmartDashboard:
   - `Drive/AllHealthy` = true
   - `Drive/StatusMessage` = "Ready"

### Before Enabling
- [ ] Clear area around robot (3m minimum)
- [ ] Spotter in position
- [ ] Battery fully charged (>12.5V)
- [ ] Emergency stop tested (both bumpers simultaneously)

---

## Controller: Xbox (Port 0)

### Driving (Field-Centric)
| Control | Action |
|---------|--------|
| **Left Stick Y** | Drive forward/backward |
| **Left Stick X** | Strafe left/right |
| **Right Stick X** | Rotate left/right |

*Squared input curve applied for finer low-speed control. 5% translation deadband, 10% rotation deadband. Slew rate limiting on all axes.*

### Buttons
| Button | Action |
|--------|--------|
| **X** (hold) | Brake - locks wheels in X pattern |
| **Right Bumper** (press) | Reset field-centric heading |
| **Both Bumpers** (hold) | Emergency stop (brake + warning logged) |

---

## Autonomous Mode

Select auto routine on SmartDashboard via **Auto Chooser**:

| Option | Description |
|--------|-------------|
| **Drive Forward** (default) | PathPlanner path: 2m forward at 2 m/s |
| **Forward Turn Back** | 2m forward, 180 turn, 1m back (ends 1m from start) |
| **Do Nothing** | Robot stays still |

---

## Specs

- **Max Speed**: ~5 m/s (~11 mph)
- **Max Rotation**: 0.75 rotations/second
- **Current Limits**: 40A drive stator, 35A drive supply, 20A steer, 120A slip
- **CAN Bus**: CANdace (CANivore)
- **Weight**: ~74 kg (~163 lbs)

---

## Dashboard Indicators

### Drive/StatusMessage
| Message | Meaning |
|---------|---------|
| "Ready" | All systems normal |
| "CAN issues" | Stale odometry, check CAN wiring |
| "XX motor hot" | Motor temperature >80C, let it cool |
| "Camera N disconnected" | Vision camera offline |
| "Low battery (X.XV)" | Brownout protection active, speed reduced |

### Key Telemetry
| Key | What It Shows |
|-----|---------------|
| `Drive/SpeedPercent` | Current speed as % of max |
| `Drive/BrownoutActive` | Speed being reduced due to low voltage |
| `Drive/BrownoutSpeedScale` | Current speed multiplier (1.0 = full) |
| `Drive/PoseConfidence` | HIGH / MEDIUM / LOW / DEAD_RECKONING |
| `Drive/ActiveCommand` | Currently running drive command |

---

## Quick Troubleshooting

1. **Robot won't drive**: Check if disabled, verify controller on Port 0
2. **Driving feels wrong direction**: Press Right Bumper to reset heading
3. **Robot feels sluggish**: Check `Drive/BrownoutActive` - may need battery swap
4. **Need to stop fast**: Hold both bumpers for emergency stop
5. **"CAN issues" on dashboard**: Check CANivore (CANdace) USB connection to RoboRIO
6. **Vision not working**: Check `Drive/Vision/0/Connected`, verify camera powered

---

## Post-Run Procedures

### Download Logs
1. Keep robot powered on
2. In Driver Station: **File > Download Logs**
3. Or use SFTP: `sftp lvuser@10.104.13.2:/home/lvuser/logs/`
4. Save to `~/frc-logs/YYYY-MM-DD/` on your laptop

### Log Files to Collect
| File | Contents |
|------|----------|
| `FRC_*.wpilog` | AdvantageKit session data (all telemetry) |
| `stdout.log` | Console output, warnings |
| `stderr.log` | Error stack traces |

### After Session
- [ ] Download logs before powering off
- [ ] Note any issues in session log
- [ ] Charge battery immediately
- [ ] Report hardware issues to mentor

### Quick Log Analysis
```bash
# View recent errors
grep -i "error\|exception" stderr.log

# Check for brownouts
grep -i "brownout" stdout.log

# Check for emergency stops
grep -i "EMERGENCY STOP" stdout.log
```

---

*Team 10413 FLUX Robotics - 2026 Season - CORAL Driver Guide*
