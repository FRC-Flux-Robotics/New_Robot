# FRC Team 10413 FLUX Robotics - 2026 Season

Swerve drivetrain robot code for the 2026 FRC season. Built with WPILib 2026.2.1, CTRE Phoenix 6, AdvantageKit logging, and PhotonVision.

## Hardware

- **Drive:** 4-module swerve (field-centric, open-loop voltage)
- **Motors:** TalonFX (Kraken/Falcon) for drive and steer
- **Sensors:** CANcoders (absolute steering), Pigeon 2 IMU (heading)
- **CAN bus:** CANivore ("CANdace")
- **Vision:** PhotonVision AprilTag cameras (optional, configured per-robot)
- **Max speed:** 4.99 m/s | **Max rotation:** ~4.7 rad/s

## Quick Start

```bash
# Build and run tests
./gradlew build

# Run tests only
./gradlew test

# Run a specific test
./gradlew test --tests "TestClassName"

# Simulate (opens WPILib simulation GUI)
./gradlew simulateJava

# Deploy to robot (must be on 10.104.13.X network)
./gradlew deploy
```

Requires Java 17.

## Driver Controls

Xbox controller on port 0:

| Input | Action |
|-------|--------|
| Left stick | Translate (field-centric) |
| Right stick X | Rotate |
| Left trigger (hold) | Aim at target while driving |
| Right trigger (hold) | Slow mode (30% speed) |
| A button (hold) | Drive to AprilTag 3 (vision, if camera configured) |
| X button | Brake (lock wheels in X pattern) |
| Right bumper | Reset field-centric heading |
| Both bumpers | Emergency stop |
| Back + Y/X | SysId dynamic (forward/reverse) |
| Start + Y/X | SysId quasistatic (forward/reverse) |

Joystick inputs use an expo response curve (default exponent 2.0) with slew rate limiting (4.0 translation, 5.0 rotation) for smooth acceleration. All input parameters are live-tunable via `DriverPreferences` on the SmartDashboard Preferences widget.

## Architecture

### Config-Driven Drivetrain Library

The drivetrain lives in `frc.lib.drivetrain` as a reusable library. A single `DrivetrainConfig` object (built via builder pattern) captures all hardware IDs, gearing, PID tuning, current limits, mechanical dimensions, and camera configuration. Different robots just need different configs.

Library classes:

| Class | Purpose |
|-------|---------|
| `DrivetrainConfig` | All drivetrain parameters in one object (builder pattern) |
| `DriveInterface` | Clean API for consumers (teleop, auto, subsystems) |
| `SwerveDrive` | Concrete swerve implementation |
| `ModuleConfig` | Per-module hardware config (CAN IDs, offsets, position) |
| `PIDGains` | PID + feedforward gains record |
| `DrivetrainIO` | IO layer interface for AdvantageKit replay |
| `DrivetrainIOTalonFX` | Real hardware IO implementation |
| `DrivetrainIOReplay` | No-op IO for log replay |
| `DriveTelemetry` | Dashboard callback interface |
| `DriveState` | Immutable telemetry snapshot record |
| `CameraConfig` | Per-camera config (name + robot-to-camera transform) |

Consumers depend on `DriveInterface`, never on `SwerveDrive` directly.

### Robot Code

| Class | Purpose |
|-------|---------|
| `Robot.java` | LoggedRobot lifecycle, AdvantageKit logging setup, creates RobotContainer |
| `RobotContainer.java` | Subsystem init, controller bindings, auto chooser, default drive command |
| `Robots.java` | Static `DrivetrainConfig` instances per robot |
| `Constants.java` | Controller port constants |
| `DriverPreferences.java` | Live-tunable driver parameters via NetworkTables Preferences |
| `DriverDashboard.java` | SmartDashboard telemetry publisher (implements `DriveTelemetry`) |
| `InputProcessing.java` | Joystick input curve + magnitude clamping utilities |
| `Autos.java` | Autonomous routine factory methods (PathPlanner paths) |
| `Fields.java` | AprilTag field layout constants (competition + training) |

### Command-Based Framework

All robot behavior uses WPILib's command-based paradigm. `CommandScheduler.getInstance().run()` is called in `robotPeriodic()`. The drivetrain exposes command factory methods (`driveFieldCentric`, `brake`, `stop`) rather than standalone Command subclasses.

## AdvantageKit Logging

The robot extends `LoggedRobot` (AdvantageKit) instead of `TimedRobot`. Logging is configured in `Robot()`:

- **Real robot:** `WPILOGWriter` (log to disk) + `NT4Publisher` (NetworkTables)
- **Simulation:** `NT4Publisher` only
- **Build metadata:** Git SHA and build date recorded via `BuildConstants`
- **IO layer:** `DrivetrainIO` interface enables log replay — `DrivetrainIOTalonFX` reads real hardware, `DrivetrainIOReplay` provides no-op inputs for replaying logs
- **Key prefix:** All drivetrain telemetry logged under `Drive/`

## Vision Integration

PhotonVision AprilTag cameras provide pose estimation for field-relative localization.

- **Camera config:** Added via `DrivetrainConfig.camera(name, robotToCamera)` builder method
- **Pose fusion:** `SwerveDrive.periodic()` automatically fuses vision estimates into the pose estimator
- **Standard deviations:** Scale with distance — farther tags produce less confident estimates
- **Rejection:** Estimates are rejected on high ambiguity or poses outside field bounds
- **Field layouts:** `Fields.COMPETITION` loads the official 2026 field; `Fields.training()` lazily loads a custom layout from `deploy/fields/training-field.json`

## Autonomous

Autonomous routines are defined in `Autos.java` as static factory methods using PathPlanner path following. Available routines:

- **Drive Forward** — drives 2m forward (default)
- **Forward Turn Back** — drives 2m forward, turns 180°, drives 1m back
- **Do Nothing** — no-op

Selected via `SendableChooser` on SmartDashboard ("Auto Chooser").

## Robot Configurations

**CORAL** (active) - Current development robot with calibrated encoder offsets and tuned PID.

**FLUX_2026** (placeholder) - Commented out in `Robots.java`, awaiting encoder calibration via Phoenix Tuner X.

To switch robots, change the config passed to `RobotContainer` in `Robot.java`:

```java
m_robotContainer = new RobotContainer(Robots.CORAL, Fields.COMPETITION);
```

## Current Limits and Safety

| Parameter | Limit |
|-----------|-------|
| Drive stator current | 40A continuous |
| Drive supply current | 35A |
| Steer stator current | 20A |
| Slip current threshold | 120A |

Current limits are always configured. Never deploy without them.

## Project Structure

```
src/main/java/
  frc/
    lib/drivetrain/             # Reusable drivetrain library
      CameraConfig.java         # Per-camera config record
      DriveInterface.java       # Consumer API
      DriveState.java           # Immutable telemetry snapshot
      DriveTelemetry.java       # Dashboard callback interface
      DrivetrainConfig.java     # Unified config with builder
      DrivetrainIO.java         # IO layer interface (AdvantageKit)
      DrivetrainIOReplay.java    # No-op IO for log replay
      DrivetrainIOTalonFX.java  # Real hardware IO
      ModuleConfig.java         # Per-module hardware config
      PIDGains.java             # PID + FF gains record
      SwerveDrive.java          # Swerve implementation
    robot/                      # Robot-specific code
      Autos.java                # Autonomous routine factories
      Constants.java            # Controller port constants
      DriverDashboard.java      # SmartDashboard telemetry
      DriverPreferences.java    # Live-tunable driver params
      Fields.java               # AprilTag field layouts
      InputProcessing.java      # Joystick input processing
      Robot.java                # LoggedRobot entry point
      RobotContainer.java       # Subsystems + bindings
      Robots.java               # Per-robot configs
      Main.java                 # WPILib bootstrap (do not modify)
src/test/java/                  # JUnit 5 tests
  frc/
    lib/drivetrain/
      DrivetrainConfigTest.java
      ModuleConfigTest.java
      PIDGainsTest.java
      SwerveDriveTest.java
    robot/
      InputProcessingTest.java
```

## Testing

Tests use JUnit 5 with WPILib's HAL simulation. Run with `./gradlew test`.

## Coordinate Convention

WPILib NWU: forward = +X, left = +Y, counter-clockwise = positive rotation. Joystick Y axis is inverted (negated in code).
