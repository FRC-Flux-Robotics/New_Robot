# FRC Team 10413 FLUX Robotics - 2026 Season

Swerve drivetrain robot code for the 2026 FRC season. Built with WPILib 2026.2.1 and CTRE Phoenix 6.

## Hardware

- **Drive:** 4-module swerve (field-centric, open-loop voltage)
- **Motors:** TalonFX (Kraken/Falcon) for drive and steer
- **Sensors:** CANcoders (absolute steering), Pigeon 2 IMU (heading)
- **CAN bus:** CANivore ("CANdace")
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
| X button | Brake (lock wheels in X pattern) |
| Right bumper | Reset field-centric heading |

Joystick inputs use a squared curve for fine control at low speeds, with slew rate limiting (6.0 translation, 8.0 rotation) for smooth acceleration.

## Architecture

### Config-Driven Drivetrain Library

The drivetrain lives in `frc.lib.drivetrain` as a reusable library. A single `DrivetrainConfig` object (built via builder pattern) captures all hardware IDs, gearing, PID tuning, current limits, and mechanical dimensions. Different robots just need different configs.

Key classes:

| Class | Package | Purpose |
|-------|---------|---------|
| `DrivetrainConfig` | `frc.lib.drivetrain` | All drivetrain parameters in one object |
| `DriveInterface` | `frc.lib.drivetrain` | Clean API for consumers (teleop, auto, subsystems) |
| `SwerveDrive` | `frc.lib.drivetrain` | Concrete swerve implementation |
| `ModuleConfig` | `frc.lib.drivetrain` | Per-module hardware config (CAN IDs, offsets, position) |
| `PIDGains` | `frc.lib.drivetrain` | PID + feedforward gains |
| `Robots` | `frc.robot` | Static config instances per robot |

Consumers depend on `DriveInterface`, never on `SwerveDrive` directly.

### Robot Code

| Class | Purpose |
|-------|---------|
| `Robot.java` | TimedRobot lifecycle, creates RobotContainer with active config |
| `RobotContainer.java` | Subsystem init, controller bindings, default drive command |
| `Constants.java` | Robot-wide constants (controller ports) |

### Command-Based Framework

All robot behavior uses WPILib's command-based paradigm. `CommandScheduler.getInstance().run()` is called in `robotPeriodic()`. The drivetrain exposes command factory methods (`driveFieldCentric`, `brake`, `stop`) rather than standalone Command subclasses.

## Robot Configurations

**CORAL** (active) - Current development robot with calibrated encoder offsets and tuned PID.

**FLUX_2026** (placeholder) - Commented out in `Robots.java`, awaiting encoder calibration via Phoenix Tuner X.

To switch robots, change the config passed to `RobotContainer` in `Robot.java`:

```java
m_robotContainer = new RobotContainer(Robots.CORAL);
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
    lib/drivetrain/         # Reusable drivetrain library
      DriveInterface.java   # Consumer API
      DrivetrainConfig.java # Unified config with builder
      SwerveDrive.java      # Swerve implementation
      ModuleConfig.java     # Per-module hardware config
      PIDGains.java         # PID + FF gains record
    robot/                  # Robot-specific code
      Robot.java            # TimedRobot entry point
      RobotContainer.java   # Subsystems + bindings
      Robots.java           # Per-robot configs
      Constants.java        # Global constants
      Main.java             # WPILib bootstrap (do not modify)
src/test/java/              # JUnit 5 tests
```

## Testing

Tests use JUnit 5 with WPILib's HAL simulation. Run with `./gradlew test`.

## Coordinate Convention

WPILib NWU: forward = +X, left = +Y, counter-clockwise = positive rotation. Joystick Y axis is inverted (negated in code).
