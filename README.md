# FRC Team 10413 FLUX Robotics — 2026 Season

Swerve drivetrain codebase for the 2026 REBUILT game. Java 17, WPILib 2026 command-based framework, CTRE Phoenix 6 hardware.

## Robots

Two robot configurations, selected at runtime via RoboRIO comments string (default: FUEL):

| Robot | CAN Bus | Pigeon ID | Track | Cameras |
|-------|---------|-----------|-------|---------|
| **FUEL** | Drivetrain | 20 | 22.25" x 22.25" | 3 (center, left, right) |
| **CORAL** | CANdace | 24 | 23.5" x 23.5" | 1 (front) |

Set RoboRIO comments to `CORAL` to use CORAL config. Anything else defaults to FUEL.

## Hardware

- **Motors**: Kraken X60 (TalonFX) — 4 drive + 4 steer
- **Encoders**: CANcoder per module
- **IMU**: Pigeon 2
- **Vision**: OV9281 cameras via PhotonVision
- **Current limits**: 60A stator / 120A supply (drive), 60A stator (steer)

## Features

- Field-centric swerve drive with operator perspective switching
- Multi-camera PhotonVision pose estimation with AdvantageKit log replay
- Vision rejection filtering (ambiguity, field bounds, z-error, speed)
- Dynamic vision std dev scaling by tag count and distance
- PathPlanner autonomous with pathfinding and alliance flipping
- SysId characterization (translation, steer, rotation)
- Wheel radius characterization
- Piecewise joystick sensitivity curves (SmartDashboard-tunable)
- Snap-to-angle heading control (D-pad)
- Slow mode (right trigger)
- X-pattern brake
- 86 unit tests

## Project Structure

```
src/main/java/frc/
├── lib/
│   ├── drivetrain/       # Reusable drivetrain library
│   │   ├── DriveInterface        # Consumer API (abstract)
│   │   ├── SwerveDrive           # CTRE implementation
│   │   ├── DrivetrainConfig      # Builder-pattern config
│   │   ├── DrivetrainIO          # Hardware abstraction (AdvantageKit)
│   │   └── ...
│   ├── vision/           # Reusable vision library
│   │   ├── VisionIO              # Hardware abstraction (AdvantageKit)
│   │   ├── VisionIOPhotonVision  # Real camera IO
│   │   └── VisionIOReplay        # Log replay IO
│   └── util/
│       └── PhoenixUtil           # CAN retry helper
└── robot/
    ├── Robot                     # Entry point, mode/robot selection
    ├── RobotContainer            # Subsystems, controls, autos
    ├── Robots                    # FUEL + CORAL hardware configs
    ├── Vision                    # Pose estimation subsystem
    ├── DriveToTag                # AprilTag approach command
    ├── Autos                     # Autonomous routines
    ├── FieldPositions            # Alliance-aware field coordinates
    ├── DriverPreferences         # Persistent driver settings
    ├── PiecewiseSensitivity      # Joystick transfer function
    └── InputProcessing           # Stick clamping
```

## Architecture

Consumers depend on `DriveInterface`, never on `SwerveDrive` directly. Both drivetrain and vision use AdvantageKit IO layers for deterministic log replay:

```
DriveInterface ← SwerveDrive ← DrivetrainIO (TalonFX | Replay)
Vision         ← VisionIO (PhotonVision | Replay)
```

See [docs/](docs/) for detailed documentation.
