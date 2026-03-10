# FLUX Swerve Library

Config-driven swerve drivetrain library for FRC robots using CTRE Phoenix 6 hardware (TalonFX, CANcoder, Pigeon 2) with WPILib 2026.

## Quick Start

Define your robot's hardware in a `DrivetrainConfig`, pass it to `SwerveDrive`, and drive:

```java
// 1. Define config (Robots.java)
public static final DrivetrainConfig MY_ROBOT = DrivetrainConfig.builder()
    .canBus("rio")
    .pigeonId(20)
    .frontLeft(new ModuleConfig(2, 1, 21, 0.125, 0.29, 0.29, false, false, false))
    .frontRight(new ModuleConfig(4, 3, 22, -0.25, 0.29, -0.29, true, false, false))
    .backLeft(new ModuleConfig(6, 5, 23, 0.05, -0.29, 0.29, false, false, false))
    .backRight(new ModuleConfig(8, 7, 24, -0.37, -0.29, -0.29, true, false, false))
    .gearing(6.39, 12.1, 4.5, 0.0508)
    .speed(4.99, 0.75 * 2 * Math.PI)
    .steerPID(new PIDGains(100, 0, 0.5, 0.1, 1.5, 0))
    .drivePID(new PIDGains(0.1, 0, 0, 0, 0.124, 0))
    .currentLimits(40, 35, 20, 120)
    .deadband(0.05, 0.1)
    .build();

// 2. Create drivetrain (RobotContainer.java)
DrivetrainConfig config = Robots.MY_ROBOT;
SwerveDrive drivetrain = new SwerveDrive(config);

// 3. Bind joystick to drive command
drivetrain.setDefaultCommand(
    drivetrain.driveFieldCentric(
        () -> -joystick.getLeftY() * config.maxSpeedMps,
        () -> -joystick.getLeftX() * config.maxSpeedMps,
        () -> -joystick.getRightX() * config.maxAngularRateRadPerSec));
```

## Architecture

Consumers depend on `DriveInterface`, never on `SwerveDrive` directly:

```
DriveInterface  <-- your subsystems, auto routines
     ^
     |
SwerveDrive  <-- concrete implementation
     |
DrivetrainConfig  <-- all hardware + tuning in one place
```

## DriveInterface Methods

### Teleop Driving

| Method | Description |
|--------|-------------|
| `driveFieldCentric(vx, vy, omega)` | Field-centric driving (m/s, rad/s) |
| `driveRobotCentric(vx, vy, omega)` | Robot-centric driving (m/s, rad/s) |
| `driveFieldCentricFacingPoint(vx, vy, target)` | Drive while auto-rotating to face a point |
| `brake()` | Lock wheels in X pattern |
| `stop()` | Zero all motor outputs |

### Autonomous Movement

| Method | Description |
|--------|-------------|
| `driveToPose(target, toleranceM)` | PID straight-line drive to pose |
| `pathfindToPose(target)` | PathPlanner pathfinding with obstacle avoidance |
| `followPath(path)` | Follow a pre-built PathPlannerPath |

### State Queries

| Method | Description |
|--------|-------------|
| `getPose()` | Current field pose (meters, radians) |
| `getVelocity()` | Current ChassisSpeeds |
| `getHeading()` | Current gyro heading |
| `isMoving()` | True if above velocity threshold |
| `distanceTo(point)` | Euclidean distance to field point (m) |
| `angleTo(point)` | Angle to field point |
| `isNear(point, toleranceM)` | Proximity check |
| `isAimedAt(point, toleranceDeg)` | Heading alignment check |
| `getFieldRelativeVelocity()` | Field-frame velocity (vx, vy) |

### Pose Management

| Method | Description |
|--------|-------------|
| `resetPose(pose)` | Reset pose estimate |
| `resetHeading()` | Zero heading (current facing = forward) |
| `addVisionMeasurement(pose, timestamp)` | Fuse vision into pose estimator |
| `getConfig()` | Access active DrivetrainConfig |

## DrivetrainConfig Builder

### Required Parameters

| Method | Description |
|--------|-------------|
| `.canBus(name)` | CAN bus name (`"rio"` or CANivore name) |
| `.pigeonId(id)` | Pigeon 2 IMU CAN ID (0-62) |
| `.frontLeft(module)` | Front-left ModuleConfig |
| `.frontRight(module)` | Front-right ModuleConfig |
| `.backLeft(module)` | Back-left ModuleConfig |
| `.backRight(module)` | Back-right ModuleConfig |
| `.gearing(drive, steer, coupling, wheelRadius)` | Gear ratios and wheel radius (meters) |
| `.speed(maxMps, maxRadPerSec)` | Max translational and angular speed |
| `.steerPID(gains)` | Steer motor PID gains |
| `.drivePID(gains)` | Drive motor PID gains |
| `.currentLimits(driveStator, driveSupply, steerStator, slip)` | Current limits (amps) |

### Optional Parameters

| Method | Default | Description |
|--------|---------|-------------|
| `.deadband(translation, rotation)` | 0, 0 | Input deadband (0.0-1.0 fraction of max) |
| `.mass(kg, moiKgM2)` | 74.0, 6.0 | Robot mass and moment of inertia (PathPlanner) |
| `.simSteerPID(gains)` | uses steerPID | Simulation-only steer gains |
| `.simDrivePID(gains)` | uses drivePID | Simulation-only drive gains |
| `.camera(name, robotToCamera)` | none | Add a vision camera |
| `.autoDrive(config)` | defaults | AutoDriveConfig for autonomous PID tuning |
| `.visionConfig(config)` | defaults | VisionConfig for pose filtering |
| `.diagnosticsConfig(config)` | defaults | DiagnosticsConfig for safety thresholds |

## ModuleConfig

Constructor: `ModuleConfig(driveMotorId, steerMotorId, encoderId, encoderOffsetRotations, xPositionMeters, yPositionMeters, invertDrive, invertSteer, invertEncoder)`

- CAN IDs: 0-62
- Encoder offset: -1.0 to 1.0 rotations (measure with Phoenix Tuner X)
- Position: meters from robot center (+X = forward, +Y = left, WPILib NWU convention)

## PIDGains

Constructor: `PIDGains(kP, kI, kD, kS, kV, kA)`

Converts to CTRE `Slot0Configs` via `.toSlot0Configs()`. Get initial values from Phoenix Tuner X.

## Sub-Configs

All sub-configs use builders with sane defaults. Only override what you need.

### AutoDriveConfig

Controls autonomous path following and drive-to-pose behavior.

```java
AutoDriveConfig.builder()
    .headingKP(5.0)                    // heading correction P gain
    .autoTranslationPID(5.0, 0, 0)    // PathPlanner translation PID
    .autoRotationPID(5.0, 0, 0)       // PathPlanner rotation PID
    .driveToPoseMaxVel(2.0)            // m/s max for driveToPose
    .driveToPoseMaxAccel(2.0)          // m/s^2 max for driveToPose
    .pathMaxAccel(2.5)                 // m/s^2 max for path following
    .build();
```

### VisionConfig

Controls vision pose estimation filtering.

```java
VisionConfig.builder()
    .maxAmbiguity(0.2)                 // reject ambiguous detections
    .maxSingleTagDist(4.0)             // max distance for single-tag (m)
    .zHeightThreshold(0.75)            // reject degenerate Z-height poses (m)
    .linearStdDevCoeff(0.02)           // linear std dev = coeff * dist² / tagCount
    .angularStdDevCoeff(0.06)          // angular std dev = coeff * dist² / tagCount
    .fieldBounds(17.0, 8.7)            // reject out-of-bounds poses
    .build();
```

### DiagnosticsConfig

Controls safety thresholds and brownout protection.

```java
DiagnosticsConfig.builder()
    .motorTempWarn(80.0)               // warning at 80C
    .motorTempError(100.0)             // error at 100C
    .currentWarnFraction(0.9)          // warn at 90% of limit
    .brownout(10.5, 7.0, 0.25)        // startV, minV, minScale
    .build();
```

## Dependencies

- **WPILib** 2026.2.1
- **CTRE Phoenix 6** (TalonFX, CANcoder, Pigeon 2)
- **PathPlannerLib** (path following and pathfinding)
- **PhotonVision** (vision pose estimation)
- **AdvantageKit** (logging annotations)
- **Java 17**

## Gradle Setup

In your `settings.gradle`:
```groovy
include ':lib'
```

In your robot `build.gradle`:
```groovy
dependencies {
    implementation project(':lib')
}
```
