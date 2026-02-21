# CLAUDE.md - FRC Robot Development Guide

## Project Overview

FRC Team 10413 FLUX Robotics - 2026 season robot project. Java-based swerve drivetrain using WPILib 2026 command-based framework with CTRE Phoenix 6 hardware.

## Repository Structure

```
/home/boris/frc/new/
├── src/main/java/frc/robot/
│   ├── Main.java                    # Entry point (do not modify)
│   ├── Robot.java                   # TimedRobot lifecycle
│   ├── RobotContainer.java          # Subsystem init + controller bindings
│   ├── RobotConfig.java             # Multi-robot hardware configuration
│   ├── Constants.java               # Robot-wide constants
│   ├── SwerveModuleConfig.java      # Per-module hardware config
│   ├── commands/                    # Command classes
│   ├── subsystems/
│   │   └── CommandSwerveDrivetrain.java  # Swerve subsystem
│   └── generated/
│       └── TunerConstants.java      # CTRE Tuner X generated (do not edit manually)
├── src/main/deploy/                 # Files deployed to roboRIO
├── src/test/java/frc/robot/         # Unit tests (JUnit 5)
├── vendordeps/                      # Vendor dependency JSONs
│   ├── Phoenix6-26.1.1.json         # CTRE Phoenix 6
│   └── WPILibNewCommands.json       # Command-based framework
├── build.gradle                     # GradleRIO 2026.1.1
└── .wpilib/                         # WPILib project settings
```

## Essential Commands

```bash
# Build and test (run from this directory)
./gradlew build              # Compile + run tests
./gradlew test               # Run tests only
./gradlew test --tests "TestClassName"  # Specific test

# Deploy to robot (must be on 10.104.13.X network)
./gradlew deploy

# Simulation
./gradlew simulateJava       # Run with simulation GUI
```

## Team & Hardware Info

- **Team:** 10413 FLUX Robotics
- **Season:** 2026, Competition Week 7
- **WPILib:** 2026.1.1 (GradleRIO)
- **Java:** 17
- **Hardware:** TalonFX motors, CANcoders, Pigeon 2 IMU, CANivore bus
- **Drive:** Swerve (4 modules, field-centric, open-loop voltage)

## Architecture Patterns

### Command-Based Framework

This project uses WPILib's command-based paradigm. All robot behavior is expressed through Commands and Subsystems managed by the CommandScheduler.

**Key rules:**
- `CommandScheduler.getInstance().run()` MUST be called in `robotPeriodic()` - already done in Robot.java
- Each subsystem can only have one active command at a time
- Default commands must never finish (use `run()` not `runOnce()`)
- Commands declare subsystem requirements via `addRequirements()`
- Command instances used in compositions cannot be reused independently

### Subsystem Pattern

Subsystems encapsulate hardware. Follow this pattern:

```java
public class MySubsystem extends SubsystemBase {
    // Hardware (private)
    private final TalonFX motor;

    // Constructor: initialize hardware, configure motors
    public MySubsystem() {
        motor = new TalonFX(CAN_ID);
        // Apply configurations
    }

    // Command factory methods (preferred over Command subclasses)
    public Command runCommand(double speed) {
        return this.startEnd(
            () -> motor.setControl(new DutyCycleOut(speed)),
            () -> motor.setControl(new DutyCycleOut(0))
        );
    }

    @Override
    public void periodic() {
        // Telemetry, odometry updates
    }
}
```

### Command Patterns

Prefer command factory methods on subsystems over separate Command subclasses:

```java
// Factory method (preferred for simple commands)
public Command intakeCommand() {
    return this.startEnd(() -> set(1.0), () -> set(0.0));
}

// Subclass (for complex stateful logic)
public class DriveForwardAuto extends Command {
    @Override public void initialize() { /* setup */ }
    @Override public void execute() { /* run each cycle */ }
    @Override public boolean isFinished() { /* termination condition */ }
    @Override public void end(boolean interrupted) { /* cleanup */ }
}
```

### Command Compositions

```java
// Sequential: A then B
commandA.andThen(commandB)

// Parallel (all finish): A and B simultaneously
commandA.alongWith(commandB)

// Race (first finishes): whichever completes first
commandA.raceWith(commandB)

// Deadline: B runs until A finishes
commandA.deadlineWith(commandB)

// Decorators
command.withTimeout(5.0)       // 5 second timeout
command.until(() -> condition)  // stop when condition true
command.repeatedly()            // restart indefinitely
```

### Trigger Bindings (Controller Buttons)

```java
// In RobotContainer.configureBindings():
driverController.a().onTrue(command);           // On press
driverController.b().whileTrue(command);         // While held
driverController.x().toggleOnTrue(command);      // Toggle on press
driverController.rightBumper().onTrue(command);  // Bumper press

// Composed triggers
driverController.leftBumper()
    .and(driverController.rightBumper())
    .onTrue(emergencyStopCommand);               // Both bumpers
```

## CTRE Phoenix 6 API Patterns

### Motor Control

```java
// Configuration (apply once in constructor)
TalonFXConfiguration config = new TalonFXConfiguration();
config.CurrentLimits.StatorCurrentLimit = 40;
config.CurrentLimits.StatorCurrentLimitEnable = true;
config.CurrentLimits.SupplyCurrentLimit = 35;
config.CurrentLimits.SupplyCurrentLimitEnable = true;
motor.getConfigurator().apply(config);

// Control requests (reuse instances, don't allocate in loops)
private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
private final VoltageOut voltage = new VoltageOut(0);
private final PositionVoltage position = new PositionVoltage(0);

motor.setControl(dutyCycle.withOutput(0.5));
motor.setControl(voltage.withOutput(6.0));
motor.setControl(position.withPosition(10.0));

// Reading status signals
double pos = motor.getPosition().getValueAsDouble();
double vel = motor.getVelocity().getValueAsDouble();

// Batch refresh for efficiency
BaseStatusSignal.refreshAll(posSignal, velSignal, voltSignal);
```

### Swerve Drive

The swerve drive uses CTRE's Phoenix 6 swerve API with `TunerSwerveDrivetrain` as the base class.

```java
// SwerveRequests (reuse instances)
private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(maxSpeed * 0.1)
    .withRotationalDeadband(maxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

// Apply via command
drivetrain.applyRequest(() -> drive
    .withVelocityX(-joystickY * maxSpeed)   // Note: joystick Y inverted
    .withVelocityY(-joystickX * maxSpeed)   // Note: joystick X inverted
    .withRotationalRate(-joystickRotation * maxAngularRate));
```

**Coordinate convention:** WPILib uses NWU (forward=+X, left=+Y, CCW=positive). Joystick Y axis is inverted (push forward = negative). Always negate joystick values.

## Current Limits & Safety

```
Drive motors:  40A stator continuous, 35A supply
Steer motors:  20A stator continuous
Slip current:  120A (wheel slip detection threshold)
```

Always configure current limits in `TalonFXConfiguration`. Never deploy without current limits enabled.

## WPILib Units System

Use type-safe units from `edu.wpi.first.units`:

```java
import static edu.wpi.first.units.Units.*;

Distance d = Inches.of(11.5);
LinearVelocity v = MetersPerSecond.of(4.99);
Angle a = Rotations.of(0.5);
Current i = Amps.of(40);

// Convert
double meters = d.in(Meters);
double radians = a.in(Radians);
```

## Testing

- Framework: JUnit 5
- Tests go in `src/test/java/frc/robot/`
- Run: `./gradlew test`
- Reports: `build/reports/tests/test/index.html`
- Initialize HAL in tests: `HAL.initialize(500, 0)` in `@BeforeAll`

### Test Pattern

```java
class MySubsystemTest {
    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    @Test
    void testBehavior() {
        // Arrange
        // Act
        // Assert
    }
}
```

## Vendor Dependencies

Installed vendordeps are in `vendordeps/`. To add new ones:

```bash
# Via command line
./gradlew vendordep --url=<vendor-json-url>

# Common vendordep URLs (check vendor sites for latest):
# CTRE Phoenix 6: https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2026-latest.json
# REVLib: https://software-metadata.revrobotics.com/REVLib-2026.json
# PathPlanner: https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib-2026.json
# PhotonVision: Check https://docs.photonvision.org for latest URL
# AdvantageKit: Check https://docs.advantagekit.org for latest URL
```

## Files to Never Modify

- `Main.java` - WPILib entry point, no static initialization allowed
- `generated/TunerConstants.java` - Auto-generated by CTRE Tuner X. Regenerate via Tuner X if hardware changes.

## Development Rules

1. **Delete first** - Less code = fewer bugs
2. **Test always** - Write tests for new subsystems and commands
3. **Safety first** - Always configure current limits, never deploy without them
4. **Simple wins** - Avoid over-engineering; use command factory methods over subclasses when possible
5. **Reuse control request objects** - Never allocate `SwerveRequest`, `DutyCycleOut`, etc. in periodic loops (GC pressure)
6. **Batch CAN reads** - Use `BaseStatusSignal.refreshAll()` instead of individual signal reads
7. **Validate inputs** - Check CAN IDs (0-62), encoder offsets, current limits
