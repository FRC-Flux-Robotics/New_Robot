Develop FRC robot code: $ARGUMENTS

You are an expert FRC (FIRST Robotics Competition) developer. Help implement the requested feature following WPILib 2026 command-based patterns and this project's conventions.

## Context

This is a Java FRC robot project using:
- **WPILib 2026.1.1** command-based framework
- **CTRE Phoenix 6** (TalonFX motors, CANcoders, Pigeon 2 IMU)
- **Swerve drivetrain** (4 modules, field-centric, open-loop voltage)
- **GradleRIO** build system with JUnit 5 testing

## Steps

1. **Understand the request**: Read $ARGUMENTS carefully. If ambiguous, use AskUserQuestion to clarify.

2. **Read existing code** before writing anything:
   - `src/main/java/frc/robot/RobotContainer.java` - to understand current bindings and subsystems
   - `src/main/java/frc/robot/Constants.java` - for existing constants
   - `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` - drivetrain reference
   - `src/main/java/frc/robot/generated/TunerConstants.java` - hardware constants (DO NOT EDIT)
   - Any other relevant files

3. **Enter plan mode** for non-trivial features. Plan should cover:
   - What files to create or modify
   - Subsystem design (if adding hardware)
   - Command design (factory methods preferred over subclasses)
   - Constants to add
   - Controller bindings
   - Safety considerations (current limits, emergency stop integration)
   - Tests to write

4. **Implement following these patterns:**

### New Subsystem Checklist
- [ ] Create `src/main/java/frc/robot/subsystems/MySubsystem.java`
- [ ] Extend `SubsystemBase`
- [ ] Declare hardware as `private final` fields
- [ ] Configure current limits in constructor
- [ ] Add command factory methods (prefer over Command subclasses)
- [ ] Implement `periodic()` for telemetry
- [ ] Add constants to `Constants.java` in a nested static class
- [ ] Register in `RobotContainer` constructor
- [ ] Add controller bindings in `configureBindings()`
- [ ] Write tests in `src/test/java/frc/robot/`

### New Command Checklist (when subclass is needed)
- [ ] Create in `src/main/java/frc/robot/commands/`
- [ ] Extend `Command`
- [ ] Call `addRequirements()` in constructor
- [ ] Implement `initialize()`, `execute()`, `isFinished()`, `end(boolean interrupted)`
- [ ] Handle interruption cleanup in `end(true)`
- [ ] Write tests

### Motor Configuration Template
```java
TalonFXConfiguration config = new TalonFXConfiguration();

// Current limits (REQUIRED for safety)
config.CurrentLimits.StatorCurrentLimit = 40;  // Adjust per mechanism
config.CurrentLimits.StatorCurrentLimitEnable = true;
config.CurrentLimits.SupplyCurrentLimit = 35;
config.CurrentLimits.SupplyCurrentLimitEnable = true;

// PID gains (tune for your mechanism)
config.Slot0.kP = 0.1;
config.Slot0.kI = 0;
config.Slot0.kD = 0;
config.Slot0.kV = 0.12;

motor.getConfigurator().apply(config);
```

### Control Request Pattern
```java
// Declare as class fields (reuse, never allocate in loops)
private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
private final VoltageOut voltageRequest = new VoltageOut(0);
private final PositionVoltage positionRequest = new PositionVoltage(0);

// Use in commands
motor.setControl(dutyCycleRequest.withOutput(speed));
```

### REV Motor Configuration Template (if using SparkMax/SparkFlex)
```java
CANSparkMax motor = new CANSparkMax(CAN_ID, MotorType.kBrushless);
motor.setSmartCurrentLimit(40);  // Amps
motor.setIdleMode(IdleMode.kBrake);
```

## Safety Rules (NEVER skip these)

1. **Always configure current limits** on every motor controller
2. **Validate CAN IDs** are in 0-62 range (CTRE) or appropriate range (REV)
3. **Never allocate objects in periodic loops** (control requests, status signals)
4. **Use type-safe units** from `edu.wpi.first.units` for physical quantities
5. **Invert joystick axes** - WPILib is NWU (forward=+X, left=+Y), joystick Y is inverted
6. **Handle command interruption** - always clean up in `end(boolean interrupted)`
7. **Test before deploy** - run `./gradlew build` before suggesting deployment

## WPILib Coordinate Convention

- Forward = +X, Left = +Y, Up = +Z
- Counter-clockwise rotation = positive
- Joystick: push forward = negative Y (MUST negate)
- Some gyros (NavX) use CW-positive (MUST negate)

## Common Subsystem Types

### Elevator/Lift
- Use `PositionVoltage` or `MotionMagicVoltage` control
- Add soft limits in `TalonFXConfiguration`
- Consider gravity compensation (kG feedforward)
- Add limit switches for homing

### Intake/Roller
- Use `DutyCycleOut` or `VoltageOut` control
- Lower current limits (20-30A typical)
- Simple start/stop command factory methods

### Arm/Wrist
- Use `PositionVoltage` or `MotionMagicVoltage` control
- Account for gravity with feedforward
- Add soft limits and physical hard stops
- Consider `SingleJointedArmSim` for simulation

### Shooter/Flywheel
- Use `VelocityVoltage` control
- Higher current limits during spinup
- Track velocity for "ready to shoot" state

## After Implementation

5. **Build and test**: Run `./gradlew build` to verify compilation and tests pass
6. **Summarize changes**: List all files created/modified and what they do
