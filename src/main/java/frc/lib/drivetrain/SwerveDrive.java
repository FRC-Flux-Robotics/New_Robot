package frc.lib.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;

/**
 * Config-driven swerve drivetrain subsystem. Constructed solely from a {@link DrivetrainConfig}.
 * Implements {@link DriveInterface} for clean API access by other subsystems.
 */
public class SwerveDrive extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        implements Subsystem, DriveInterface {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private static final double kMovingThresholdMps = 0.02;
    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

    // Diagnostic thresholds
    private static final double ODOMETRY_MIN_HZ = 50.0;
    private static final double MOTOR_TEMP_WARN_C = 80.0;
    private static final double MOTOR_TEMP_ERROR_C = 100.0;
    private static final double ALIGNMENT_ERROR_WARN_DEG = 30.0;
    private static final double CURRENT_WARN_FRACTION = 0.9;
    private static final double DIAGNOSTIC_COOLDOWN_SEC = 2.0;

    private final DrivetrainConfig config;

    // Reusable swerve requests (never allocate in loops)
    private final SwerveRequest.FieldCentric fieldCentricRequest;
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    // Operator perspective
    private static final Rotation2d kBlueAlliancePerspective = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspective = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

    // Diagnostics
    private double lastDiagnosticTimeSec = 0;

    // Simulation
    private Notifier simNotifier = null;
    private double lastSimTime;

    public SwerveDrive(DrivetrainConfig config) {
        super(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                config.toSwerveDrivetrainConstants(),
                config.createModuleConstants(config.frontLeft),
                config.createModuleConstants(config.frontRight),
                config.createModuleConstants(config.backLeft),
                config.createModuleConstants(config.backRight));

        this.config = config;

        fieldCentricRequest = new SwerveRequest.FieldCentric()
                .withDeadband(config.maxSpeedMps * config.translationDeadband)
                .withRotationalDeadband(config.maxAngularRateRadPerSec * config.rotationDeadband)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // --- DriveInterface: Movement commands ---

    @Override
    public Command driveFieldCentric(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
        return run(() -> setControl(
                fieldCentricRequest
                        .withVelocityX(vx.getAsDouble())
                        .withVelocityY(vy.getAsDouble())
                        .withRotationalRate(omega.getAsDouble())));
    }

    @Override
    public Command driveRobotCentric(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
        return run(() -> setControl(
                robotCentricRequest
                        .withVelocityX(vx.getAsDouble())
                        .withVelocityY(vy.getAsDouble())
                        .withRotationalRate(omega.getAsDouble())));
    }

    @Override
    public Command brake() {
        return run(() -> setControl(brakeRequest));
    }

    @Override
    public Command stop() {
        return run(() -> setControl(idleRequest));
    }

    // --- DriveInterface: Point-to-point ---

    @Override
    public Command driveToPose(Pose2d target, double toleranceMeters) {
        // PID controllers for X, Y (meters), and rotation (radians)
        ProfiledPIDController xController =
                new ProfiledPIDController(5.0, 0, 0, new TrapezoidProfile.Constraints(2.0, 2.0));
        ProfiledPIDController yController =
                new ProfiledPIDController(5.0, 0, 0, new TrapezoidProfile.Constraints(2.0, 2.0));
        ProfiledPIDController rotController =
                new ProfiledPIDController(5.0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(toleranceMeters);
        yController.setTolerance(toleranceMeters);
        rotController.setTolerance(Math.toRadians(2.0));

        return run(() -> {
                    Pose2d current = getPose();
                    double vx = xController.calculate(current.getX(), target.getX());
                    double vy = yController.calculate(current.getY(), target.getY());
                    double omega = rotController.calculate(
                            current.getRotation().getRadians(), target.getRotation().getRadians());
                    setControl(fieldCentricRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
                })
                .until(() -> xController.atGoal() && yController.atGoal() && rotController.atGoal());
    }

    // --- DriveInterface: Path planning (stubs for Sprint 1) ---

    @Override
    public Command pathfindToPose(Pose2d target) {
        System.out.println("[SwerveDrive] pathfindToPose is a stub — add PathPlanner in Sprint 1");
        return Commands.none();
    }

    @Override
    public Command followPath(Object path) {
        System.out.println("[SwerveDrive] followPath is a stub — add PathPlanner in Sprint 1");
        return Commands.none();
    }

    // --- DriveInterface: State queries ---

    @Override
    public Pose2d getPose() {
        return getState().Pose;
    }

    @Override
    public ChassisSpeeds getVelocity() {
        return getState().Speeds;
    }

    @Override
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    @Override
    public boolean isMoving() {
        ChassisSpeeds speeds = getVelocity();
        double linearSpeed =
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        return linearSpeed > kMovingThresholdMps;
    }

    // --- DriveInterface: Pose management ---

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public void resetHeading() {
        seedFieldCentric();
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        super.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public DrivetrainConfig getConfig() {
        return config;
    }

    // --- Subsystem ---

    @Override
    public void periodic() {
        // Apply operator perspective based on alliance color
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspective
                                : kBlueAlliancePerspective);
                hasAppliedOperatorPerspective = true;
            });
        }

        // --- Telemetry ---
        SwerveDriveState state = getState();
        Pose2d pose = state.Pose;
        ChassisSpeeds speeds = state.Speeds;

        // Robot-level
        SmartDashboard.putNumber("Drive/PositionX", pose.getX());
        SmartDashboard.putNumber("Drive/PositionY", pose.getY());
        SmartDashboard.putNumber("Drive/RotationDeg", pose.getRotation().getDegrees());

        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drive/SpeedMps", linearSpeed);
        SmartDashboard.putNumber("Drive/SpeedPercent", linearSpeed / config.maxSpeedMps * 100.0);
        SmartDashboard.putNumber("Drive/AngularRateDegPerSec",
                Math.toDegrees(speeds.omegaRadiansPerSecond));
        SmartDashboard.putNumber("Drive/OdometryHz", 1.0 / state.OdometryPeriod);

        // Active command
        Command active = getCurrentCommand();
        SmartDashboard.putString("Drive/ActiveCommand", active != null ? active.getName() : "none");

        // Per-module
        SwerveModuleState[] moduleStates = state.ModuleStates;
        SwerveModuleState[] moduleTargets = state.ModuleTargets;
        for (int i = 0; i < 4; i++) {
            String prefix = "Drive/" + MODULE_NAMES[i] + "/";

            // Target vs actual angles and speeds
            double targetAngle = moduleTargets[i].angle.getDegrees();
            double actualAngle = moduleStates[i].angle.getDegrees();
            SmartDashboard.putNumber(prefix + "TargetAngleDeg", targetAngle);
            SmartDashboard.putNumber(prefix + "ActualAngleDeg", actualAngle);
            SmartDashboard.putNumber(prefix + "AngleErrorDeg", targetAngle - actualAngle);
            SmartDashboard.putNumber(prefix + "TargetSpeedMps", moduleTargets[i].speedMetersPerSecond);
            SmartDashboard.putNumber(prefix + "ActualSpeedMps", moduleStates[i].speedMetersPerSecond);

            // Motor temperatures and current
            SwerveModule<TalonFX, TalonFX, CANcoder> module = getModule(i);
            SmartDashboard.putNumber(prefix + "DriveTempC",
                    module.getDriveMotor().getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber(prefix + "SteerTempC",
                    module.getSteerMotor().getDeviceTemp().getValueAsDouble());
            SmartDashboard.putNumber(prefix + "DriveCurrentA",
                    module.getDriveMotor().getStatorCurrent().getValueAsDouble());
        }

        checkDiagnostics(state);
    }

    private void checkDiagnostics(SwerveDriveState state) {
        double now = Timer.getFPGATimestamp();
        boolean cooldownExpired = (now - lastDiagnosticTimeSec) >= DIAGNOSTIC_COOLDOWN_SEC;
        boolean warned = false;

        // Stale odometry (CAN timeout indicator)
        double odometryHz = 1.0 / state.OdometryPeriod;
        boolean odometryStale = odometryHz < ODOMETRY_MIN_HZ;
        SmartDashboard.putBoolean("Drive/Diagnostics/OdometryStale", odometryStale);
        if (odometryStale && cooldownExpired) {
            DriverStation.reportWarning(
                    String.format("Drive: Stale odometry (%.0f Hz) - possible CAN timeout", odometryHz), false);
            warned = true;
        }

        // Per-module diagnostics
        SwerveModuleState[] moduleStates = state.ModuleStates;
        SwerveModuleState[] moduleTargets = state.ModuleTargets;
        for (int i = 0; i < 4; i++) {
            String name = MODULE_NAMES[i];
            String diagPrefix = "Drive/Diagnostics/" + name + "/";
            SwerveModule<TalonFX, TalonFX, CANcoder> module = getModule(i);

            // Motor temperatures
            double driveTemp = module.getDriveMotor().getDeviceTemp().getValueAsDouble();
            double steerTemp = module.getSteerMotor().getDeviceTemp().getValueAsDouble();
            boolean driveTempWarn = driveTemp > MOTOR_TEMP_WARN_C;
            boolean steerTempWarn = steerTemp > MOTOR_TEMP_WARN_C;
            SmartDashboard.putBoolean(diagPrefix + "DriveTempWarn", driveTempWarn);
            SmartDashboard.putBoolean(diagPrefix + "SteerTempWarn", steerTempWarn);

            if (cooldownExpired) {
                if (driveTemp > MOTOR_TEMP_ERROR_C) {
                    DriverStation.reportError(
                            String.format("Drive: %s drive motor OVERTEMP (%.0fC)!", name, driveTemp), false);
                    warned = true;
                } else if (driveTempWarn) {
                    DriverStation.reportWarning(
                            String.format("Drive: %s drive motor hot (%.0fC)", name, driveTemp), false);
                    warned = true;
                }
                if (steerTemp > MOTOR_TEMP_ERROR_C) {
                    DriverStation.reportError(
                            String.format("Drive: %s steer motor OVERTEMP (%.0fC)!", name, steerTemp), false);
                    warned = true;
                } else if (steerTempWarn) {
                    DriverStation.reportWarning(
                            String.format("Drive: %s steer motor hot (%.0fC)", name, steerTemp), false);
                    warned = true;
                }
            }

            // Module alignment error
            double angleError = Math.abs(
                    moduleTargets[i].angle.getDegrees() - moduleStates[i].angle.getDegrees());
            boolean alignmentWarn = angleError > ALIGNMENT_ERROR_WARN_DEG;
            SmartDashboard.putBoolean(diagPrefix + "AlignmentWarn", alignmentWarn);
            if (alignmentWarn && cooldownExpired) {
                DriverStation.reportWarning(
                        String.format("Drive: %s module misaligned (%.0f deg error)", name, angleError), false);
                warned = true;
            }

            // Drive current near limit
            double driveCurrent = module.getDriveMotor().getStatorCurrent().getValueAsDouble();
            boolean currentWarn = driveCurrent > config.driveStatorCurrentLimit * CURRENT_WARN_FRACTION;
            SmartDashboard.putBoolean(diagPrefix + "CurrentWarn", currentWarn);
            if (currentWarn && cooldownExpired) {
                DriverStation.reportWarning(
                        String.format("Drive: %s drive current high (%.0fA/%.0fA limit)",
                                name, driveCurrent, config.driveStatorCurrentLimit),
                        false);
                warned = true;
            }
        }

        if (warned) {
            lastDiagnosticTimeSec = now;
        }
    }

    // --- Simulation ---

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Returns a command that applies the specified swerve request.
     * Convenience method for direct SwerveRequest usage (e.g., in RobotContainer bindings).
     */
    public Command applyRequest(java.util.function.Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }
}
