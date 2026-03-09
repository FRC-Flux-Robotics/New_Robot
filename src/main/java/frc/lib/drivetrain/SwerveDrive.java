package frc.lib.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;

/**
 * Config-driven swerve drivetrain subsystem. Constructed solely from a {@link DrivetrainConfig}.
 * Implements {@link DriveInterface} for clean API access by other subsystems.
 */
public class SwerveDrive extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
        implements Subsystem, DriveInterface, AutoCloseable {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private static final double kMovingThresholdMps = 0.02;
    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

    // Brownout protection
    private static final double BROWNOUT_START_V = 10.5;
    private static final double BROWNOUT_MIN_V = 7.0;
    private static final double BROWNOUT_MIN_SCALE = 0.25;

    // Diagnostic thresholds
    private static final double ODOMETRY_MIN_HZ = 50.0;
    private static final double MOTOR_TEMP_WARN_C = 80.0;
    private static final double MOTOR_TEMP_ERROR_C = 100.0;
    private static final double ALIGNMENT_ERROR_WARN_DEG = 30.0;
    private static final double CURRENT_WARN_FRACTION = 0.9;
    private static final double DIAGNOSTIC_COOLDOWN_SEC = 2.0;
    private static final double VISION_MAX_AMBIGUITY = 0.2;
    private static final double VISION_MAX_DIST_M = 4.0;

    // Field bounds (FRC field ~16.54m x 8.21m, with 0.5m margin for robot overhang)
    private static final double FIELD_MAX_X_M = 17.0;
    private static final double FIELD_MAX_Y_M = 8.7;

    enum PoseConfidence { HIGH, MEDIUM, LOW, DEAD_RECKONING }

    private final DrivetrainConfig config;
    private final DrivetrainIO io;
    private final DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();
    private final PathConstraints pathConstraints;

    // Reusable swerve requests (never allocate in loops)
    private final SwerveRequest.FieldCentric fieldCentricRequest;
    private final SwerveRequest.FieldCentric driveToPoseRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric autoRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.FieldCentricFacingAngle facingAngleRequest;
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    // SysId characterization requests
    private final SwerveRequest.SysIdSwerveTranslation sysIdTranslationRequest =
            new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation sysIdRotationRequest =
            new SwerveRequest.SysIdSwerveRotation();

    private final SysIdRoutine sysIdTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,        // default ramp rate (1 V/s)
                    Volts.of(4), // 4V step to prevent brownout
                    null,        // default timeout (10 s)
                    state -> com.ctre.phoenix6.SignalLogger.writeString(
                            "SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(sysIdTranslationRequest.withVolts(output)),
                    null, this));

    private final SysIdRoutine sysIdRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6).per(Second), // rad/s² as "volts/s"
                    Volts.of(Math.PI),                  // rad/s as "volts"
                    null,
                    state -> com.ctre.phoenix6.SignalLogger.writeString(
                            "SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        setControl(sysIdRotationRequest.withRotationalRate(output.in(Volts)));
                        com.ctre.phoenix6.SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null, this));

    // Operator perspective
    private static final Rotation2d kBlueAlliancePerspective = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspective = Rotation2d.k180deg;
    private boolean hasAppliedOperatorPerspective = false;

    // Diagnostics
    private double lastDiagnosticTimeSec = 0;

    // Vision confidence tracking
    private double lastAcceptedVisionTimeSec = 0;
    private int lastMaxTagCount = 0;
    private double lastMinAmbiguity = 1.0;

    // Telemetry
    private DriveTelemetry telemetry;
    private boolean lastAllHealthy = true;
    private String lastStatusMessage = "Ready";

    // Simulation
    private Notifier simNotifier = null;
    private double lastSimTime;

    public SwerveDrive(DrivetrainConfig config) {
        this(config, null, null);
    }

    public SwerveDrive(DrivetrainConfig config, AprilTagFieldLayout fieldLayout) {
        this(config, fieldLayout, null);
    }

    SwerveDrive(DrivetrainConfig config, AprilTagFieldLayout fieldLayout, DrivetrainIO io) {
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
        this.io = (io != null) ? io : new DrivetrainIOTalonFX(
                this::getState, this::getModule, config.driveGearRatio,
                config.cameras, fieldLayout);

        pathConstraints = new PathConstraints(
                config.maxSpeedMps,
                2.5, // conservative linear accel (m/s²) — avoids tipping and wheel slip
                config.maxAngularRateRadPerSec,
                Math.PI); // conservative angular accel (rad/s²)

        fieldCentricRequest = new SwerveRequest.FieldCentric()
                .withDeadband(config.maxSpeedMps * config.translationDeadband)
                .withRotationalDeadband(config.maxAngularRateRadPerSec * config.rotationDeadband)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        facingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
                .withDeadband(config.maxSpeedMps * config.translationDeadband)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        facingAngleRequest.HeadingController.setPID(5.0, 0, 0);
        facingAngleRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getVelocity,
                (speeds, feedforwards) -> {
                    setControl(autoRequest
                            .withVelocityX(speeds.vxMetersPerSecond)
                            .withVelocityY(speeds.vyMetersPerSecond)
                            .withRotationalRate(speeds.omegaRadiansPerSecond));
                },
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0, 0),
                        new PIDConstants(5.0, 0, 0)),
                config.toRobotConfig(),
                () -> DriverStation.getAlliance()
                        .orElse(Alliance.Blue) == Alliance.Red,
                this);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // --- DriveInterface: Movement commands ---

    @Override
    public Command driveFieldCentric(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
        return run(() -> {
            double scale = getVoltageSpeedScale();
            setControl(fieldCentricRequest
                    .withVelocityX(vx.getAsDouble() * scale)
                    .withVelocityY(vy.getAsDouble() * scale)
                    .withRotationalRate(omega.getAsDouble() * scale));
        });
    }

    @Override
    public Command driveRobotCentric(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
        return run(() -> {
            double scale = getVoltageSpeedScale();
            setControl(robotCentricRequest
                    .withVelocityX(vx.getAsDouble() * scale)
                    .withVelocityY(vy.getAsDouble() * scale)
                    .withRotationalRate(omega.getAsDouble() * scale));
        });
    }

    @Override
    public Command driveFieldCentricFacingPoint(
            DoubleSupplier vx, DoubleSupplier vy,
            Supplier<Translation2d> fieldTarget) {
        return run(() -> {
            double scale = getVoltageSpeedScale();
            Translation2d target = fieldTarget.get();
            Pose2d pose = getPose();
            double dx = target.getX() - pose.getX();
            double dy = target.getY() - pose.getY();
            Rotation2d targetAngle = new Rotation2d(dx, dy);

            setControl(facingAngleRequest
                    .withVelocityX(vx.getAsDouble() * scale)
                    .withVelocityY(vy.getAsDouble() * scale)
                    .withTargetDirection(targetAngle));

            Logger.recordOutput("Drive/FacingPoint/TargetAngleDeg", targetAngle.getDegrees());
            Logger.recordOutput("Drive/FacingPoint/HeadingErrorDeg",
                    targetAngle.minus(pose.getRotation()).getDegrees());
        });
    }

    @Override
    public Command brake() {
        return run(() -> setControl(brakeRequest));
    }

    @Override
    public Command stop() {
        return run(() -> setControl(idleRequest));
    }

    // --- SysId characterization ---

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdTranslation.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdTranslation.dynamic(direction);
    }

    public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRotation.quasistatic(direction);
    }

    public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
        return sysIdRotation.dynamic(direction);
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
                    double scale = getVoltageSpeedScale();
                    double vx = xController.calculate(current.getX(), target.getX()) * scale;
                    double vy = yController.calculate(current.getY(), target.getY()) * scale;
                    double omega = rotController.calculate(
                            current.getRotation().getRadians(), target.getRotation().getRadians()) * scale;
                    setControl(driveToPoseRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));

                    // PID telemetry for tuning
                    Logger.recordOutput("Drive/DriveToPose/ErrorX", xController.getPositionError());
                    Logger.recordOutput("Drive/DriveToPose/ErrorY", yController.getPositionError());
                    Logger.recordOutput("Drive/DriveToPose/ErrorRotDeg",
                            Math.toDegrees(rotController.getPositionError()));
                    Logger.recordOutput("Drive/DriveToPose/OutputVx", vx);
                    Logger.recordOutput("Drive/DriveToPose/OutputVy", vy);
                    Logger.recordOutput("Drive/DriveToPose/OutputOmega", omega);
                    Logger.recordOutput("Drive/DriveToPose/AtGoal",
                            xController.atGoal() && yController.atGoal() && rotController.atGoal());
                })
                .until(() -> xController.atGoal() && yController.atGoal() && rotController.atGoal());
    }

    // --- DriveInterface: Path planning (PathPlanner) ---

    @Override
    public Command pathfindToPose(Pose2d target) {
        return AutoBuilder.pathfindToPose(target, pathConstraints);
    }

    @Override
    public Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path);
    }

    // --- DriveInterface: State queries ---

    @Override
    public Pose2d getPose() {
        SwerveDriveState state = getState();
        return state != null ? state.Pose : new Pose2d();
    }

    @Override
    public ChassisSpeeds getVelocity() {
        SwerveDriveState state = getState();
        return state != null ? state.Speeds : new ChassisSpeeds();
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

    /**
     * Returns standard deviations for vision measurement based on tag count and distance,
     * or null if the measurement should be rejected.
     */
    Matrix<N3, N1> getVisionStdDevs(int tagCount, double avgDistM) {
        if (tagCount == 1 && avgDistM > VISION_MAX_DIST_M) {
            return null; // reject single tag far away
        }
        if (tagCount >= 2 && avgDistM <= VISION_MAX_DIST_M) {
            return VecBuilder.fill(0.3, 0.3, 0.5); // multi-tag close: high trust
        }
        // 1 tag close, or 2+ tags far: moderate trust, don't trust rotation
        return VecBuilder.fill(0.5, 0.5, 999);
    }

    PoseConfidence getPoseConfidence(double secSinceVision, int maxTagCount, double minAmbiguity) {
        if (secSinceVision > 5.0) return PoseConfidence.DEAD_RECKONING;
        if (maxTagCount >= 2 && minAmbiguity <= 0.1 && secSinceVision <= 1.0) return PoseConfidence.HIGH;
        if (secSinceVision <= 2.0) return PoseConfidence.MEDIUM;
        return PoseConfidence.LOW;
    }

    boolean isVisionPoseOnField(Pose2d pose) {
        return pose.getX() >= 0 && pose.getX() <= FIELD_MAX_X_M
            && pose.getY() >= 0 && pose.getY() <= FIELD_MAX_Y_M;
    }

    @Override
    public DrivetrainConfig getConfig() {
        return config;
    }

    /** Set the telemetry consumer to receive DriveState snapshots each cycle. */
    public void setTelemetry(DriveTelemetry telemetry) {
        this.telemetry = telemetry;
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

        // --- IO inputs (logged for replay) ---
        io.updateInputs(inputs);
        Logger.processInputs("Drive", inputs);

        // --- Computed outputs ---
        SwerveDriveState state = getState();
        if (state == null) return;
        Pose2d pose = state.Pose;
        ChassisSpeeds speeds = state.Speeds;

        // Struct logging for complex types
        Logger.recordOutput("Drive/Pose", pose);
        Logger.recordOutput("Drive/Speeds", speeds);

        // Robot-level computed outputs
        Logger.recordOutput("Drive/PositionX", pose.getX());
        Logger.recordOutput("Drive/PositionY", pose.getY());
        Logger.recordOutput("Drive/RotationDeg", pose.getRotation().getDegrees());

        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Logger.recordOutput("Drive/SpeedMps", linearSpeed);
        Logger.recordOutput("Drive/SpeedPercent", linearSpeed / config.maxSpeedMps * 100.0);
        Logger.recordOutput("Drive/AngularRateDegPerSec",
                Math.toDegrees(speeds.omegaRadiansPerSecond));
        Logger.recordOutput("Drive/OdometryHz",
                inputs.odometryPeriodSec > 0 ? 1.0 / inputs.odometryPeriodSec : 0);

        // Brownout protection
        double brownoutScale = getVoltageSpeedScale();
        Logger.recordOutput("Drive/BatteryVoltage", inputs.batteryVoltage);
        Logger.recordOutput("Drive/BrownoutSpeedScale", brownoutScale);
        Logger.recordOutput("Drive/BrownoutActive", brownoutScale < 1.0);

        // Active command
        Command active = getCurrentCommand();
        Logger.recordOutput("Drive/ActiveCommand", active != null ? active.getName() : "none");

        // Per-module struct outputs
        SwerveModuleState[] moduleStates = state.ModuleStates;
        SwerveModuleState[] moduleTargets = state.ModuleTargets;
        Logger.recordOutput("Drive/ModuleStates", moduleStates);
        Logger.recordOutput("Drive/ModuleTargets", moduleTargets);

        // Per-module current telemetry
        double totalCurrentA = 0;
        for (int i = 0; i < 4; i++) {
            String prefix = "Drive/" + MODULE_NAMES[i] + "/";
            Logger.recordOutput(prefix + "DriveCurrentA", inputs.driveCurrentA[i]);
            Logger.recordOutput(prefix + "SteerCurrentA", inputs.steerCurrentA[i]);
            totalCurrentA += inputs.driveCurrentA[i] + inputs.steerCurrentA[i];
        }
        Logger.recordOutput("Drive/TotalCurrentA", totalCurrentA);

        // Vision fusion
        int cycleMaxTagCount = 0;
        double cycleMinAmbiguity = 1.0;
        boolean anyAccepted = false;

        for (int i = 0; i < inputs.visionConnected.length; i++) {
            String vPrefix = "Drive/Vision/" + i + "/";
            Logger.recordOutput(vPrefix + "Connected", inputs.visionConnected[i]);
            Logger.recordOutput(vPrefix + "TagCount", inputs.visionTagCount[i]);
            Logger.recordOutput(vPrefix + "Ambiguity", inputs.visionAmbiguity[i]);
            Logger.recordOutput(vPrefix + "AvgTagDistM", inputs.visionAvgTagDistM[i]);

            if (inputs.visionHasEstimate[i]) {
                Pose2d visionPose = new Pose2d(
                        inputs.visionPoseX[i],
                        inputs.visionPoseY[i],
                        Rotation2d.fromDegrees(inputs.visionPoseRotDeg[i]));
                Logger.recordOutput(vPrefix + "EstimatedPose", visionPose);

                double odometryDelta = pose.getTranslation().getDistance(visionPose.getTranslation());
                Logger.recordOutput(vPrefix + "OdometryDeltaM", odometryDelta);
                double latencyMs = (Timer.getFPGATimestamp() - inputs.visionTimestampSec[i]) * 1000.0;
                Logger.recordOutput(vPrefix + "LatencyMs", latencyMs);

                if (!isVisionPoseOnField(visionPose)) {
                    Logger.recordOutput(vPrefix + "Rejected", true);
                    Logger.recordOutput(vPrefix + "RejectReason", "OutOfBounds");
                } else if (inputs.visionAmbiguity[i] > VISION_MAX_AMBIGUITY) {
                    Logger.recordOutput(vPrefix + "Rejected", true);
                    Logger.recordOutput(vPrefix + "RejectReason", "HighAmbiguity");
                } else {
                    Matrix<N3, N1> stdDevs = getVisionStdDevs(
                            inputs.visionTagCount[i], inputs.visionAvgTagDistM[i]);
                    if (stdDevs == null) {
                        Logger.recordOutput(vPrefix + "Rejected", true);
                        Logger.recordOutput(vPrefix + "RejectReason", "SingleTagFar");
                    } else {
                        Logger.recordOutput(vPrefix + "Rejected", false);
                        Logger.recordOutput(vPrefix + "RejectReason", "");
                        super.addVisionMeasurement(visionPose,
                                Utils.fpgaToCurrentTime(inputs.visionTimestampSec[i]), stdDevs);
                        anyAccepted = true;
                        cycleMaxTagCount = Math.max(cycleMaxTagCount, inputs.visionTagCount[i]);
                        cycleMinAmbiguity = Math.min(cycleMinAmbiguity, inputs.visionAmbiguity[i]);
                    }
                }
            }
        }

        if (anyAccepted) {
            lastAcceptedVisionTimeSec = Timer.getFPGATimestamp();
            lastMaxTagCount = cycleMaxTagCount;
            lastMinAmbiguity = cycleMinAmbiguity;
        }

        PoseConfidence confidence = getPoseConfidence(
                Timer.getFPGATimestamp() - lastAcceptedVisionTimeSec,
                lastMaxTagCount, lastMinAmbiguity);
        Logger.recordOutput("Drive/PoseConfidence", confidence.name());

        checkDiagnostics(state);

        // Build and publish DriveState snapshot
        if (telemetry != null) {
            telemetry.update(new DriveState(
                    linearSpeed,
                    linearSpeed / config.maxSpeedMps * 100.0,
                    totalCurrentA,
                    inputs.driveCurrentA,
                    inputs.steerCurrentA,
                    inputs.batteryVoltage,
                    brownoutScale < 1.0,
                    brownoutScale,
                    lastAllHealthy,
                    lastStatusMessage,
                    active != null ? active.getName() : "none"));
        }
    }

    private void checkDiagnostics(SwerveDriveState state) {
        double now = Timer.getFPGATimestamp();
        boolean cooldownExpired = (now - lastDiagnosticTimeSec) >= DIAGNOSTIC_COOLDOWN_SEC;
        boolean warned = false;

        // Brownout protection active
        double brownoutScale = getVoltageSpeedScale();
        Logger.recordOutput("Drive/Diagnostics/BrownoutActive", brownoutScale < 1.0);
        if (brownoutScale < 1.0 && cooldownExpired) {
            DriverStation.reportWarning(
                    String.format("Drive: Brownout protection active (%.1fV, %.0f%% speed)",
                            inputs.batteryVoltage, brownoutScale * 100.0),
                    false);
            warned = true;
        }

        // Stale odometry (CAN timeout indicator)
        double odometryHz = inputs.odometryPeriodSec > 0 ? 1.0 / inputs.odometryPeriodSec : 0;
        boolean odometryStale = odometryHz < ODOMETRY_MIN_HZ;
        Logger.recordOutput("Drive/Diagnostics/OdometryStale", odometryStale);
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

            // Motor temperatures (from IO inputs)
            double driveTemp = inputs.driveTempC[i];
            double steerTemp = inputs.steerTempC[i];
            boolean driveTempWarn = driveTemp > MOTOR_TEMP_WARN_C;
            boolean steerTempWarn = steerTemp > MOTOR_TEMP_WARN_C;
            Logger.recordOutput(diagPrefix + "DriveTempWarn", driveTempWarn);
            Logger.recordOutput(diagPrefix + "SteerTempWarn", steerTempWarn);

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
                    moduleTargets[i].angle.minus(moduleStates[i].angle).getDegrees());
            boolean alignmentWarn = angleError > ALIGNMENT_ERROR_WARN_DEG;
            Logger.recordOutput(diagPrefix + "AlignmentWarn", alignmentWarn);
            if (alignmentWarn && cooldownExpired) {
                DriverStation.reportWarning(
                        String.format("Drive: %s module misaligned (%.0f deg error)", name, angleError), false);
                warned = true;
            }

            // Drive current near limit (from IO inputs)
            double driveCurrent = inputs.driveCurrentA[i];
            boolean currentWarn = driveCurrent > config.driveStatorCurrentLimit * CURRENT_WARN_FRACTION;
            Logger.recordOutput(diagPrefix + "CurrentWarn", currentWarn);
            if (currentWarn && cooldownExpired) {
                DriverStation.reportWarning(
                        String.format("Drive: %s drive current high (%.0fA/%.0fA limit)",
                                name, driveCurrent, config.driveStatorCurrentLimit),
                        false);
                warned = true;
            }

            // Steer current near limit
            double steerCurrent = inputs.steerCurrentA[i];
            boolean steerCurrentWarn = steerCurrent > config.steerStatorCurrentLimit * CURRENT_WARN_FRACTION;
            Logger.recordOutput(diagPrefix + "SteerCurrentWarn", steerCurrentWarn);
            if (steerCurrentWarn && cooldownExpired) {
                DriverStation.reportWarning(
                        String.format("Drive: %s steer current high (%.0fA/%.0fA limit)",
                                name, steerCurrent, config.steerStatorCurrentLimit),
                        false);
                warned = true;
            }
        }

        if (warned) {
            lastDiagnosticTimeSec = now;
        }

        // Health summary for driver dashboard
        lastAllHealthy = !odometryStale && brownoutScale >= 1.0;
        lastStatusMessage = "Ready";

        if (odometryStale) {
            lastAllHealthy = false;
            lastStatusMessage = "CAN issues";
        }

        for (int i = 0; i < 4; i++) {
            if (inputs.driveTempC[i] > MOTOR_TEMP_WARN_C || inputs.steerTempC[i] > MOTOR_TEMP_WARN_C) {
                lastAllHealthy = false;
                lastStatusMessage = MODULE_NAMES[i] + " motor hot";
            }
        }

        // Check vision camera connectivity
        for (int i = 0; i < inputs.visionConnected.length; i++) {
            if (!inputs.visionConnected[i]) {
                lastAllHealthy = false;
                lastStatusMessage = "Camera " + i + " disconnected";
            }
        }

        if (brownoutScale < 1.0) {
            lastStatusMessage = String.format("Low battery (%.1fV)", inputs.batteryVoltage);
        }

        Logger.recordOutput("Drive/AllHealthy", lastAllHealthy);
        Logger.recordOutput("Drive/StatusMessage", lastStatusMessage);
    }

    // --- Brownout protection ---

    double getVoltageSpeedScale() {
        double voltage = inputs.batteryVoltage;
        if (voltage >= BROWNOUT_START_V) return 1.0;
        if (voltage <= BROWNOUT_MIN_V) return BROWNOUT_MIN_SCALE;
        return BROWNOUT_MIN_SCALE + (1.0 - BROWNOUT_MIN_SCALE)
                * (voltage - BROWNOUT_MIN_V) / (BROWNOUT_START_V - BROWNOUT_MIN_V);
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

    @Override
    public void close() {
        if (simNotifier != null) {
            simNotifier.stop();
            simNotifier.close();
            simNotifier = null;
        }
    }

    /**
     * Returns a command that applies the specified swerve request.
     * Convenience method for direct SwerveRequest usage (e.g., in RobotContainer bindings).
     */
    public Command applyRequest(java.util.function.Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }
}
