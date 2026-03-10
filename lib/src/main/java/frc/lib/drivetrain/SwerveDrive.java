package frc.lib.drivetrain;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Config-driven swerve drivetrain subsystem. Constructed solely from a {@link DrivetrainConfig}.
 * Implements {@link DriveInterface} for clean API access by other subsystems.
 */
public class SwerveDrive extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem, DriveInterface, AutoCloseable {

  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
  private static final double MOVING_THRESHOLD_MPS = 0.02;
  private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

  enum PoseConfidence {
    HIGH,
    MEDIUM,
    LOW,
    DEAD_RECKONING
  }

  private final DrivetrainConfig config;
  private final DrivetrainIO io;
  private final DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();
  private final PathConstraints pathConstraints;

  // Reusable swerve requests (never allocate in loops)
  private final SwerveRequest.FieldCentric fieldCentricRequest;
  private final SwerveRequest.FieldCentric driveToPoseRequest =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric robotCentricRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.RobotCentric autoRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
  private final SwerveRequest.FieldCentricFacingAngle facingAngleRequest;
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  // SysId characterization requests
  private final SwerveRequest.SysIdSwerveTranslation sysIdTranslationRequest =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SysIdRoutine sysIdTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // default ramp rate (1 V/s)
              Volts.of(4), // 4V step to prevent brownout
              null, // default timeout (10 s)
              state ->
                  com.ctre.phoenix6.SignalLogger.writeString(
                      "SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(sysIdTranslationRequest.withVolts(output)), null, this));

  // Operator perspective
  private static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE = Rotation2d.kZero;
  private static final Rotation2d RED_ALLIANCE_PERSPECTIVE = Rotation2d.k180deg;
  private boolean hasAppliedOperatorPerspective = false;

  // Diagnostics
  private double lastDiagnosticTimeSec = 0;
  private int periodicCycleCount = 0;
  private boolean loggedOdometryHz = false;

  // Vision confidence tracking
  private double lastAcceptedVisionTimeSec = 0;
  private int lastMaxTagCount = 0;
  private double lastMinAmbiguity = 1.0;

  // Telemetry
  private DriveTelemetry telemetry;
  private boolean lastAllHealthy = true;
  private String lastStatusMessage = "Ready";

  // SignalLogger pre-allocated arrays (avoid GC churn)
  private final double[] signalLogPose = new double[3];
  private final double[] signalLogSpeeds = new double[3];
  private final double[] signalLogModuleStates = new double[8];

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
    this.io =
        (io != null)
            ? io
            : new DrivetrainIOTalonFX(
                this::getState,
                this::getModule,
                config.driveGearRatio,
                config.cameras,
                fieldLayout);

    getPigeon2().optimizeBusUtilization();

    pathConstraints =
        new PathConstraints(
            config.maxSpeedMps,
            config.autoDrive.pathMaxAccelMps2,
            config.maxAngularRateRadPerSec,
            config.autoDrive.pathMaxAngularAccelRadPerSec2);

    fieldCentricRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(config.maxSpeedMps * config.translationDeadband)
            .withRotationalDeadband(config.maxAngularRateRadPerSec * config.rotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    facingAngleRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(config.maxSpeedMps * config.translationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    facingAngleRequest.HeadingController.setPID(config.autoDrive.headingKP, 0, 0);
    facingAngleRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getVelocity,
        (speeds, feedforwards) -> {
          setControl(
              autoRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        },
        new PPHolonomicDriveController(
            new PIDConstants(
                config.autoDrive.autoTranslationKP,
                config.autoDrive.autoTranslationKI,
                config.autoDrive.autoTranslationKD),
            new PIDConstants(
                config.autoDrive.autoRotationKP,
                config.autoDrive.autoRotationKI,
                config.autoDrive.autoRotationKD)),
        config.toRobotConfig(),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    SignalLogger.start();

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  // --- DriveInterface: Movement commands ---

  @Override
  public Command driveFieldCentric(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return run(
        () -> {
          double scale = getVoltageSpeedScale();
          setControl(
              fieldCentricRequest
                  .withVelocityX(vx.getAsDouble() * scale)
                  .withVelocityY(vy.getAsDouble() * scale)
                  .withRotationalRate(omega.getAsDouble() * scale));
        });
  }

  @Override
  public Command driveRobotCentric(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
    return run(
        () -> {
          double scale = getVoltageSpeedScale();
          setControl(
              robotCentricRequest
                  .withVelocityX(vx.getAsDouble() * scale)
                  .withVelocityY(vy.getAsDouble() * scale)
                  .withRotationalRate(omega.getAsDouble() * scale));
        });
  }

  @Override
  public Command driveFieldCentricFacingPoint(
      DoubleSupplier vx, DoubleSupplier vy, Supplier<Translation2d> fieldTarget) {
    return run(
        () -> {
          double scale = getVoltageSpeedScale();
          Translation2d target = fieldTarget.get();
          Pose2d pose = getPose();
          double dx = target.getX() - pose.getX();
          double dy = target.getY() - pose.getY();
          Rotation2d targetAngle = new Rotation2d(dx, dy);

          setControl(
              facingAngleRequest
                  .withVelocityX(vx.getAsDouble() * scale)
                  .withVelocityY(vy.getAsDouble() * scale)
                  .withTargetDirection(targetAngle));

          Logger.recordOutput("Drive/FacingPoint/TargetAngleDeg", targetAngle.getDegrees());
          Logger.recordOutput(
              "Drive/FacingPoint/HeadingErrorDeg",
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

  // --- Wheel radius characterization ---

  @Override
  public Command characterizeWheelRadius() {
    SlewRateLimiter rateLimiter = new SlewRateLimiter(0.05);
    double driveBaseRadius = config.driveBaseRadius();
    double[] startPositions = new double[4];
    double[] gyroDelta = {0};
    Rotation2d[] lastHeading = {null};
    boolean[] settled = {false};
    double[] settleTimer = {0};

    return run(() -> {
          double omega = rateLimiter.calculate(1.0);
          setControl(robotCentricRequest.withRotationalRate(omega));

          Rotation2d heading = getHeading();
          if (lastHeading[0] == null) {
            lastHeading[0] = heading;
            settleTimer[0] = Timer.getFPGATimestamp();
            return;
          }

          if (!settled[0]) {
            if (Timer.getFPGATimestamp() - settleTimer[0] >= 1.0) {
              settled[0] = true;
              lastHeading[0] = heading;
              for (int i = 0; i < 4; i++) {
                startPositions[i] = inputs.drivePositionRad[i];
              }
            }
            return;
          }

          gyroDelta[0] += heading.minus(lastHeading[0]).getRadians();
          lastHeading[0] = heading;
        })
        .finallyDo(
            () -> {
              double totalWheelDelta = 0;
              for (int i = 0; i < 4; i++) {
                totalWheelDelta += Math.abs(inputs.drivePositionRad[i] - startPositions[i]);
              }
              double avgWheelDelta = totalWheelDelta / 4.0;

              if (avgWheelDelta < 0.001) {
                System.out.println(
                    "[WheelRadiusCharacterization] Not enough wheel movement to calculate radius");
                return;
              }

              double radius = (Math.abs(gyroDelta[0]) * driveBaseRadius) / avgWheelDelta;
              double radiusInches = radius / 0.0254;

              System.out.printf(
                  "[WheelRadiusCharacterization] Wheel radius: %.4f m (%.3f in)%n",
                  radius, radiusInches);
              Logger.recordOutput("Drive/CharacterizedWheelRadiusM", radius);
              Logger.recordOutput("Drive/CharacterizedWheelRadiusIn", radiusInches);
            });
  }

  // --- SysId characterization ---

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdTranslation.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdTranslation.dynamic(direction);
  }

  // --- DriveInterface: Point-to-point ---

  @Override
  public Command driveToPose(Pose2d target, double toleranceMeters) {
    // PID controllers for X, Y (meters), and rotation (radians)
    AutoDriveConfig ad = config.autoDrive;
    ProfiledPIDController xController =
        new ProfiledPIDController(
            ad.autoTranslationKP,
            ad.autoTranslationKI,
            ad.autoTranslationKD,
            new TrapezoidProfile.Constraints(ad.driveToPoseMaxVelMps, ad.driveToPoseMaxAccelMps2));
    ProfiledPIDController yController =
        new ProfiledPIDController(
            ad.autoTranslationKP,
            ad.autoTranslationKI,
            ad.autoTranslationKD,
            new TrapezoidProfile.Constraints(ad.driveToPoseMaxVelMps, ad.driveToPoseMaxAccelMps2));
    ProfiledPIDController rotController =
        new ProfiledPIDController(
            ad.autoRotationKP,
            ad.autoRotationKI,
            ad.autoRotationKD,
            new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(toleranceMeters);
    yController.setTolerance(toleranceMeters);
    rotController.setTolerance(Math.toRadians(ad.driveToPoseRotToleranceDeg));

    return run(() -> {
          Pose2d current = getPose();
          double scale = getVoltageSpeedScale();
          double vx = xController.calculate(current.getX(), target.getX()) * scale;
          double vy = yController.calculate(current.getY(), target.getY()) * scale;
          double omega =
              rotController.calculate(
                      current.getRotation().getRadians(), target.getRotation().getRadians())
                  * scale;
          setControl(
              driveToPoseRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));

          // PID telemetry for tuning
          Logger.recordOutput("Drive/DriveToPose/ErrorX", xController.getPositionError());
          Logger.recordOutput("Drive/DriveToPose/ErrorY", yController.getPositionError());
          Logger.recordOutput(
              "Drive/DriveToPose/ErrorRotDeg", Math.toDegrees(rotController.getPositionError()));
          Logger.recordOutput("Drive/DriveToPose/OutputVx", vx);
          Logger.recordOutput("Drive/DriveToPose/OutputVy", vy);
          Logger.recordOutput("Drive/DriveToPose/OutputOmega", omega);
          Logger.recordOutput(
              "Drive/DriveToPose/AtGoal",
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
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return linearSpeed > MOVING_THRESHOLD_MPS;
  }

  @Override
  public double distanceTo(Translation2d point) {
    return getPose().getTranslation().getDistance(point);
  }

  @Override
  public Rotation2d angleTo(Translation2d point) {
    Translation2d robot = getPose().getTranslation();
    return new Rotation2d(point.getX() - robot.getX(), point.getY() - robot.getY());
  }

  @Override
  public boolean isNear(Translation2d point, double toleranceMeters) {
    return distanceTo(point) <= toleranceMeters;
  }

  @Override
  public boolean isAimedAt(Translation2d point, double toleranceDegrees) {
    double error = Math.abs(angleTo(point).minus(getHeading()).getDegrees());
    return error <= toleranceDegrees;
  }

  @Override
  public Translation2d getFieldRelativeVelocity() {
    ChassisSpeeds speeds = getVelocity();
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
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
   * Returns standard deviations for vision measurement based on tag count and distance, or null if
   * the measurement should be rejected. Uses distance²/tagCount scaling formula.
   */
  Matrix<N3, N1> getVisionStdDevs(int tagCount, double avgDistM) {
    VisionConfig vc = config.visionConfig;
    if (tagCount == 1 && avgDistM > vc.maxSingleTagDistM) {
      return null; // reject single tag far away
    }
    double distSqOverTags = avgDistM * avgDistM / tagCount;
    double linearStdDev = vc.linearStdDevCoeff * distSqOverTags;
    double angularStdDev = vc.angularStdDevCoeff * distSqOverTags;
    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

  PoseConfidence getPoseConfidence(double secSinceVision, int maxTagCount, double minAmbiguity) {
    if (secSinceVision > 5.0) return PoseConfidence.DEAD_RECKONING;
    if (maxTagCount >= 2 && minAmbiguity <= 0.1 && secSinceVision <= 1.0)
      return PoseConfidence.HIGH;
    if (secSinceVision <= 2.0) return PoseConfidence.MEDIUM;
    return PoseConfidence.LOW;
  }

  boolean isVisionPoseOnField(Pose2d pose) {
    VisionConfig vc = config.visionConfig;
    return pose.getX() >= 0
        && pose.getX() <= vc.fieldMaxX
        && pose.getY() >= 0
        && pose.getY() <= vc.fieldMaxY;
  }

  @Override
  public DrivetrainConfig getConfig() {
    return config;
  }

  public PhotonPipelineResult getLatestCameraResult(int cameraIndex) {
    return io.getLatestResult(cameraIndex);
  }

  /** Set the telemetry consumer to receive DriveState snapshots each cycle. */
  public void setTelemetry(DriveTelemetry telemetry) {
    this.telemetry = telemetry;
  }

  // --- Subsystem ---

  @Override
  public void periodic() {
    periodicCycleCount++;
    boolean fullLogCycle = (periodicCycleCount % 10 == 0);

    applyOperatorPerspective();

    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    SwerveDriveState state = getState();
    if (state == null) return;

    ChassisSpeeds speeds = state.Speeds;
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    logPoseAndSpeed(state, speeds, linearSpeed, fullLogCycle);
    logToSignalLogger(state, linearSpeed);

    if (!loggedOdometryHz && inputs.odometryPeriodSec > 0) {
      double hz = 1.0 / inputs.odometryPeriodSec;
      System.out.printf("[SwerveDrive] High-frequency odometry active: %.0f Hz%n", hz);
      loggedOdometryHz = true;
    }

    double brownoutScale = getVoltageSpeedScale();
    logBrownout(brownoutScale, fullLogCycle);

    Command active = getCurrentCommand();
    Logger.recordOutput("Drive/ActiveCommand", active != null ? active.getName() : "none");
    Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
    Logger.recordOutput("Drive/ModuleTargets", state.ModuleTargets);

    double totalCurrentA = logModuleTelemetry(fullLogCycle);
    fuseVision(state.Pose, fullLogCycle);
    checkDiagnostics(state, fullLogCycle);
    publishDriveState(linearSpeed, totalCurrentA, brownoutScale, active, fullLogCycle);
  }

  private void applyOperatorPerspective() {
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RED_ALLIANCE_PERSPECTIVE
                        : BLUE_ALLIANCE_PERSPECTIVE);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void logPoseAndSpeed(
      SwerveDriveState state, ChassisSpeeds speeds, double linearSpeed, boolean fullLogCycle) {
    Pose2d pose = state.Pose;
    Logger.recordOutput("Drive/Pose", pose);
    Logger.recordOutput("Drive/Speeds", speeds);

    if (fullLogCycle) {
      Logger.recordOutput("Drive/PositionX", pose.getX());
      Logger.recordOutput("Drive/PositionY", pose.getY());
      Logger.recordOutput("Drive/RotationDeg", pose.getRotation().getDegrees());
      Logger.recordOutput("Drive/SpeedMps", linearSpeed);
      Logger.recordOutput("Drive/SpeedPercent", linearSpeed / config.maxSpeedMps * 100.0);
      Logger.recordOutput(
          "Drive/AngularRateDegPerSec", Math.toDegrees(speeds.omegaRadiansPerSecond));
      Logger.recordOutput(
          "Drive/OdometryHz", inputs.odometryPeriodSec > 0 ? 1.0 / inputs.odometryPeriodSec : 0);
    }
  }

  private void logBrownout(double brownoutScale, boolean fullLogCycle) {
    if (fullLogCycle) {
      Logger.recordOutput("Drive/BatteryVoltage", inputs.batteryVoltage);
      Logger.recordOutput("Drive/BrownoutSpeedScale", brownoutScale);
      Logger.recordOutput("Drive/BrownoutActive", brownoutScale < 1.0);
    }
  }

  private void logToSignalLogger(SwerveDriveState state, double linearSpeed) {
    Pose2d pose = state.Pose;
    signalLogPose[0] = pose.getX();
    signalLogPose[1] = pose.getY();
    signalLogPose[2] = pose.getRotation().getRadians();
    SignalLogger.writeDoubleArray("Drive/Pose", signalLogPose);

    ChassisSpeeds speeds = state.Speeds;
    signalLogSpeeds[0] = speeds.vxMetersPerSecond;
    signalLogSpeeds[1] = speeds.vyMetersPerSecond;
    signalLogSpeeds[2] = speeds.omegaRadiansPerSecond;
    SignalLogger.writeDoubleArray("Drive/Speeds", signalLogSpeeds);

    SwerveModuleState[] moduleStates = state.ModuleStates;
    for (int i = 0; i < 4; i++) {
      signalLogModuleStates[i * 2] = moduleStates[i].speedMetersPerSecond;
      signalLogModuleStates[i * 2 + 1] = moduleStates[i].angle.getRadians();
    }
    SignalLogger.writeDoubleArray("Drive/ModuleStates", signalLogModuleStates);

    SignalLogger.writeDouble("Drive/SpeedMps", linearSpeed);
  }

  /** Logs per-module current telemetry and returns total current draw. */
  private double logModuleTelemetry(boolean fullLogCycle) {
    double totalCurrentA = 0;
    for (int i = 0; i < 4; i++) {
      totalCurrentA += inputs.driveCurrentA[i] + inputs.steerCurrentA[i];
    }
    if (fullLogCycle) {
      for (int i = 0; i < 4; i++) {
        String prefix = "Drive/" + MODULE_NAMES[i] + "/";
        Logger.recordOutput(prefix + "DriveCurrentA", inputs.driveCurrentA[i]);
        Logger.recordOutput(prefix + "SteerCurrentA", inputs.steerCurrentA[i]);
      }
      Logger.recordOutput("Drive/TotalCurrentA", totalCurrentA);
    }
    return totalCurrentA;
  }

  private void fuseVision(Pose2d pose, boolean fullLogCycle) {
    int cycleMaxTagCount = 0;
    double cycleMinAmbiguity = 1.0;
    boolean anyAccepted = false;

    for (int i = 0; i < inputs.visionConnected.length; i++) {
      if (fullLogCycle) {
        String vPrefix = "Drive/Vision/" + i + "/";
        Logger.recordOutput(vPrefix + "Connected", inputs.visionConnected[i]);
        Logger.recordOutput(vPrefix + "TagCount", inputs.visionTagCount[i]);
        Logger.recordOutput(vPrefix + "Ambiguity", inputs.visionAmbiguity[i]);
        Logger.recordOutput(vPrefix + "AvgTagDistM", inputs.visionAvgTagDistM[i]);
      }

      if (inputs.visionHasEstimate[i]) {
        Pose2d visionPose =
            new Pose2d(
                inputs.visionPoseX[i],
                inputs.visionPoseY[i],
                Rotation2d.fromDegrees(inputs.visionPoseRotDeg[i]));

        if (fullLogCycle) {
          String vPrefix = "Drive/Vision/" + i + "/";
          Logger.recordOutput(vPrefix + "EstimatedPose", visionPose);
          double odometryDelta = pose.getTranslation().getDistance(visionPose.getTranslation());
          Logger.recordOutput(vPrefix + "OdometryDeltaM", odometryDelta);
          double latencyMs = (Timer.getFPGATimestamp() - inputs.visionTimestampSec[i]) * 1000.0;
          Logger.recordOutput(vPrefix + "LatencyMs", latencyMs);
        }

        if (i < inputs.visionPoseZ.length
            && Math.abs(inputs.visionPoseZ[i]) > config.visionConfig.zHeightThresholdM) {
          if (fullLogCycle) {
            String vPrefix = "Drive/Vision/" + i + "/";
            Logger.recordOutput(vPrefix + "Rejected", true);
            Logger.recordOutput(vPrefix + "RejectReason", "ZHeight");
          }
        } else if (!isVisionPoseOnField(visionPose)) {
          if (fullLogCycle) {
            String vPrefix = "Drive/Vision/" + i + "/";
            Logger.recordOutput(vPrefix + "Rejected", true);
            Logger.recordOutput(vPrefix + "RejectReason", "OutOfBounds");
          }
        } else if (inputs.visionAmbiguity[i] > config.visionConfig.maxAmbiguity) {
          if (fullLogCycle) {
            String vPrefix = "Drive/Vision/" + i + "/";
            Logger.recordOutput(vPrefix + "Rejected", true);
            Logger.recordOutput(vPrefix + "RejectReason", "HighAmbiguity");
          }
        } else {
          Matrix<N3, N1> stdDevs =
              getVisionStdDevs(inputs.visionTagCount[i], inputs.visionAvgTagDistM[i]);
          if (stdDevs == null) {
            if (fullLogCycle) {
              String vPrefix = "Drive/Vision/" + i + "/";
              Logger.recordOutput(vPrefix + "Rejected", true);
              Logger.recordOutput(vPrefix + "RejectReason", "SingleTagFar");
            }
          } else {
            if (fullLogCycle) {
              String vPrefix = "Drive/Vision/" + i + "/";
              Logger.recordOutput(vPrefix + "Rejected", false);
              Logger.recordOutput(vPrefix + "RejectReason", "");
            }
            super.addVisionMeasurement(
                visionPose, Utils.fpgaToCurrentTime(inputs.visionTimestampSec[i]), stdDevs);
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

    PoseConfidence confidence =
        getPoseConfidence(
            Timer.getFPGATimestamp() - lastAcceptedVisionTimeSec,
            lastMaxTagCount,
            lastMinAmbiguity);
    Logger.recordOutput("Drive/PoseConfidence", confidence.name());
  }

  private void publishDriveState(
      double linearSpeed,
      double totalCurrentA,
      double brownoutScale,
      Command active,
      boolean fullLogCycle) {
    if (telemetry != null && fullLogCycle) {
      telemetry.update(
          new DriveState(
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

  private void checkDiagnostics(SwerveDriveState state, boolean fullLogCycle) {
    DiagnosticsConfig diag = config.diagnosticsConfig;
    double now = Timer.getFPGATimestamp();
    boolean cooldownExpired = (now - lastDiagnosticTimeSec) >= diag.diagnosticCooldownSec;
    boolean warned = false;

    // Brownout protection active
    double brownoutScale = getVoltageSpeedScale();
    if (fullLogCycle) {
      Logger.recordOutput("Drive/Diagnostics/BrownoutActive", brownoutScale < 1.0);
    }
    if (brownoutScale < 1.0 && cooldownExpired) {
      DriverStation.reportWarning(
          String.format(
              "Drive: Brownout protection active (%.1fV, %.0f%% speed)",
              inputs.batteryVoltage, brownoutScale * 100.0),
          false);
      warned = true;
    }

    // Stale odometry (CAN timeout indicator)
    double odometryHz = inputs.odometryPeriodSec > 0 ? 1.0 / inputs.odometryPeriodSec : 0;
    boolean odometryStale = odometryHz < diag.odometryMinHz;
    if (fullLogCycle) {
      Logger.recordOutput("Drive/Diagnostics/OdometryStale", odometryStale);
    }
    if (odometryStale && cooldownExpired) {
      DriverStation.reportWarning(
          String.format("Drive: Stale odometry (%.0f Hz) - possible CAN timeout", odometryHz),
          false);
      warned = true;
    }

    // Per-module diagnostics
    SwerveModuleState[] moduleStates = state.ModuleStates;
    SwerveModuleState[] moduleTargets = state.ModuleTargets;
    for (int i = 0; i < 4; i++) {
      String name = MODULE_NAMES[i];

      // Motor temperatures (from IO inputs)
      double driveTemp = inputs.driveTempC[i];
      double steerTemp = inputs.steerTempC[i];
      boolean driveTempWarn = driveTemp > diag.motorTempWarnC;
      boolean steerTempWarn = steerTemp > diag.motorTempWarnC;
      if (fullLogCycle) {
        String diagPrefix = "Drive/Diagnostics/" + name + "/";
        Logger.recordOutput(diagPrefix + "DriveTempWarn", driveTempWarn);
        Logger.recordOutput(diagPrefix + "SteerTempWarn", steerTempWarn);
      }

      if (cooldownExpired) {
        if (driveTemp > diag.motorTempErrorC) {
          DriverStation.reportError(
              String.format("Drive: %s drive motor OVERTEMP (%.0fC)!", name, driveTemp), false);
          warned = true;
        } else if (driveTempWarn) {
          DriverStation.reportWarning(
              String.format("Drive: %s drive motor hot (%.0fC)", name, driveTemp), false);
          warned = true;
        }
        if (steerTemp > diag.motorTempErrorC) {
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
      double angleError =
          Math.abs(moduleTargets[i].angle.minus(moduleStates[i].angle).getDegrees());
      boolean alignmentWarn = angleError > diag.alignmentErrorWarnDeg;
      if (fullLogCycle) {
        String diagPrefix = "Drive/Diagnostics/" + name + "/";
        Logger.recordOutput(diagPrefix + "AlignmentWarn", alignmentWarn);
      }
      if (alignmentWarn && cooldownExpired) {
        DriverStation.reportWarning(
            String.format("Drive: %s module misaligned (%.0f deg error)", name, angleError), false);
        warned = true;
      }

      // Drive current near limit (from IO inputs)
      double driveCurrent = inputs.driveCurrentA[i];
      boolean currentWarn =
          driveCurrent > config.driveStatorCurrentLimit * diag.currentWarnFraction;
      if (fullLogCycle) {
        String diagPrefix = "Drive/Diagnostics/" + name + "/";
        Logger.recordOutput(diagPrefix + "CurrentWarn", currentWarn);
      }
      if (currentWarn && cooldownExpired) {
        DriverStation.reportWarning(
            String.format(
                "Drive: %s drive current high (%.0fA/%.0fA limit)",
                name, driveCurrent, config.driveStatorCurrentLimit),
            false);
        warned = true;
      }

      // Steer current near limit
      double steerCurrent = inputs.steerCurrentA[i];
      boolean steerCurrentWarn =
          steerCurrent > config.steerStatorCurrentLimit * diag.currentWarnFraction;
      if (fullLogCycle) {
        String diagPrefix = "Drive/Diagnostics/" + name + "/";
        Logger.recordOutput(diagPrefix + "SteerCurrentWarn", steerCurrentWarn);
      }
      if (steerCurrentWarn && cooldownExpired) {
        DriverStation.reportWarning(
            String.format(
                "Drive: %s steer current high (%.0fA/%.0fA limit)",
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
      if (inputs.driveTempC[i] > diag.motorTempWarnC
          || inputs.steerTempC[i] > diag.motorTempWarnC) {
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

    if (fullLogCycle) {
      Logger.recordOutput("Drive/AllHealthy", lastAllHealthy);
      Logger.recordOutput("Drive/StatusMessage", lastStatusMessage);
    }
  }

  // --- Brownout protection ---

  double getVoltageSpeedScale() {
    DiagnosticsConfig diag = config.diagnosticsConfig;
    double voltage = inputs.batteryVoltage;
    if (voltage >= diag.brownoutStartV) return 1.0;
    if (voltage <= diag.brownoutMinV) return diag.brownoutMinScale;
    return diag.brownoutMinScale
        + (1.0 - diag.brownoutMinScale)
            * (voltage - diag.brownoutMinV)
            / (diag.brownoutStartV - diag.brownoutMinV);
  }

  // --- Simulation ---

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
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
   * Returns a command that applies the specified swerve request. Convenience method for direct
   * SwerveRequest usage (e.g., in RobotContainer bindings).
   */
  public Command applyRequest(java.util.function.Supplier<SwerveRequest> requestSupplier) {
    return run(() -> setControl(requestSupplier.get()));
  }
}
