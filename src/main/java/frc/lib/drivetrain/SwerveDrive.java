package frc.lib.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

/** Swerve drivetrain implementation backed by CTRE SwerveDrivetrain. */
public class SwerveDrive extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements DriveInterface {

  private final DrivetrainConfig m_config;
  private final double m_maxSpeed;
  private final double m_maxAngularSpeed;

  private final SwerveRequest.FieldCentric m_fieldCentric =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  private final SwerveRequest.RobotCentric m_robotCentric =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final SwerveRequest.FieldCentricFacingAngle m_facingAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

  private final SwerveRequest.ApplyRobotSpeeds m_applyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final SwerveRequest.Idle m_idle = new SwerveRequest.Idle();
  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

  /* SysId characterization requests */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  /* SysId routine for characterizing drive motors (translation) */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Default ramp rate (1 V/s)
              Volts.of(4), // 4V dynamic step to prevent brownout
              null, // Default timeout (10 s)
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer motors */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Default ramp rate (1 V/s)
              Volts.of(7), // 7V dynamic step
              null, // Default timeout (10 s)
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_steerCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing rotation (heading controller gains) */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(Math.PI / 6).per(Second), // rad/s² (SysId only supports "volts per second")
              Volts.of(Math.PI), // rad/s (SysId only supports "volts")
              null, // Default timeout (10 s)
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  private double m_lastSimTime;

  private final DrivetrainIO m_io;
  private final DrivetrainIOInputsAutoLogged m_ioInputs = new DrivetrainIOInputsAutoLogged();

  // Cached telemetry state — written by CTRE odometry thread, read by main thread in periodic()
  private volatile SwerveDriveState m_cachedState;

  /** Creates a SwerveDrive with real hardware IO. */
  public SwerveDrive(DrivetrainConfig config) {
    this(config, null);
  }

  /** Creates a SwerveDrive with the given IO (pass DrivetrainIOReplay for replay mode). */
  public SwerveDrive(DrivetrainConfig config, DrivetrainIO io) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        buildDrivetrainConstants(config),
        buildModuleConstants(config));

    m_config = config;
    m_maxSpeed = config.maxSpeedMps * config.speedCoefficient;
    m_maxAngularSpeed = config.maxAngularRateRadPerSec * config.speedCoefficient;

    m_fieldCentric
        .withDeadband(m_maxSpeed * config.translationDeadband)
        .withRotationalDeadband(m_maxAngularSpeed * config.rotationDeadband);

    m_robotCentric
        .withDeadband(m_maxSpeed * config.translationDeadband)
        .withRotationalDeadband(m_maxAngularSpeed * config.rotationDeadband);

    m_facingAngle.HeadingController.setPID(
        config.headingGains.kP, config.headingGains.kI, config.headingGains.kD);
    m_facingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    m_facingAngle
        .withDeadband(m_maxSpeed * config.translationDeadband)
        .withRotationalDeadband(m_maxAngularSpeed * config.rotationDeadband);

    m_io = (io != null) ? io : new DrivetrainIOTalonFX(this, config.canBusName);

    configurePathPlanner();
    registerTelemetry(this::cacheTelemetry);

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();
    var simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.setName("SwerveSim");
    simNotifier.startPeriodic(0.005);
  }

  private static SwerveDrivetrainConstants buildDrivetrainConstants(DrivetrainConfig config) {
    return new SwerveDrivetrainConstants()
        .withCANBusName(config.canBusName)
        .withPigeon2Id(config.pigeonId);
  }

  @SuppressWarnings("unchecked")
  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      []
      buildModuleConstants(DrivetrainConfig config) {

    Slot0Configs steerGains =
        new Slot0Configs()
            .withKP(config.steerGains.kP)
            .withKI(config.steerGains.kI)
            .withKD(config.steerGains.kD)
            .withKS(config.steerGains.kS)
            .withKV(config.steerGains.kV)
            .withKA(config.steerGains.kA)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    Slot0Configs driveGains =
        new Slot0Configs()
            .withKP(config.driveGains.kP)
            .withKI(config.driveGains.kI)
            .withKD(config.driveGains.kD)
            .withKS(config.driveGains.kS)
            .withKV(config.driveGains.kV)
            .withKA(config.driveGains.kA);

    // Drive motor initial configs: current limits + brake mode
    TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    driveInitialConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (config.driveStatorCurrentLimit > 0) {
      driveInitialConfigs.CurrentLimits.StatorCurrentLimit = config.driveStatorCurrentLimit;
      driveInitialConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    }
    if (config.driveSupplyCurrentLimit > 0) {
      driveInitialConfigs.CurrentLimits.SupplyCurrentLimit = config.driveSupplyCurrentLimit;
      driveInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    // Steer motor initial configs: current limits + MotionMagicExpo parameters
    TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration();
    if (config.steerStatorCurrentLimit > 0) {
      steerInitialConfigs.CurrentLimits.StatorCurrentLimit = config.steerStatorCurrentLimit;
      steerInitialConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    }
    steerInitialConfigs.MotionMagic.MotionMagicExpo_kV = config.steerMotionMagicExpoKv;
    steerInitialConfigs.MotionMagic.MotionMagicExpo_kA = config.steerMotionMagicExpoKa;

    CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        factory =
            new SwerveModuleConstantsFactory<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(config.driveGearRatio)
                .withSteerMotorGearRatio(config.steerGearRatio)
                .withCouplingGearRatio(config.couplingRatio)
                .withWheelRadius(Inches.of(config.wheelRadiusInches))
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSlipCurrent(Amps.of(120))
                .withSpeedAt12Volts(MetersPerSecond.of(config.maxSpeedMps))
                .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(KilogramSquareMeters.of(0.01))
                .withDriveInertia(KilogramSquareMeters.of(0.01))
                .withSteerFrictionVoltage(Volts.of(0.2))
                .withDriveFrictionVoltage(Volts.of(0.2));

    return new SwerveModuleConstants[] {
      createModule(factory, config.frontLeft),
      createModule(factory, config.frontRight),
      createModule(factory, config.backLeft),
      createModule(factory, config.backRight)
    };
  }

  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      createModule(
          SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              factory,
          ModuleConfig module) {
    return factory.createModuleConstants(
        module.steerMotorId,
        module.driveMotorId,
        module.encoderId,
        Rotations.of(module.encoderOffset),
        Inches.of(module.xPositionInches),
        Inches.of(module.yPositionInches),
        module.invertDrive,
        module.invertSteer,
        false);
  }

  private void configurePathPlanner() {
    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getVelocity,
        (speeds, feedforwards) -> {
          var discretized = ChassisSpeeds.discretize(speeds, 0.02);
          this.setControl(
              m_applyRobotSpeeds
                  .withSpeeds(discretized)
                  .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                  .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
        },
        new PPHolonomicDriveController(
            new PIDConstants(10.0, 0.0, 0.0), new PIDConstants(7.0, 0.0, 0.0)),
        buildPathPlannerConfig(m_config),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
  }

  /**
   * Creates a command to follow a pre-built PathPlanner path.
   *
   * @param pathName name of the path file (without extension) in deploy/pathplanner/paths/
   * @return command that follows the path, or a no-op command if the path fails to load
   */
  @Override
  public Command followPathCommand(String pathName) {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: " + pathName, e.getStackTrace());
      return Commands.none();
    }
  }

  private static RobotConfig buildPathPlannerConfig(DrivetrainConfig config) {
    double wheelRadiusMeters = Units.inchesToMeters(config.wheelRadiusInches);
    double halfTrackMeters = Units.inchesToMeters(config.trackWidthInches / 2.0);
    double halfLengthMeters = Units.inchesToMeters(config.trackLengthInches / 2.0);

    com.pathplanner.lib.config.ModuleConfig moduleConfig =
        new com.pathplanner.lib.config.ModuleConfig(
            wheelRadiusMeters,
            config.maxSpeedMps,
            1.0,
            DCMotor.getKrakenX60(1),
            config.driveGearRatio,
            config.driveStatorCurrentLimit,
            1);

    return new RobotConfig(
        60.0,
        6.0,
        moduleConfig,
        new Translation2d(halfLengthMeters, halfTrackMeters),
        new Translation2d(halfLengthMeters, -halfTrackMeters),
        new Translation2d(-halfLengthMeters, halfTrackMeters),
        new Translation2d(-halfLengthMeters, -halfTrackMeters));
  }

  // --- DriveInterface methods ---

  @Override
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    // Discretize to prevent curved paths when translating and rotating simultaneously
    ChassisSpeeds speeds = ChassisSpeeds.discretize(xSpeed, ySpeed, rot, periodSeconds);

    if (fieldRelative) {
      this.setControl(
          m_fieldCentric
              .withVelocityX(speeds.vxMetersPerSecond)
              .withVelocityY(speeds.vyMetersPerSecond)
              .withRotationalRate(speeds.omegaRadiansPerSecond));
    } else {
      this.setControl(
          m_robotCentric
              .withVelocityX(speeds.vxMetersPerSecond)
              .withVelocityY(speeds.vyMetersPerSecond)
              .withRotationalRate(speeds.omegaRadiansPerSecond));
    }
  }

  @Override
  public void driveFieldCentricFacingAngle(
      double xSpeed, double ySpeed, Rotation2d targetAngle, double periodSeconds) {
    this.setControl(
        m_facingAngle.withVelocityX(xSpeed).withVelocityY(ySpeed).withTargetDirection(targetAngle));
  }

  @Override
  public void updateOdometry() {
    // No-op: CTRE handles odometry internally on a high-frequency thread
  }

  @Override
  public Pose2d getPose() {
    return this.getState().Pose;
  }

  @Override
  public ChassisSpeeds getVelocity() {
    return this.getState().Speeds;
  }

  @Override
  public Rotation2d getHeading() {
    return getPigeon2().getRotation2d();
  }

  @Override
  public DriveState getDriveState() {
    return new DriveState(getPose(), getVelocity(), getHeading());
  }

  @Override
  public void resetHeading() {
    seedFieldCentric();
  }

  @Override
  public void resetPose(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  public void setDeadband(double translationFraction, double rotationFraction) {
    double transDb = m_maxSpeed * translationFraction;
    double rotDb = m_maxAngularSpeed * rotationFraction;
    m_fieldCentric.withDeadband(transDb).withRotationalDeadband(rotDb);
    m_robotCentric.withDeadband(transDb).withRotationalDeadband(rotDb);
    m_facingAngle.withDeadband(transDb).withRotationalDeadband(rotDb);
  }

  @Override
  public void setIdle() {
    this.setControl(m_idle);
  }

  @Override
  public void setBrake() {
    this.setControl(m_brake);
  }

  @Override
  public void setOperatorForward(Rotation2d forward) {
    setOperatorPerspectiveForward(forward);
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    super.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  @Override
  public DrivetrainConfig getConfig() {
    return m_config;
  }

  @Override
  public double getMaxSpeed() {
    return m_maxSpeed;
  }

  @Override
  public double getMaxAngularSpeed() {
    return m_maxAngularSpeed;
  }

  /** Returns a command to run SysId quasistatic test in the given direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /** Returns a command to run SysId dynamic test in the given direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  private static final double kWheelRadiusCharacterizationSpeed = 1.0; // rad/s

  /**
   * Returns a command that spins the robot in place to measure effective wheel radius. Hold the
   * button to spin; release to stop and read the result from SmartDashboard
   * ("WheelRadius/RadiusInches") or the Driver Station console.
   */
  public Command wheelRadiusCharacterization() {
    final double[] initialYawRotations = {0};
    final double[] initialDrivePositions = new double[4];
    final double[] lastRadiusInches = {0};

    ModuleConfig[] moduleConfigs = {
      m_config.frontLeft, m_config.frontRight, m_config.backLeft, m_config.backRight
    };

    return Commands.sequence(
            // Record initial gyro yaw and drive motor positions
            runOnce(
                () -> {
                  initialYawRotations[0] = getPigeon2().getYaw().getValueAsDouble();
                  var modules = getModules();
                  for (int i = 0; i < 4; i++) {
                    initialDrivePositions[i] =
                        modules[i].getDriveMotor().getPosition().getValueAsDouble();
                  }
                }),
            // Spin and continuously measure wheel radius
            run(
                () -> {
                  setControl(
                      m_rotationCharacterization.withRotationalRate(
                          kWheelRadiusCharacterizationSpeed));

                  double gyroYawRotations = getPigeon2().getYaw().getValueAsDouble();
                  double gyroDeltaRadians =
                      (gyroYawRotations - initialYawRotations[0]) * 2.0 * Math.PI;

                  if (Math.abs(gyroDeltaRadians) < 0.1) {
                    return; // Not enough rotation yet
                  }

                  double radiusSum = 0;
                  int validModules = 0;
                  var modules = getModules();

                  for (int i = 0; i < 4; i++) {
                    double motorDelta =
                        modules[i].getDriveMotor().getPosition().getValueAsDouble()
                            - initialDrivePositions[i];
                    double wheelRotations = Math.abs(motorDelta) / m_config.driveGearRatio;

                    if (wheelRotations < 0.01) {
                      continue; // Skip modules with negligible rotation
                    }

                    double moduleRadiusMeters =
                        Math.hypot(
                            Units.inchesToMeters(moduleConfigs[i].xPositionInches),
                            Units.inchesToMeters(moduleConfigs[i].yPositionInches));

                    double wheelRadiusMeters =
                        (moduleRadiusMeters * Math.abs(gyroDeltaRadians))
                            / (wheelRotations * 2.0 * Math.PI);
                    radiusSum += wheelRadiusMeters;
                    validModules++;
                  }

                  if (validModules > 0) {
                    double avgRadiusMeters = radiusSum / validModules;
                    lastRadiusInches[0] = Units.metersToInches(avgRadiusMeters);

                    SmartDashboard.putNumber("WheelRadius/RadiusInches", lastRadiusInches[0]);
                    SmartDashboard.putNumber("WheelRadius/RadiusMeters", avgRadiusMeters);
                    SmartDashboard.putNumber(
                        "WheelRadius/GyroRotationsDeg", Math.toDegrees(gyroDeltaRadians));
                    Logger.recordOutput("Drive/WheelRadiusCharacterization", lastRadiusInches[0]);
                  }
                }))
        .finallyDo(
            () -> {
              setControl(m_idle);
              if (lastRadiusInches[0] > 0) {
                DriverStation.reportWarning(
                    String.format(
                        "Measured wheel radius: %.4f inches (%.4f meters)",
                        lastRadiusInches[0], Units.inchesToMeters(lastRadiusInches[0])),
                    false);
              }
            });
  }

  private void cacheTelemetry(SwerveDrivetrain.SwerveDriveState state) {
    m_cachedState = state;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_ioInputs);
    Logger.processInputs("Drive", m_ioInputs);

    // Log telemetry cached from the odometry thread (safe: main thread only)
    SwerveDriveState state = m_cachedState;
    if (state != null) {
      Logger.recordOutput("Drive/Pose", state.Pose);
      Logger.recordOutput("Drive/Speeds", state.Speeds);

      for (int i = 0; i < state.ModuleStates.length; i++) {
        Logger.recordOutput(
            "Drive/Module" + i + "/Angle", state.ModuleStates[i].angle.getDegrees());
        Logger.recordOutput(
            "Drive/Module" + i + "/Speed", state.ModuleStates[i].speedMetersPerSecond);
      }

      for (int i = 0; i < state.ModuleTargets.length; i++) {
        Logger.recordOutput(
            "Drive/Module" + i + "/TargetAngle", state.ModuleTargets[i].angle.getDegrees());
        Logger.recordOutput(
            "Drive/Module" + i + "/TargetSpeed", state.ModuleTargets[i].speedMetersPerSecond);
      }

      Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);
    }
  }
}
