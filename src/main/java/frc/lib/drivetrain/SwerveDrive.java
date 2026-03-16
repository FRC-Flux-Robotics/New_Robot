package frc.lib.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Swerve drivetrain implementation backed by CTRE SwerveDrivetrain. */
public class SwerveDrive
    extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements Subsystem, DriveInterface {

  private final DrivetrainConfig m_config;
  private final double m_maxSpeed;
  private final double m_maxAngularSpeed;

  private final SwerveRequest.FieldCentric m_fieldCentric =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.RobotCentric m_robotCentric =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final DrivetrainIO m_io;
  private final DrivetrainIO.Inputs m_ioInputs = new DrivetrainIO.Inputs();

  public SwerveDrive(DrivetrainConfig config) {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        buildDrivetrainConstants(config),
        buildModuleConstants(config));

    m_config = config;
    m_maxSpeed = config.maxSpeedMps;
    m_maxAngularSpeed = config.maxAngularRateRadPerSec;

    m_fieldCentric
        .withDeadband(m_maxSpeed * config.translationDeadband)
        .withRotationalDeadband(m_maxAngularSpeed * config.rotationDeadband);

    m_robotCentric
        .withDeadband(m_maxSpeed * config.translationDeadband)
        .withRotationalDeadband(m_maxAngularSpeed * config.rotationDeadband);

    m_io = new DrivetrainIOTalonFX(this);
  }

  private static SwerveDrivetrainConstants buildDrivetrainConstants(DrivetrainConfig config) {
    return new SwerveDrivetrainConstants()
        .withCANBusName(config.canBusName)
        .withPigeon2Id(config.pigeonId);
  }

  @SuppressWarnings("unchecked")
  private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
      buildModuleConstants(DrivetrainConfig config) {

    Slot0Configs steerGains = new Slot0Configs()
        .withKP(config.steerGains.kP)
        .withKI(config.steerGains.kI)
        .withKD(config.steerGains.kD)
        .withKS(config.steerGains.kS)
        .withKV(config.steerGains.kV)
        .withKA(config.steerGains.kA)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    Slot0Configs driveGains = new Slot0Configs()
        .withKP(config.driveGains.kP)
        .withKI(config.driveGains.kI)
        .withKD(config.driveGains.kD)
        .withKS(config.driveGains.kS)
        .withKV(config.driveGains.kV)
        .withKA(config.driveGains.kA);

    // Drive motor initial configs: current limits
    TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    if (config.driveStatorCurrentLimit > 0) {
      driveInitialConfigs.CurrentLimits.StatorCurrentLimit = config.driveStatorCurrentLimit;
      driveInitialConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    }
    if (config.driveSupplyCurrentLimit > 0) {
      driveInitialConfigs.CurrentLimits.SupplyCurrentLimit = config.driveSupplyCurrentLimit;
      driveInitialConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    // Steer motor initial configs: current limits (if any)
    TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration();
    if (config.steerStatorCurrentLimit > 0) {
      steerInitialConfigs.CurrentLimits.StatorCurrentLimit = config.steerStatorCurrentLimit;
      steerInitialConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    }

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

  private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      createModule(
          SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> factory,
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

  // --- DriveInterface methods ---

  @Override
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    if (fieldRelative) {
      this.setControl(
          m_fieldCentric
              .withVelocityX(xSpeed)
              .withVelocityY(ySpeed)
              .withRotationalRate(rot));
    } else {
      this.setControl(
          m_robotCentric
              .withVelocityX(xSpeed)
              .withVelocityY(ySpeed)
              .withRotationalRate(rot));
    }
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

  @Override
  public void periodic() {
    m_io.updateInputs(m_ioInputs);
  }

  /** Returns the latest batch-refreshed IO inputs. */
  public DrivetrainIO.Inputs getIOInputs() {
    return m_ioInputs;
  }

  // --- Implementation-specific methods for TunableDashboard ---

  public void applyCurrentLimits(double driveStator, double driveSupply, double steerStator) {
    for (int i = 0; i < 4; i++) {
      SwerveModule<TalonFX, TalonFX, CANcoder> module = getModule(i);

      var driveConfig = new CurrentLimitsConfigs();
      module.getDriveMotor().getConfigurator().refresh(driveConfig);
      if (driveStator > 0) {
        driveConfig.StatorCurrentLimit = driveStator;
        driveConfig.StatorCurrentLimitEnable = true;
      } else {
        driveConfig.StatorCurrentLimitEnable = false;
      }
      if (driveSupply > 0) {
        driveConfig.SupplyCurrentLimit = driveSupply;
        driveConfig.SupplyCurrentLimitEnable = true;
      } else {
        driveConfig.SupplyCurrentLimitEnable = false;
      }
      module.getDriveMotor().getConfigurator().apply(driveConfig);

      var steerConfig = new CurrentLimitsConfigs();
      module.getSteerMotor().getConfigurator().refresh(steerConfig);
      if (steerStator > 0) {
        steerConfig.StatorCurrentLimit = steerStator;
        steerConfig.StatorCurrentLimitEnable = true;
      } else {
        steerConfig.StatorCurrentLimitEnable = false;
      }
      module.getSteerMotor().getConfigurator().apply(steerConfig);
    }
  }

  public void applySteerPID(PIDGains gains) {
    for (int i = 0; i < 4; i++) {
      SwerveModule<TalonFX, TalonFX, CANcoder> module = getModule(i);
      var config = new Slot0Configs();
      module.getSteerMotor().getConfigurator().refresh(config);
      config.kP = gains.kP;
      config.kI = gains.kI;
      config.kD = gains.kD;
      config.kS = gains.kS;
      config.kV = gains.kV;
      config.kA = gains.kA;
      module.getSteerMotor().getConfigurator().apply(config);
    }
  }

}
