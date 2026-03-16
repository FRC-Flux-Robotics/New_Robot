// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.drivetrain.ModuleConfig;
import frc.lib.drivetrain.PIDGains;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_encoder;

  private final double m_wheelCircumferenceMeters;
  private final double m_driveGearRatio;
  private final double m_steerGearRatio;
  private final int m_encoderId;

  private final VelocityVoltage m_driveRequest = new VelocityVoltage(0);
  private final PositionVoltage m_steerRequest = new PositionVoltage(0);

  public SwerveModule(
      ModuleConfig moduleConfig,
      PIDGains steerGains,
      PIDGains driveGains,
      double driveStatorLimit,
      double driveSupplyLimit,
      double steerStatorLimit,
      String canbus,
      double driveGearRatio,
      double steerGearRatio,
      double wheelRadiusInches) {

    double wheelRadiusMeters = edu.wpi.first.math.util.Units.inchesToMeters(wheelRadiusInches);
    m_wheelCircumferenceMeters = 2.0 * Math.PI * wheelRadiusMeters;
    m_driveGearRatio = driveGearRatio;
    m_steerGearRatio = steerGearRatio;
    m_encoderId = moduleConfig.encoderId;

    m_driveMotor = new TalonFX(moduleConfig.driveMotorId, canbus);
    m_steerMotor = new TalonFX(moduleConfig.steerMotorId, canbus);
    m_encoder = new CANcoder(moduleConfig.encoderId, canbus);

    // Configure CANcoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = moduleConfig.encoderOffset;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_encoder.getConfigurator().apply(encoderConfig);

    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted =
        moduleConfig.invertDrive
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = driveGearRatio;
    applyCurrentLimitsToConfig(driveConfig, driveStatorLimit, driveSupplyLimit);
    driveConfig.Slot0.kP = driveGains.kP;
    driveConfig.Slot0.kI = driveGains.kI;
    driveConfig.Slot0.kD = driveGains.kD;
    driveConfig.Slot0.kS = driveGains.kS;
    driveConfig.Slot0.kV = driveGains.kV;
    driveConfig.Slot0.kA = driveGains.kA;
    m_driveMotor.getConfigurator().apply(driveConfig);

    // Configure steer motor
    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.Inverted =
        moduleConfig.invertSteer
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.Feedback.FeedbackRemoteSensorID = moduleConfig.encoderId;
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerConfig.Feedback.RotorToSensorRatio = steerGearRatio;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    if (steerStatorLimit > 0) {
      steerConfig.CurrentLimits.StatorCurrentLimit = steerStatorLimit;
      steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    }
    steerConfig.Slot0.kP = steerGains.kP;
    steerConfig.Slot0.kI = steerGains.kI;
    steerConfig.Slot0.kD = steerGains.kD;
    steerConfig.Slot0.kS = steerGains.kS;
    steerConfig.Slot0.kV = steerGains.kV;
    steerConfig.Slot0.kA = steerGains.kA;
    m_steerMotor.getConfigurator().apply(steerConfig);
  }

  private void applyCurrentLimitsToConfig(
      TalonFXConfiguration config, double statorLimit, double supplyLimit) {
    if (statorLimit > 0) {
      config.CurrentLimits.StatorCurrentLimit = statorLimit;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
    }
    if (supplyLimit > 0) {
      config.CurrentLimits.SupplyCurrentLimit = supplyLimit;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
  }

  public void applyCurrentLimits(double driveStator, double driveSupply, double steerStator) {
    var driveConfig = new TalonFXConfiguration();
    // Re-read current config and update current limits
    m_driveMotor.getConfigurator().refresh(driveConfig);
    applyCurrentLimitsToConfig(driveConfig, driveStator, driveSupply);
    if (driveStator <= 0) {
      driveConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    }
    if (driveSupply <= 0) {
      driveConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    }
    m_driveMotor.getConfigurator().apply(driveConfig.CurrentLimits);

    var steerConfig = new TalonFXConfiguration();
    m_steerMotor.getConfigurator().refresh(steerConfig);
    if (steerStator > 0) {
      steerConfig.CurrentLimits.StatorCurrentLimit = steerStator;
      steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    } else {
      steerConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    }
    m_steerMotor.getConfigurator().apply(steerConfig.CurrentLimits);
  }

  public void applySteerPID(PIDGains gains) {
    var config = new TalonFXConfiguration();
    m_steerMotor.getConfigurator().refresh(config);
    config.Slot0.kP = gains.kP;
    config.Slot0.kI = gains.kI;
    config.Slot0.kD = gains.kD;
    config.Slot0.kS = gains.kS;
    config.Slot0.kV = gains.kV;
    config.Slot0.kA = gains.kA;
    m_steerMotor.getConfigurator().apply(config.Slot0);
  }

  public double getDriveCurrent() {
    return m_driveMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getSteerCurrent() {
    return m_steerMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getAngle() {
    return m_steerMotor.getPosition().getValueAsDouble() * 360.0;
  }

  public SwerveModuleState getState() {
    double driveVelocityMps =
        m_driveMotor.getVelocity().getValueAsDouble() * m_wheelCircumferenceMeters;
    Rotation2d steerAngle =
        Rotation2d.fromRotations(m_steerMotor.getPosition().getValueAsDouble());
    return new SwerveModuleState(driveVelocityMps, steerAngle);
  }

  public SwerveModulePosition getPosition() {
    double drivePositionMeters =
        m_driveMotor.getPosition().getValueAsDouble() * m_wheelCircumferenceMeters;
    Rotation2d steerAngle =
        Rotation2d.fromRotations(m_steerMotor.getPosition().getValueAsDouble());
    return new SwerveModulePosition(drivePositionMeters, steerAngle);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    var steerAngle = Rotation2d.fromRotations(m_steerMotor.getPosition().getValueAsDouble());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(steerAngle);

    // Scale speed by cosine of angle error for smoother driving
    desiredState.cosineScale(steerAngle);

    // Convert m/s to wheel rotations per second
    double driveRps = desiredState.speedMetersPerSecond / m_wheelCircumferenceMeters;
    m_driveMotor.setControl(m_driveRequest.withVelocity(driveRps));

    // Command steer position in rotations
    m_steerMotor.setControl(m_steerRequest.withPosition(desiredState.angle.getRotations()));
  }
}
