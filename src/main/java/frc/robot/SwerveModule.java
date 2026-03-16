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

public class SwerveModule {
  // Constants from tuner-project.json (WCP X1 12T)
  private static final double kWheelRadiusMeters = 0.0508; // 2.0 inches
  private static final double kWheelCircumferenceMeters = 2.0 * Math.PI * kWheelRadiusMeters;
  private static final double kDriveGearRatio = 6.394736842105262;
  private static final double kSteerGearRatio = 12.1;
  private static final double kCouplingRatio = 4.5;
  private static final double kStatorCurrentLimit = 60.0;
  private static final double kSupplyCurrentLimit = 120.0;

  private final TalonFX m_driveMotor;
  private final TalonFX m_steerMotor;
  private final CANcoder m_encoder;

  private final VelocityVoltage m_driveRequest = new VelocityVoltage(0);
  private final PositionVoltage m_steerRequest = new PositionVoltage(0);

  /**
   * Constructs a SwerveModule with Talon FX drive/steer motors and a CANcoder.
   *
   * @param driveMotorId CAN ID of the drive TalonFX.
   * @param steerMotorId CAN ID of the steer TalonFX.
   * @param encoderId CAN ID of the CANcoder.
   * @param canbus Name of the CAN bus.
   * @param encoderOffset Magnet offset for the CANcoder in rotations.
   * @param driveInverted Whether the drive motor is inverted.
   * @param steerInverted Whether the steer motor is inverted.
   */
  public SwerveModule(
      int driveMotorId,
      int steerMotorId,
      int encoderId,
      String canbus,
      double encoderOffset,
      boolean driveInverted,
      boolean steerInverted) {

    m_driveMotor = new TalonFX(driveMotorId, canbus);
    m_steerMotor = new TalonFX(steerMotorId, canbus);
    m_encoder = new CANcoder(encoderId, canbus);

    // Configure CANcoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = encoderOffset;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    m_encoder.getConfigurator().apply(encoderConfig);

    // Configure drive motor
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted =
        driveInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = kDriveGearRatio;
    driveConfig.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    // PID gains - tune these for your robot!
    driveConfig.Slot0.kP = 0.1;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = 0.0;
    driveConfig.Slot0.kV = 0.12;
    m_driveMotor.getConfigurator().apply(driveConfig);

    // Configure steer motor
    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.Inverted =
        steerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.Feedback.FeedbackRemoteSensorID = encoderId;
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerConfig.Feedback.RotorToSensorRatio = kSteerGearRatio;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    // PID gains - tune these for your robot!
    steerConfig.Slot0.kP = 100.0;
    steerConfig.Slot0.kI = 0.0;
    steerConfig.Slot0.kD = 0.5;
    m_steerMotor.getConfigurator().apply(steerConfig);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double driveVelocityMps =
        m_driveMotor.getVelocity().getValueAsDouble() * kWheelCircumferenceMeters;
    Rotation2d steerAngle =
        Rotation2d.fromRotations(m_steerMotor.getPosition().getValueAsDouble());
    return new SwerveModuleState(driveVelocityMps, steerAngle);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double drivePositionMeters =
        m_driveMotor.getPosition().getValueAsDouble() * kWheelCircumferenceMeters;
    Rotation2d steerAngle =
        Rotation2d.fromRotations(m_steerMotor.getPosition().getValueAsDouble());
    return new SwerveModulePosition(drivePositionMeters, steerAngle);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var steerAngle = Rotation2d.fromRotations(m_steerMotor.getPosition().getValueAsDouble());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(steerAngle);

    // Scale speed by cosine of angle error for smoother driving
    desiredState.cosineScale(steerAngle);

    // Convert m/s to wheel rotations per second
    double driveRps = desiredState.speedMetersPerSecond / kWheelCircumferenceMeters;
    m_driveMotor.setControl(m_driveRequest.withVelocity(driveRps));

    // Command steer position in rotations
    m_steerMotor.setControl(m_steerRequest.withPosition(desiredState.angle.getRotations()));
  }
}
