package frc.lib.mechanism;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.util.PhoenixSignals;
import frc.lib.util.PhoenixUtil;

/** Real hardware IO for a dual TalonFX motor mechanism with batched CAN refresh. */
public class MechanismIODualTalonFX implements MechanismIO {

  private final TalonFX m_motor;
  private final TalonFX m_follower;

  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage m_motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  private final StatusSignal<Angle> m_position;
  private final StatusSignal<AngularVelocity> m_velocity;
  private final StatusSignal<Current> m_statorCurrent;
  private final StatusSignal<Current> m_supplyCurrent;
  private final StatusSignal<Voltage> m_voltage;
  private final StatusSignal<Temperature> m_temp;

  private final StatusSignal<Current> m_followerStatorCurrent;
  private final StatusSignal<Current> m_followerSupplyCurrent;

  private final BaseStatusSignal[] m_allSignals;
  private final int m_signalGroup;

  public MechanismIODualTalonFX(MechanismConfig config) {
    if (!config.isDualMotor()) {
      throw new IllegalArgumentException(
          "MechanismIODualTalonFX requires a dual-motor config (secondMotorId must be set)");
    }

    m_motor = new TalonFX(config.motorId, config.canBus);
    m_follower = new TalonFX(config.secondMotorId, config.canBus);

    // Build shared config
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    talonConfig.Slot0.kP = config.pidGains.kP;
    talonConfig.Slot0.kI = config.pidGains.kI;
    talonConfig.Slot0.kD = config.pidGains.kD;
    talonConfig.Slot0.kS = config.pidGains.kS;
    talonConfig.Slot0.kV = config.pidGains.kV;
    talonConfig.Slot0.kA = config.pidGains.kA;

    talonConfig.MotorOutput.NeutralMode =
        config.neutralModeBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    if (config.hasStatorCurrentLimit()) {
      talonConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(config.statorCurrentLimit));
      talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    }
    if (config.hasSupplyCurrentLimit()) {
      talonConfig.CurrentLimits.withSupplyCurrentLimit(Amps.of(config.supplyCurrentLimit));
      talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    talonConfig.Voltage.withPeakForwardVoltage(Volts.of(config.peakVoltage));
    talonConfig.Voltage.withPeakReverseVoltage(Volts.of(-config.peakVoltage));

    if (config.hasSoftLimits()) {
      talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.softLimitForward;
      talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.softLimitReverse;
    }

    if (config.hasMotionMagic()) {
      talonConfig.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicCruiseVelocity;
      talonConfig.MotionMagic.MotionMagicAcceleration = config.motionMagicAcceleration;
    }

    // Primary motor inversion
    InvertedValue primaryInversion =
        config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Follower inversion: flip if counter-rotating
    InvertedValue followerInversion;
    if (config.counterRotating) {
      followerInversion =
          config.inverted
              ? InvertedValue.CounterClockwise_Positive
              : InvertedValue.Clockwise_Positive;
    } else {
      followerInversion = primaryInversion;
    }

    // Apply to primary
    talonConfig.MotorOutput.Inverted = primaryInversion;
    PhoenixUtil.tryUntilOk(() -> m_motor.getConfigurator().apply(talonConfig), 5);

    // Apply to follower with its own inversion
    talonConfig.MotorOutput.Inverted = followerInversion;
    PhoenixUtil.tryUntilOk(() -> m_follower.getConfigurator().apply(talonConfig), 5);

    // Cache status signals — primary motor
    m_position = m_motor.getPosition();
    m_velocity = m_motor.getVelocity();
    m_statorCurrent = m_motor.getStatorCurrent();
    m_supplyCurrent = m_motor.getSupplyCurrent();
    m_voltage = m_motor.getMotorVoltage();
    m_temp = m_motor.getDeviceTemp();

    // Follower — only need currents for summing
    m_followerStatorCurrent = m_follower.getStatorCurrent();
    m_followerSupplyCurrent = m_follower.getSupplyCurrent();

    m_allSignals =
        new BaseStatusSignal[] {
          m_position,
          m_velocity,
          m_statorCurrent,
          m_supplyCurrent,
          m_voltage,
          m_temp,
          m_followerStatorCurrent,
          m_followerSupplyCurrent
        };

    m_signalGroup = PhoenixSignals.register(config.canBus, m_allSignals);
  }

  /** Returns the primary TalonFX motor for control requests. */
  public TalonFX getMotor() {
    return m_motor;
  }

  /** Returns the follower TalonFX motor for control requests. */
  public TalonFX getFollowerMotor() {
    return m_follower;
  }

  @Override
  public void setPosition(double positionRotations) {
    m_motor.setControl(m_positionRequest.withPosition(positionRotations));
  }

  @Override
  public void setVelocity(double velocityRPS) {
    m_motor.setControl(m_velocityRequest.withVelocity(velocityRPS));
    m_follower.setControl(m_velocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setMotionMagicPosition(double positionRotations) {
    m_motor.setControl(m_motionMagicRequest.withPosition(positionRotations));
  }

  @Override
  public void stop() {
    m_motor.setControl(m_neutralRequest);
    m_follower.setControl(m_neutralRequest);
  }

  @Override
  public void resetPosition() {
    m_motor.setPosition(0);
  }

  @Override
  public void updateInputs(MechanismIOInputsAutoLogged inputs) {
    inputs.positionRotations = m_position.getValueAsDouble();
    inputs.velocityRPM = m_velocity.getValueAsDouble() * 60.0;
    inputs.statorCurrentA =
        m_statorCurrent.getValueAsDouble() + m_followerStatorCurrent.getValueAsDouble();
    inputs.supplyCurrentA =
        m_supplyCurrent.getValueAsDouble() + m_followerSupplyCurrent.getValueAsDouble();
    inputs.appliedVoltage = m_voltage.getValueAsDouble();
    inputs.tempCelsius = m_temp.getValueAsDouble();
    inputs.motorConnected = PhoenixSignals.isOK(m_signalGroup);
  }
}
