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
import frc.lib.util.PhoenixUtil;

/** Real hardware IO for a single TalonFX motor mechanism with batched CAN refresh. */
public class MechanismIOTalonFX implements MechanismIO {

  private final TalonFX m_motor;

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
  private final BaseStatusSignal[] m_allSignals;

  public MechanismIOTalonFX(MechanismConfig config) {
    m_motor = new TalonFX(config.motorId, config.canBus);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    // PID slot 0
    talonConfig.Slot0.kP = config.pidGains.kP;
    talonConfig.Slot0.kI = config.pidGains.kI;
    talonConfig.Slot0.kD = config.pidGains.kD;
    talonConfig.Slot0.kS = config.pidGains.kS;
    talonConfig.Slot0.kV = config.pidGains.kV;
    talonConfig.Slot0.kA = config.pidGains.kA;

    // Motor output
    talonConfig.MotorOutput.Inverted =
        config.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    talonConfig.MotorOutput.NeutralMode =
        config.neutralModeBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // Current limits
    if (config.hasStatorCurrentLimit()) {
      talonConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(config.statorCurrentLimit));
      talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    }
    if (config.hasSupplyCurrentLimit()) {
      talonConfig.CurrentLimits.withSupplyCurrentLimit(Amps.of(config.supplyCurrentLimit));
      talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    // Voltage limits
    talonConfig.Voltage.withPeakForwardVoltage(Volts.of(config.peakVoltage));
    talonConfig.Voltage.withPeakReverseVoltage(Volts.of(-config.peakVoltage));

    // Software limits
    if (config.hasSoftLimits()) {
      talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = config.softLimitForward;
      talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = config.softLimitReverse;
    }

    // Motion Magic
    if (config.hasMotionMagic()) {
      talonConfig.MotionMagic.MotionMagicCruiseVelocity = config.motionMagicCruiseVelocity;
      talonConfig.MotionMagic.MotionMagicAcceleration = config.motionMagicAcceleration;
    }

    // Apply config with retry
    PhoenixUtil.tryUntilOk(() -> m_motor.getConfigurator().apply(talonConfig), 5);

    // Cache status signals for batch refresh
    m_position = m_motor.getPosition();
    m_velocity = m_motor.getVelocity();
    m_statorCurrent = m_motor.getStatorCurrent();
    m_supplyCurrent = m_motor.getSupplyCurrent();
    m_voltage = m_motor.getMotorVoltage();
    m_temp = m_motor.getDeviceTemp();

    m_allSignals =
        new BaseStatusSignal[] {
          m_position, m_velocity, m_statorCurrent, m_supplyCurrent, m_voltage, m_temp
        };
  }

  /** Returns the underlying TalonFX motor for control requests. */
  public TalonFX getMotor() {
    return m_motor;
  }

  @Override
  public void setPosition(double positionRotations) {
    m_motor.setControl(m_positionRequest.withPosition(positionRotations));
  }

  @Override
  public void setVelocity(double velocityRPS) {
    m_motor.setControl(m_velocityRequest.withVelocity(velocityRPS));
  }

  @Override
  public void setMotionMagicPosition(double positionRotations) {
    m_motor.setControl(m_motionMagicRequest.withPosition(positionRotations));
  }

  @Override
  public void stop() {
    m_motor.setControl(m_neutralRequest);
  }

  @Override
  public void resetPosition() {
    m_motor.setPosition(0);
  }

  @Override
  public void updateInputs(MechanismIOInputsAutoLogged inputs) {
    var status = BaseStatusSignal.refreshAll(m_allSignals);

    inputs.positionRotations = m_position.getValueAsDouble();
    inputs.velocityRPS = m_velocity.getValueAsDouble();
    inputs.statorCurrentA = m_statorCurrent.getValueAsDouble();
    inputs.supplyCurrentA = m_supplyCurrent.getValueAsDouble();
    inputs.appliedVoltage = m_voltage.getValueAsDouble();
    inputs.tempCelsius = m_temp.getValueAsDouble();
    inputs.motorConnected = status.isOK();
  }
}
