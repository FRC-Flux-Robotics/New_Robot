package frc.lib.mechanism;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Velocity-controlled mechanism subsystem using MechanismIO with AdvantageKit logging. */
public class VelocityMechanism extends SubsystemBase {

  private final MechanismIO m_io;
  private final MechanismConfig m_config;
  private final MechanismIOInputsAutoLogged m_inputs = new MechanismIOInputsAutoLogged();

  private double m_targetRPM = 0.0;
  private final double m_tolerance;

  public VelocityMechanism(MechanismIO io, MechanismConfig config) {
    if (config.controlMode != ControlMode.VELOCITY) {
      throw new IllegalArgumentException(
          "VelocityMechanism requires VELOCITY control mode, got " + config.controlMode);
    }
    m_io = io;
    m_config = config;
    m_tolerance = config.jogStep > 0 ? config.jogStep : 5.0;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs(m_config.name, m_inputs);
    Logger.recordOutput(m_config.name + "/TargetRPM", m_targetRPM);
    Logger.recordOutput(m_config.name + "/AtTarget", atTarget());
  }

  /** Command the mechanism to a velocity in RPM. */
  public void setVelocity(double rpm) {
    m_targetRPM = rpm;
    m_io.setVelocity(rpm / 60.0); // TalonFX expects RPS
  }

  /** Command motor to neutral output and reset target to zero. */
  public void stop() {
    m_targetRPM = 0.0;
    m_io.stop();
  }

  /** Whether current velocity is within tolerance of target. */
  public boolean atTarget() {
    return Math.abs(getVelocity() - m_targetRPM) <= m_tolerance;
  }

  /** Current velocity in RPM (from last IO update). */
  public double getVelocity() {
    return m_inputs.velocityRPM;
  }

  /** Get the current target velocity in RPM. */
  public double getTargetVelocity() {
    return m_targetRPM;
  }
}
