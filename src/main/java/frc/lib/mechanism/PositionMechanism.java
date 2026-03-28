package frc.lib.mechanism;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Position-controlled mechanism subsystem using MechanismIO with AdvantageKit logging. */
public class PositionMechanism extends SubsystemBase {

  private final MechanismIO m_io;
  private final MechanismConfig m_config;
  private final MechanismIOInputsAutoLogged m_inputs = new MechanismIOInputsAutoLogged();

  private double m_targetPosition = 0.0;
  private final double m_tolerance;

  public PositionMechanism(MechanismIO io, MechanismConfig config) {
    if (config.controlMode != ControlMode.POSITION) {
      throw new IllegalArgumentException(
          "PositionMechanism requires POSITION control mode, got " + config.controlMode);
    }
    m_io = io;
    m_config = config;
    m_tolerance = config.jogStep > 0 ? config.jogStep : 1.0;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs(m_config.name, m_inputs);
    Logger.recordOutput(m_config.name + "/TargetPosition", m_targetPosition);
    Logger.recordOutput(m_config.name + "/AtTarget", atTarget());
  }

  /** Command the mechanism to a position in rotations. Uses MotionMagic if configured. */
  public void setPosition(double positionRotations) {
    m_targetPosition = positionRotations;
    if (m_config.hasMotionMagic()) {
      m_io.setMotionMagicPosition(positionRotations);
    } else {
      m_io.setPosition(positionRotations);
    }
  }

  /** Jog position up by config jogStep from current position. */
  public void jogUp() {
    setPosition(getPosition() + m_config.jogStep);
  }

  /** Jog position down by config jogStep from current position. */
  public void jogDown() {
    setPosition(getPosition() - m_config.jogStep);
  }

  /** Command motor to neutral output. */
  public void stop() {
    m_io.stop();
  }

  /** Whether current position is within tolerance of target. */
  public boolean atTarget() {
    return Math.abs(getPosition() - m_targetPosition) <= m_tolerance;
  }

  /** Current position in rotations (from last IO update). */
  public double getPosition() {
    return m_inputs.positionRotations;
  }

  /** Reset encoder to zero. */
  public void resetPosition() {
    m_io.resetPosition();
  }

  /** Get the current target position. */
  public double getTargetPosition() {
    return m_targetPosition;
  }
}
