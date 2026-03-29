package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.mechanism.VelocityMechanism;
import java.util.function.DoubleSupplier;

/**
 * Runs a velocity mechanism at a supplied speed (RPM). Sign of the speed determines direction.
 * Stops on end. Finishes after timeout if set, otherwise runs until cancelled.
 */
public class VelocityCmd extends Command {
  private final VelocityMechanism m_mechanism;
  private final DoubleSupplier m_speedRPM;
  private final double m_timeoutSeconds;
  private final Timer m_timer = new Timer();

  /**
   * @param mechanism the velocity mechanism to control
   * @param speedRPM supplier returning speed in RPM (negative = reverse)
   * @param timeoutSeconds seconds before auto-finish (0 = no timeout)
   */
  public VelocityCmd(VelocityMechanism mechanism, DoubleSupplier speedRPM, double timeoutSeconds) {
    m_mechanism = mechanism;
    m_speedRPM = speedRPM;
    m_timeoutSeconds = timeoutSeconds;
    addRequirements(mechanism);
  }

  /** No-timeout constructor — runs until cancelled. */
  public VelocityCmd(VelocityMechanism mechanism, DoubleSupplier speedRPM) {
    this(mechanism, speedRPM, 0);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_mechanism.setVelocity(m_speedRPM.getAsDouble());
  }

  @Override
  public void execute() {
    m_mechanism.setVelocity(m_speedRPM.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_mechanism.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timeoutSeconds > 0 && m_timer.hasElapsed(m_timeoutSeconds);
  }
}
