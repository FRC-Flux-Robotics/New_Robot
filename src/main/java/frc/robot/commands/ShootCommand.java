package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.mechanism.VelocityMechanism;
import java.util.function.DoubleSupplier;

/**
 * Runs the shooter at a supplied speed. Stays active until cancelled (works with toggleOnTrue).
 * Stops the shooter on end.
 */
public class ShootCommand extends Command {
  private final VelocityMechanism m_shooter;
  private final DoubleSupplier m_speedRPS;

  /**
   * @param shooter the shooter mechanism
   * @param speedRPS supplier returning speed in RPS (sign determines direction)
   */
  public ShootCommand(VelocityMechanism shooter, DoubleSupplier speedRPS) {
    m_shooter = shooter;
    m_speedRPS = speedRPS;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setVelocity(m_speedRPS.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
