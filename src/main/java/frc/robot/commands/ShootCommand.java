package frc.robot.commands;

import frc.lib.mechanism.VelocityMechanism;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Toggles the shooter on/off. If spinning, stops it. If stopped, starts at supplied speed.
 * Finishes immediately after toggling.
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
    if (m_shooter.getTargetVelocity() != 0) {
      m_shooter.stop();
    } else {
      m_shooter.setVelocity(m_speedRPS.getAsDouble());
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
