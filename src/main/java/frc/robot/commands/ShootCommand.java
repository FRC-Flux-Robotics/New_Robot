package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.mechanism.VelocityMechanism;
import java.util.function.DoubleSupplier;

/**
 * Toggles the shooter on/off. If spinning, stops it. If stopped, starts at supplied speed. Finishes
 * immediately after toggling.
 */
public class ShootCommand extends Command {
  private final VelocityMechanism m_shooter;
  private final DoubleSupplier m_speedRPM;

  /**
   * @param shooter the shooter mechanism
   * @param speedRPM supplier returning speed in RPM (sign determines direction)
   */
  public ShootCommand(VelocityMechanism shooter, DoubleSupplier speedRPM) {
    m_shooter = shooter;
    m_speedRPM = speedRPM;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    if (m_shooter.getTargetVelocity() != 0) {
      m_shooter.stop();
    } else {
      m_shooter.setVelocity(m_speedRPM.getAsDouble());
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
