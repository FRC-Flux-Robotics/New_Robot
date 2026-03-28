package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.robot.RangeTable;

/** Sets shooter speed and hood position from a range preset. Finishes immediately. */
public class SetShooterRangeCmd extends Command {
  private final VelocityMechanism m_shooter;
  private final PositionMechanism m_hood;
  private final RangeTable m_rangeTable;
  private final int m_rangeIndex;

  public SetShooterRangeCmd(
      VelocityMechanism shooter, PositionMechanism hood, RangeTable rangeTable, int rangeIndex) {
    m_shooter = shooter;
    m_hood = hood;
    m_rangeTable = rangeTable;
    m_rangeIndex = rangeIndex;
    addRequirements(shooter, hood);
  }

  @Override
  public void initialize() {
    double speedRPS = m_rangeTable.getSpeedPreset(m_rangeIndex);
    double elevation = m_rangeTable.getElevationPreset(m_rangeIndex);
    m_shooter.setVelocity(speedRPS);
    m_hood.setPosition(elevation);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
