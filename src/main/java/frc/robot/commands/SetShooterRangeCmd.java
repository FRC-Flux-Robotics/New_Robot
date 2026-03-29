package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.robot.MechanismTuning;

/** Sets shooter speed and hood position from a tunable range preset. Finishes immediately. */
public class SetShooterRangeCmd extends Command {
  private final VelocityMechanism m_shooter;
  private final PositionMechanism m_hood;
  private final int m_rangeIndex;

  public SetShooterRangeCmd(VelocityMechanism shooter, PositionMechanism hood, int rangeIndex) {
    m_shooter = shooter;
    m_hood = hood;
    m_rangeIndex = rangeIndex;
    addRequirements(shooter, hood);
  }

  @Override
  public void initialize() {
    double speedRPS;
    double elevation;
    switch (m_rangeIndex) {
      case 0:
        speedRPS = MechanismTuning.speedShort();
        elevation = MechanismTuning.hoodShort();
        break;
      case 1:
        speedRPS = MechanismTuning.speedMid();
        elevation = MechanismTuning.hoodMid();
        break;
      default:
        speedRPS = MechanismTuning.speedLong();
        elevation = MechanismTuning.hoodLong();
        break;
    }
    m_shooter.setVelocity(speedRPS);
    m_hood.setPosition(elevation);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
