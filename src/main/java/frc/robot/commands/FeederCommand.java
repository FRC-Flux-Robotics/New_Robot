package frc.robot.commands;

import frc.robot.FuelConstants;
import frc.robot.FuelConstants.IndexerConstants;
import frc.robot.subsystems.VelocityMech;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to run the feeder wheels. */
public class FeederCommand extends Command {
  private final VelocityMech feeder;
  private final int direction;

  public FeederCommand(VelocityMech feeder, double speed, int dir) {
    this.feeder = feeder;
    direction = dir;
    addRequirements(feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = IndexerConstants.FeederSpeed;
    if (direction != FuelConstants.Forward)
      speed = -speed;
    feeder.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
