package frc.robot.commands;

import frc.robot.FuelConstants;
import frc.robot.FuelConstants.IndexerConstants;
import frc.robot.subsystems.VelocityMech;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to run the indexer wheels. */
public class IndexerCommand extends Command {
  private final VelocityMech indexer;
  private final int direction;

  public IndexerCommand(VelocityMech indexer, double speed, int dir) {
    this.indexer = indexer;
    direction = dir;
    indexer.setTargetSpeed(speed);
    addRequirements(indexer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double speed = IndexerConstants.Speed;
    if (direction != FuelConstants.Forward)
      speed = -speed;
    indexer.setSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
