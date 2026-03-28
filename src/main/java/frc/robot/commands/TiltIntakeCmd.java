package frc.robot.commands;

import frc.robot.subsystems.PositionMech;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to tilt the intake. */
public class TiltIntakeCmd extends Command {
  private final PositionMech intake;
  private final int direction;

  public TiltIntakeCmd(PositionMech intake, int direction) {
    this.intake = intake;
    this.direction = direction;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.reset();
    intake.run1(-10);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
      intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
