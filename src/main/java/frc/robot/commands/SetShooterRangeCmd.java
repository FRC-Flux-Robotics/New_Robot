package frc.robot.commands;

import frc.robot.RangeTable;
import frc.robot.subsystems.PositionMech;
import frc.robot.subsystems.VelocityMech2;

import edu.wpi.first.wpilibj2.command.Command;

/** A command to set shooter speed and hood position from a range preset. */
public class SetShooterRangeCmd extends Command {
  private final VelocityMech2 shooter;
  private final PositionMech hood;
  private final RangeTable rangeTable;
  private final int range;

  public SetShooterRangeCmd(VelocityMech2 shooter, PositionMech hood, RangeTable rangeTable, int range) {
    this.shooter = shooter;
    this.hood = hood;
    this.range = range;
    this.rangeTable = rangeTable;

    addRequirements(shooter, hood);
  }

  @Override
  public void initialize() {
    double speed = rangeTable.getSpeedPreset(range);
    double hoodPos = rangeTable.getElevationPreset(range);
    System.out.println("SetShootCmd: " + speed + " / " + hoodPos);

    shooter.setTargetSpeed(speed);
    shooter.setSpeed(speed);
    hood.run(hoodPos);
  }

    @Override
    public void execute()
    {
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
