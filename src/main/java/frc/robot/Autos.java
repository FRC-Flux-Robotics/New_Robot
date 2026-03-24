package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.drivetrain.DriveInterface;

/** Simple time-based auto routines. No PathPlanner dependency. */
public final class Autos {

  private Autos() {}

  /** Do nothing. */
  public static Command none() {
    return Commands.none();
  }

  /** Drive to nearest AprilTag using vision, with 10s timeout. */
  public static Command driveToNearestTag(Vision vision, DriveInterface drive) {
    return new DriveToTag(vision, drive).withTimeout(10.0);
  }

  /** Drive forward at 1 m/s for 2 seconds, then stop. */
  public static Command driveForward(DriveInterface drive) {
    return Commands.run(
            () -> drive.drive(1.0, 0, 0, true, 0.02), drive)
        .withTimeout(2.0)
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive));
  }

  /** Drive forward 2s, rotate 90 deg for 1s, drive forward 1s, stop. */
  public static Command forwardTurnBack(DriveInterface drive) {
    return Commands.run(
            () -> drive.drive(1.0, 0, 0, true, 0.02), drive)
        .withTimeout(2.0)
        .andThen(
            Commands.run(
                    () -> drive.drive(0, 0, Math.PI / 2.0, true, 0.02), drive)
                .withTimeout(1.0))
        .andThen(
            Commands.run(
                    () -> drive.drive(1.0, 0, 0, true, 0.02), drive)
                .withTimeout(1.0))
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive));
  }
}
