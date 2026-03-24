package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  /** PathPlanner test: pathfind to 3 poses in sequence with waits between. */
  public static Command pathPlannerTest(DriveInterface drive) {
    PathConstraints constraints = new PathConstraints(2.0, 1.5, Math.PI, Math.PI, 12.0);
    return Commands.sequence(
        AutoBuilder.pathfindToPose(new Pose2d(2, 0, Rotation2d.kZero), constraints),
        Commands.waitSeconds(1.0),
        AutoBuilder.pathfindToPose(new Pose2d(2, 2, Rotation2d.fromDegrees(90)), constraints),
        Commands.waitSeconds(1.0),
        AutoBuilder.pathfindToPose(new Pose2d(0, 0, Rotation2d.kZero), constraints));
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
