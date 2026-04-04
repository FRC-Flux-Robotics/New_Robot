package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    return Commands.run(() -> drive.drive(1.0, 0, 0, true, 0.02), drive)
        .withTimeout(2.0)
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive));
  }

  /** PathPlanner test: pathfind to 3 poses in sequence with waits between. */
  public static Command pathPlannerTest(DriveInterface drive) {
    PathConstraints constraints = new PathConstraints(2.0, 1.5, Math.PI, Math.PI, 12.0);
    try {
      return Commands.sequence(
          AutoBuilder.pathfindToPose(new Pose2d(2, 0, Rotation2d.kZero), constraints),
          Commands.waitSeconds(1.0),
          AutoBuilder.pathfindToPose(new Pose2d(2, 2, Rotation2d.fromDegrees(90)), constraints),
          Commands.waitSeconds(1.0),
          AutoBuilder.pathfindToPose(new Pose2d(0, 0, Rotation2d.kZero), constraints));
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to create PathPlanner test auto", e.getStackTrace());
      return Commands.none();
    }
  }

  /** Precision test: drive a 2m square and return to origin. Logs position error. */
  public static Command precisionSquare(DriveInterface drive) {
    double speed = 1.0; // m/s
    double sideTime = 2.0; // seconds per side (2m at 1 m/s)
    double settleTime = 0.5; // pause between sides

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Pose2d startPose = drive.getPose();
              SmartDashboard.putNumber("PrecisionTest/StartX", startPose.getX());
              SmartDashboard.putNumber("PrecisionTest/StartY", startPose.getY());
              SmartDashboard.putNumber("PrecisionTest/ErrorMeters", 0.0);
              SmartDashboard.putNumber("PrecisionTest/ErrorDegrees", 0.0);
              SmartDashboard.putString("PrecisionTest/Status", "Running");
            }),
        // Side 1: +X
        Commands.run(() -> drive.drive(speed, 0, 0, true, 0.02), drive).withTimeout(sideTime),
        Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
        Commands.waitSeconds(settleTime),
        // Side 2: +Y
        Commands.run(() -> drive.drive(0, speed, 0, true, 0.02), drive).withTimeout(sideTime),
        Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
        Commands.waitSeconds(settleTime),
        // Side 3: -X
        Commands.run(() -> drive.drive(-speed, 0, 0, true, 0.02), drive).withTimeout(sideTime),
        Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
        Commands.waitSeconds(settleTime),
        // Side 4: -Y
        Commands.run(() -> drive.drive(0, -speed, 0, true, 0.02), drive).withTimeout(sideTime),
        Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
        Commands.runOnce(
            () -> {
              Pose2d endPose = drive.getPose();
              double startX = SmartDashboard.getNumber("PrecisionTest/StartX", 0);
              double startY = SmartDashboard.getNumber("PrecisionTest/StartY", 0);
              double errorMeters =
                  endPose.getTranslation().getDistance(new Translation2d(startX, startY));
              double errorDegrees = Math.abs(endPose.getRotation().getDegrees());
              SmartDashboard.putNumber("PrecisionTest/ErrorMeters", errorMeters);
              SmartDashboard.putNumber("PrecisionTest/ErrorDegrees", errorDegrees);
              SmartDashboard.putString("PrecisionTest/Status", "Done");
            }));
  }

  /** Pathfind to start of "Hub to Depot" path, then follow it. */
  public static Command hubToDepot(DriveInterface drive) {
    PathConstraints constraints = new PathConstraints(2.0, 1.5, Math.PI, Math.PI, 12.0);
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Hub to Depot");
      return AutoBuilder.pathfindThenFollowPath(path, constraints);
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to load path: Hub to Depot", e.getStackTrace());
      return Commands.none();
    }
  }

  /** Pathfind to start of "Collect" path, then follow it. */
  public static Command collect(DriveInterface drive) {
    PathConstraints constraints = new PathConstraints(2.0, 1.5, Math.PI, Math.PI, 12.0);
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Collect");
      return AutoBuilder.pathfindThenFollowPath(path, constraints);
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to load path: Collect", e.getStackTrace());
      return Commands.none();
    }
  }

  /** Pathfind to Hub from anywhere, brake, then shoot for 10 seconds. */
  public static Command hub(DriveInterface drive) {
    PathConstraints constraints = new PathConstraints(2.0, 1.5, Math.PI, Math.PI, 12.0);
    Pose2d hubPose = new Pose2d(3.5, 4.1, Rotation2d.kZero);
    return Commands.sequence(
        // Drive to hub while spinning up shooter so it's ready on arrival
        Commands.deadline(
            AutoBuilder.pathfindToPose(hubPose, constraints),
            NamedCommands.getCommand("spinUpShooter")),
        Commands.runOnce(() -> drive.setBrake(), drive),
        Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
        // Keep shooter running while feeding
        Commands.deadline(
            NamedCommands.getCommand("feed").withTimeout(10.0),
            NamedCommands.getCommand("spinUpShooter")),
        NamedCommands.getCommand("stopAll"));
  }

  /** Drive forward 2s, rotate 180 deg, drive back 2s, stop. */
  public static Command forwardTurnBack(DriveInterface drive) {
    return Commands.run(() -> drive.drive(1.0, 0, 0, true, 0.02), drive)
        .withTimeout(2.0)
        .andThen(Commands.run(() -> drive.drive(0, 0, Math.PI, true, 0.02), drive).withTimeout(1.0))
        .andThen(Commands.run(() -> drive.drive(1.0, 0, 0, false, 0.02), drive).withTimeout(2.0))
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive));
  }
}
