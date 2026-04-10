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

/** Auto routines for testing and competition. All speeds standardized to TEST_SPEED. */
public final class Autos {

  /** Shared test speed — slow enough for operator to e-stop. */
  private static final double TEST_SPEED = 0.2; // m/s

  private static final double TEST_ACCEL = 0.15; // m/s²
  private static final double TEST_ANG_VEL = Math.PI * 0.1; // rad/s
  private static final double TEST_ANG_ACCEL = Math.PI * 0.1; // rad/s²

  private static final PathConstraints TEST_CONSTRAINTS =
      new PathConstraints(TEST_SPEED, TEST_ACCEL, TEST_ANG_VEL, TEST_ANG_ACCEL, 12.0);

  private Autos() {}

  /** Do nothing. */
  public static Command none() {
    return Commands.none();
  }

  /** Drive to nearest AprilTag using vision, with 10s timeout. */
  public static Command driveToNearestTag(Vision vision, DriveInterface drive) {
    return new DriveToTag(vision, drive).withTimeout(10.0);
  }

  /** Drive forward at TEST_SPEED for 10 seconds, then stop. */
  public static Command driveForward(DriveInterface drive) {
    return Commands.run(() -> drive.drive(TEST_SPEED, 0, 0, true, 0.02), drive)
        .withTimeout(10.0)
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive));
  }

  /** Precision test: drive a 2m square and return to origin. Logs position error. */
  public static Command precisionSquare(DriveInterface drive) {
    double sideTime = 2.0 / TEST_SPEED; // 2m at TEST_SPEED
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
        Commands.run(() -> drive.drive(TEST_SPEED, 0, 0, true, 0.02), drive).withTimeout(sideTime),
        Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
        Commands.waitSeconds(settleTime),
        // Side 2: +Y
        Commands.run(() -> drive.drive(0, TEST_SPEED, 0, true, 0.02), drive).withTimeout(sideTime),
        Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
        Commands.waitSeconds(settleTime),
        // Side 3: -X
        Commands.run(() -> drive.drive(-TEST_SPEED, 0, 0, true, 0.02), drive).withTimeout(sideTime),
        Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
        Commands.waitSeconds(settleTime),
        // Side 4: -Y
        Commands.run(() -> drive.drive(0, -TEST_SPEED, 0, true, 0.02), drive).withTimeout(sideTime),
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

  /** Follow "Hub to Depot" path. Robot must be pre-positioned at hub with pose reset. */
  public static Command hubToDepot(DriveInterface drive) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Hub to Depot");
      return SafeAutoBuilder.wrap(path, drive);
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to load path: Hub to Depot", e.getStackTrace());
      return Commands.none();
    }
  }

  /** Follow "Collect" path. Robot must be pre-positioned at hub with pose reset. */
  public static Command collect(DriveInterface drive) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Collect");
      return SafeAutoBuilder.wrap(path, drive);
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to load path: Collect", e.getStackTrace());
      return Commands.none();
    }
  }

  /** Pathfind to near Hub (outside reef obstacle), brake, then shoot for 10 seconds. */
  public static Command hub(DriveInterface drive) {
    Pose2d hubApproach = new Pose2d(3.25, 4.05, Rotation2d.kZero);
    return Commands.sequence(
        Commands.deadline(
            AutoBuilder.pathfindToPose(hubApproach, TEST_CONSTRAINTS),
            NamedCommands.getCommand("spinUpShooter")),
        Commands.runOnce(() -> drive.setBrake(), drive),
        Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
        Commands.deadline(
            NamedCommands.getCommand("feed").withTimeout(10.0),
            NamedCommands.getCommand("spinUpShooter")),
        NamedCommands.getCommand("stopAll"));
  }

  /** Follow "To Hub" path, brake, then shoot for 10 seconds. Pre-position robot at path start. */
  public static Command hubFollow(DriveInterface drive) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("To Hub");
      return Commands.sequence(
          Commands.deadline(
              SafeAutoBuilder.wrap(path, drive), NamedCommands.getCommand("spinUpShooter")),
          Commands.runOnce(() -> drive.setBrake(), drive),
          Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
          Commands.deadline(
              NamedCommands.getCommand("feed").withTimeout(10.0),
              NamedCommands.getCommand("spinUpShooter")),
          NamedCommands.getCommand("stopAll"));
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to load path: To Hub", e.getStackTrace());
      return Commands.none();
    }
  }

  /** Drive forward, rotate 180 deg, drive back, stop. */
  public static Command forwardTurnBack(DriveInterface drive) {
    double driveTime = 2.0 / TEST_SPEED; // 2m at TEST_SPEED
    return Commands.run(() -> drive.drive(TEST_SPEED, 0, 0, true, 0.02), drive)
        .withTimeout(driveTime)
        .andThen(
            Commands.run(() -> drive.drive(0, 0, TEST_ANG_VEL, true, 0.02), drive)
                .withTimeout(Math.PI / TEST_ANG_VEL))
        .andThen(
            Commands.run(() -> drive.drive(TEST_SPEED, 0, 0, false, 0.02), drive)
                .withTimeout(driveTime))
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive));
  }
}
