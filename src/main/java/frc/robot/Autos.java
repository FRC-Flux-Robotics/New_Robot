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
import java.util.Set;

/** Auto routines for testing and competition. All speeds standardized to TEST_SPEED. */
public final class Autos {

  /** Shared test speed — above deadband (~0.48 m/s), slow enough for operator to e-stop. */
  private static final double TEST_SPEED = 1.0; // m/s

  private static final double TEST_ACCEL = 1.0; // m/s²
  private static final double TEST_ANG_VEL = Math.PI * 0.2; // rad/s
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
    return Commands.run(() -> drive.drive(TEST_SPEED, 0, 0, false, 0.02), drive)
        .withTimeout(10.0)
        .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, false, 0.02), drive));
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

  /** Back up, deploy intake, follow "Hub to Depot" path, return to hub and shoot. */
  public static Command hubToDepot(DriveInterface drive) {
    double distanceMeters = 0.1524; // 6 inches
    Pose2d[] startPose = new Pose2d[1];

    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Hub to Depot");
      return Commands.sequence(
          // Back up 6 inches to path start position
          Commands.runOnce(() -> startPose[0] = drive.getPose()),
          Commands.run(() -> drive.drive(-TEST_SPEED, 0, 0, false, 0.02), drive)
              .until(
                  () ->
                      drive.getPose().getTranslation().getDistance(startPose[0].getTranslation())
                          >= distanceMeters)
              .withTimeout(3.0),
          Commands.runOnce(() -> drive.drive(0, 0, 0, false, 0.02), drive),
          // Deploy intake
          NamedCommands.getCommand("deployIntake"),
          // Follow path (collect fuel, return to hub)
          SafeAutoBuilder.wrap(path, drive),
          // Shoot at hub
          Commands.deadline(
              Commands.sequence(
                  Commands.waitSeconds(1.0), NamedCommands.getCommand("feed").withTimeout(10.0)),
              NamedCommands.getCommand("spinUpShooter")),
          NamedCommands.getCommand("stopAll"));
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to load path: Hub to Depot", e.getStackTrace());
      return Commands.none();
    }
  }

  /** Drive to depot (W1 only), start intake. Test auto. */
  public static Command hubToW1(DriveInterface drive) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Hub to W1");
      return Commands.sequence(
          SafeAutoBuilder.wrap(path, drive),
          // Explicit startIntake (not via event marker) to isolate the issue
          NamedCommands.getCommand("startIntake"),
          Commands.waitSeconds(3.0),
          NamedCommands.getCommand("stopIntake"),
          NamedCommands.getCommand("stopAll"));
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to load path: Hub to W1", e.getStackTrace());
      return Commands.none();
    }
  }

  /** Back up 6 inches and deploy intake. Uses odometry for accurate distance. */
  public static Command backAndDeploy(DriveInterface drive) {
    double distanceMeters = 0.1524; // 6 inches
    Pose2d[] startPose = new Pose2d[1];

    return Commands.sequence(
        Commands.runOnce(() -> startPose[0] = drive.getPose()),
        Commands.run(() -> drive.drive(-TEST_SPEED, 0, 0, false, 0.02), drive)
            .until(
                () ->
                    drive.getPose().getTranslation().getDistance(startPose[0].getTranslation())
                        >= distanceMeters)
            .withTimeout(3.0),
        Commands.runOnce(() -> drive.drive(0, 0, 0, false, 0.02), drive),
        NamedCommands.getCommand("deployIntake"));
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

  /**
   * Pathfind to hub from anywhere on our alliance half, then shoot. Deferred so alliance is
   * resolved at runtime, not at construction.
   */
  public static Command hub(DriveInterface drive) {
    return Commands.defer(
        () -> {
          Pose2d target = FieldPositions.resolve("HUB");

          return Commands.sequence(
              AutoBuilder.pathfindToPose(target, TEST_CONSTRAINTS),
              Commands.runOnce(() -> drive.drive(0, 0, 0, true, 0.02), drive),
              // Spin up for 1s, then feed while keeping shooter running
              Commands.deadline(
                  Commands.sequence(
                      Commands.waitSeconds(1.0),
                      NamedCommands.getCommand("feed").withTimeout(10.0)),
                  NamedCommands.getCommand("spinUpShooter")),
              NamedCommands.getCommand("stopAll"));
        },
        Set.of());
  }

  /** Hub to Depot (collect + shoot), then Collect path. */
  public static Command hubToDepotThenCollect(DriveInterface drive) {
    return Commands.sequence(hubToDepot(drive), collect(drive));
  }

  /**
   * Full cycle: back up, deploy intake, collect fuel via Hub to Depot path, shoot at hub, then
   * drive to neutral zone via Collect path.
   */
  public static Command fullCycle(DriveInterface drive) {
    try {
      PathPlannerPath hubToDepotPath = PathPlannerPath.fromPathFile("Hub to Depot");
      PathPlannerPath collectPath = PathPlannerPath.fromPathFile("Collect");
      return Commands.sequence(
          // Back up and deploy intake
          backAndDeploy(drive),
          // Follow Hub to Depot path (collect fuel, return to hub)
          SafeAutoBuilder.wrap(hubToDepotPath, drive),
          // Shoot at hub
          Commands.deadline(
              Commands.sequence(
                  Commands.waitSeconds(1.0), NamedCommands.getCommand("feed").withTimeout(10.0)),
              NamedCommands.getCommand("spinUpShooter")),
          NamedCommands.getCommand("stopAll"),
          // Drive to neutral zone
          SafeAutoBuilder.wrap(collectPath, drive));
    } catch (Exception e) {
      edu.wpi.first.wpilibj.DriverStation.reportError(
          "Failed to load paths for full cycle", e.getStackTrace());
      return Commands.none();
    }
  }

  /** Drive forward, rotate 180 deg, drive back, stop. */
  public static Command forwardTurnBack(DriveInterface drive) {
    double driveTime = 2.0 / TEST_SPEED; // 2m at TEST_SPEED
    // Capture start heading at runtime, turn 180° relative to it
    Rotation2d[] turnTarget = new Rotation2d[1];
    return Commands.sequence(
        // Leg 1: drive forward (robot-centric)
        Commands.run(() -> drive.drive(TEST_SPEED, 0, 0, false, 0.02), drive)
            .withTimeout(driveTime),
        Commands.runOnce(() -> drive.drive(0, 0, 0, false, 0.02), drive),
        // Compute 180° from current heading
        Commands.runOnce(
            () -> turnTarget[0] = drive.getHeading().plus(Rotation2d.fromDegrees(180))),
        // Turn 180° using closed-loop heading control
        Commands.run(() -> drive.driveFieldCentricFacingAngle(0, 0, turnTarget[0], 0.02), drive)
            .until(() -> Math.abs(drive.getHeading().minus(turnTarget[0]).getDegrees()) < 5.0)
            .withTimeout(10.0),
        // Leg 2: drive forward (robot-centric, now facing back)
        Commands.run(() -> drive.drive(TEST_SPEED, 0, 0, false, 0.02), drive)
            .withTimeout(driveTime),
        Commands.runOnce(() -> drive.drive(0, 0, 0, false, 0.02), drive));
  }
}
