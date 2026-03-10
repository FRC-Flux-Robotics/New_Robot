package frc.robot;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivetrain.DriveInterface;
import java.util.List;

public final class Autos {

  private Autos() {}

  public static Command driveForward(DriveInterface drive) {
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                List.of(new Pose2d(0, 0, Rotation2d.kZero), new Pose2d(2, 0, Rotation2d.kZero))),
            new PathConstraints(2.0, 1.5, Math.PI, Math.PI),
            null,
            new GoalEndState(0, Rotation2d.kZero));
    return drive.followPath(path);
  }

  /**
   * Drives 2m forward, turns 180 degrees, then drives 1m (back toward start). Ends facing away from
   * starting direction, 1m from the starting position.
   */
  public static Command forwardTurnBack(DriveInterface drive) {
    PathConstraints constraints = new PathConstraints(2.0, 1.5, Math.PI, Math.PI);

    // Leg 1: drive 2m forward, arrive stopped facing forward
    PathPlannerPath forward =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                List.of(new Pose2d(0, 0, Rotation2d.kZero), new Pose2d(2, 0, Rotation2d.kZero))),
            constraints,
            null,
            new GoalEndState(0, Rotation2d.kZero));

    // Leg 2: from 2m mark, drive 1m back toward start (now facing 180 deg)
    PathPlannerPath back =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                List.of(
                    new Pose2d(2, 0, Rotation2d.k180deg), new Pose2d(1, 0, Rotation2d.k180deg))),
            constraints,
            null,
            new GoalEndState(0, Rotation2d.k180deg));

    return drive.followPath(forward).andThen(drive.followPath(back));
  }
}
