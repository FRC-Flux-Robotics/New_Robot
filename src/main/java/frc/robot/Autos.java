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
        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                        List.of(
                                new Pose2d(0, 0, Rotation2d.kZero),
                                new Pose2d(2, 0, Rotation2d.kZero))),
                new PathConstraints(2.0, 1.5, Math.PI, Math.PI),
                null,
                new GoalEndState(0, Rotation2d.kZero));
        return drive.followPath(path);
    }
}
