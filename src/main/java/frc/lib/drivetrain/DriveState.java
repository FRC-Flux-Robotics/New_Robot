package frc.lib.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Snapshot of drivetrain state at a point in time. */
public class DriveState {
  public final Pose2d pose;
  public final ChassisSpeeds velocity;
  public final Rotation2d heading;

  public DriveState(Pose2d pose, ChassisSpeeds velocity, Rotation2d heading) {
    this.pose = pose;
    this.velocity = velocity;
    this.heading = heading;
  }
}
