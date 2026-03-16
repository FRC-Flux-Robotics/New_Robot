package frc.lib.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Clean API for drivetrain consumers. Program against this, not the concrete implementation. */
public interface DriveInterface {
  // Movement
  void drive(double xSpeed, double ySpeed, double rot,
             boolean fieldRelative, double periodSeconds);
  void updateOdometry();

  // State
  Pose2d getPose();
  ChassisSpeeds getVelocity();
  Rotation2d getHeading();
  DriveState getState();

  // Pose management
  void resetHeading();

  // Config
  DrivetrainConfig getConfig();
  double getMaxSpeed();
  double getMaxAngularSpeed();
}
