package frc.lib.drivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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
  DriveState getDriveState();

  // Pose management
  void resetHeading();
  void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs);

  // Config
  DrivetrainConfig getConfig();
  double getMaxSpeed();
  double getMaxAngularSpeed();
}
