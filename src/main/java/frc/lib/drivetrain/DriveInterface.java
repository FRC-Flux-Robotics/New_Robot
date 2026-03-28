package frc.lib.drivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Clean API for drivetrain consumers. Program against this, not the concrete implementation. */
public interface DriveInterface extends Subsystem {
  // Movement
  void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds);

  void updateOdometry();

  // State
  Pose2d getPose();

  ChassisSpeeds getVelocity();

  Rotation2d getHeading();

  DriveState getDriveState();

  // Pose management
  void resetHeading();

  void resetPose(Pose2d pose);

  void setOperatorForward(Rotation2d forward);

  void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs);

  /** Drive with translation control while automatically rotating to face targetAngle. */
  default void driveFieldCentricFacingAngle(
      double xSpeed, double ySpeed, Rotation2d targetAngle, double periodSeconds) {
    throw new UnsupportedOperationException("FieldCentricFacingAngle not supported");
  }

  // Control
  /** Reconfigure request deadbands as a fraction of max speed (e.g. 0.1 = 10%). */
  default void setDeadband(double translationFraction, double rotationFraction) {}

  /** Sets motors to idle (no output). Use when robot is disabled. */
  default void setIdle() {}

  /** Sets wheels to X-pattern brake. Use to hold position. */
  default void setBrake() {}

  /** Follow a pre-built PathPlanner path by name. Returns Commands.none() if path fails to load. */
  default Command followPathCommand(String pathName) {
    throw new UnsupportedOperationException("followPathCommand not supported");
  }

  // Config
  DrivetrainConfig getConfig();

  double getMaxSpeed();

  double getMaxAngularSpeed();
}
