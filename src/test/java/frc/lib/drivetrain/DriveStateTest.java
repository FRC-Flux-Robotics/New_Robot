package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.junit.jupiter.api.Test;

class DriveStateTest {

  @Test
  void storesAllFields() {
    Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(45));
    ChassisSpeeds velocity = new ChassisSpeeds(1.5, 0.5, 0.1);
    Rotation2d heading = Rotation2d.fromDegrees(90);

    DriveState state = new DriveState(pose, velocity, heading);

    assertEquals(pose, state.pose);
    assertEquals(velocity, state.velocity);
    assertEquals(heading, state.heading);
  }

  @Test
  void storesZeroState() {
    Pose2d pose = new Pose2d();
    ChassisSpeeds velocity = new ChassisSpeeds();
    Rotation2d heading = new Rotation2d();

    DriveState state = new DriveState(pose, velocity, heading);

    assertEquals(0.0, state.pose.getX());
    assertEquals(0.0, state.pose.getY());
    assertEquals(0.0, state.heading.getDegrees());
  }
}
