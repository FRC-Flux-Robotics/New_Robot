package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import org.junit.jupiter.api.Test;

class DriveInterfaceTest {

  /** Stub implementation to verify the interface is usable. */
  private static class StubDrive implements DriveInterface {
    boolean driveCalled = false;
    boolean odometryCalled = false;
    boolean headingReset = false;

    @Override
    public void drive(double xSpeed, double ySpeed, double rot,
                      boolean fieldRelative, double periodSeconds) {
      driveCalled = true;
    }

    @Override
    public void updateOdometry() {
      odometryCalled = true;
    }

    @Override
    public Pose2d getPose() {
      return new Pose2d();
    }

    @Override
    public ChassisSpeeds getVelocity() {
      return new ChassisSpeeds();
    }

    @Override
    public Rotation2d getHeading() {
      return new Rotation2d();
    }

    @Override
    public DriveState getDriveState() {
      return new DriveState(getPose(), getVelocity(), getHeading());
    }

    @Override
    public void resetHeading() {
      headingReset = true;
    }

    @Override
    public void resetPose(Pose2d pose) {
      // no-op stub
    }

    @Override
    public void setOperatorForward(Rotation2d forward) {
      // no-op stub
    }

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
      // no-op stub
    }

    @Override
    public DrivetrainConfig getConfig() {
      return null;
    }

    @Override
    public double getMaxSpeed() {
      return 4.5;
    }

    @Override
    public double getMaxAngularSpeed() {
      return Math.PI;
    }
  }

  @Test
  void stubImplementsAllMethods() {
    DriveInterface drive = new StubDrive();

    drive.drive(1.0, 0.5, 0.1, true, 0.02);
    drive.updateOdometry();
    drive.resetHeading();

    assertNotNull(drive.getPose());
    assertNotNull(drive.getVelocity());
    assertNotNull(drive.getHeading());
    assertNotNull(drive.getDriveState());
    assertEquals(4.5, drive.getMaxSpeed());
    assertEquals(Math.PI, drive.getMaxAngularSpeed());
  }

  @Test
  void driveCallsAreTracked() {
    StubDrive stub = new StubDrive();

    assertFalse(stub.driveCalled);
    stub.drive(0, 0, 0, false, 0.02);
    assertTrue(stub.driveCalled);
  }

  @Test
  void resetHeadingCallIsTracked() {
    StubDrive stub = new StubDrive();

    assertFalse(stub.headingReset);
    stub.resetHeading();
    assertTrue(stub.headingReset);
  }

  @Test
  void facingAngleThrowsOnStub() {
    StubDrive stub = new StubDrive();
    assertThrows(UnsupportedOperationException.class,
        () -> stub.driveFieldCentricFacingAngle(0, 0, new Rotation2d(), 0.02));
  }

  @Test
  void followPathCommandThrowsOnStub() {
    StubDrive stub = new StubDrive();
    assertThrows(UnsupportedOperationException.class,
        () -> stub.followPathCommand("TestPath"));
  }

  @Test
  void getStateReturnsConsistentSnapshot() {
    DriveInterface drive = new StubDrive();
    DriveState state = drive.getDriveState();

    assertEquals(drive.getPose(), state.pose);
    assertEquals(drive.getVelocity(), state.velocity);
    assertEquals(drive.getHeading(), state.heading);
  }
}
