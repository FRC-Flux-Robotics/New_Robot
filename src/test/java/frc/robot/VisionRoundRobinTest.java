package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DriveInterface;
import frc.lib.drivetrain.DriveState;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.vision.VisionIO;
import frc.lib.vision.VisionIOInputsAutoLogged;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class VisionRoundRobinTest {

  @BeforeAll
  static void initHAL() {
    HAL.initialize(500, 0);
  }

  private static class CountingVisionIO implements VisionIO {
    int updateCount = 0;

    @Override
    public void updateInputs(VisionIOInputsAutoLogged inputs) {
      updateCount++;
      inputs.connected = true;
    }
  }

  private static class StubDrive implements DriveInterface {
    int visionMeasurementCount = 0;

    @Override
    public void drive(
        double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {}

    @Override
    public void updateOdometry() {}

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
    public void resetHeading() {}

    @Override
    public void resetPose(Pose2d pose) {}

    @Override
    public void setOperatorForward(Rotation2d forward) {}

    @Override
    public void addVisionMeasurement(
        Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
      visionMeasurementCount++;
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

  private static CameraConfig dummyCamera(String name) {
    return new CameraConfig(name, new Transform3d(new Translation3d(), new Rotation3d()));
  }

  private Vision createVision(CountingVisionIO[] ios) {
    CameraConfig[] cameras = new CameraConfig[ios.length];
    for (int i = 0; i < ios.length; i++) {
      cameras[i] = dummyCamera("cam" + i);
    }
    return new Vision(ios, cameras, new StubDrive());
  }

  @Test
  void roundRobinCyclesThroughCameras() {
    var io0 = new CountingVisionIO();
    var io1 = new CountingVisionIO();
    var io2 = new CountingVisionIO();
    Vision vision = createVision(new CountingVisionIO[] {io0, io1, io2});

    for (int i = 0; i < 6; i++) {
      vision.periodic();
    }

    assertEquals(2, io0.updateCount, "Camera 0 should be updated twice in 6 cycles");
    assertEquals(2, io1.updateCount, "Camera 1 should be updated twice in 6 cycles");
    assertEquals(2, io2.updateCount, "Camera 2 should be updated twice in 6 cycles");
  }

  @Test
  void roundRobinIndexWraps() {
    var ios =
        new CountingVisionIO[] {
          new CountingVisionIO(), new CountingVisionIO(), new CountingVisionIO()
        };
    Vision vision = createVision(ios);

    assertEquals(0, vision.getNextCameraIndex());
    vision.periodic(); // updates camera 0
    assertEquals(1, vision.getNextCameraIndex());
    vision.periodic(); // updates camera 1
    assertEquals(2, vision.getNextCameraIndex());
    vision.periodic(); // updates camera 2
    assertEquals(0, vision.getNextCameraIndex(), "Index should wrap back to 0");
  }

  @Test
  void singleCameraUpdatesEveryLoop() {
    var io = new CountingVisionIO();
    Vision vision = createVision(new CountingVisionIO[] {io});

    for (int i = 0; i < 5; i++) {
      vision.periodic();
    }

    assertEquals(5, io.updateCount, "Single camera should be updated every cycle");
  }

  @Test
  void onlyUpdatedCameraIsProcessed() {
    var ios =
        new CountingVisionIO[] {
          new CountingVisionIO(), new CountingVisionIO(), new CountingVisionIO()
        };
    var drive = new StubDrive();
    CameraConfig[] cameras = {dummyCamera("c0"), dummyCamera("c1"), dummyCamera("c2")};
    Vision vision = new Vision(ios, cameras, drive);

    // Make camera 1 return a valid pose — but it won't be processed until its turn
    ios[1] =
        new CountingVisionIO() {
          @Override
          public void updateInputs(VisionIOInputsAutoLogged inputs) {
            super.updateInputs(inputs);
            inputs.posePresent = true;
            inputs.poseX = 5.0;
            inputs.poseY = 3.0;
            inputs.poseZ = 0.0;
            inputs.poseRotRadians = 0.0;
            inputs.poseTimestamp = 0.1;
            inputs.targetCount = 2;
            inputs.targetFiducialIds = new int[] {1, 2};
            inputs.targetPoseAmbiguities = new double[] {0.1, 0.1};
          }
        };

    // Recreate vision with the new ios array
    vision = new Vision(ios, cameras, drive);

    // Cycle 1: updates camera 0 (no pose) — no vision measurement
    vision.periodic();
    assertEquals(0, drive.visionMeasurementCount);

    // Cycle 2: updates camera 1 (has valid pose) — should add vision measurement
    vision.periodic();
    assertEquals(1, drive.visionMeasurementCount);

    // Cycle 3: updates camera 2 (no pose) — no new vision measurement
    vision.periodic();
    assertEquals(
        1, drive.visionMeasurementCount, "Camera 1's stale data should not be reprocessed");
  }
}
