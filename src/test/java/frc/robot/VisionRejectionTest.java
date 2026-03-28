package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.vision.VisionIO;
import org.junit.jupiter.api.Test;

class VisionRejectionTest {

  /** Creates inputs simulating a single tag with the given ambiguity. */
  private static VisionIO.VisionIOInputs makeInputs(int tagCount, double ambiguity) {
    var inputs = new VisionIO.VisionIOInputs();
    inputs.targetCount = tagCount;
    inputs.targetFiducialIds = new int[tagCount];
    inputs.targetPoseAmbiguities = new double[tagCount];
    for (int i = 0; i < tagCount; i++) {
      inputs.targetFiducialIds[i] = i + 1;
      inputs.targetPoseAmbiguities[i] = ambiguity;
    }
    return inputs;
  }

  private static Pose3d fieldCenter() {
    return new Pose3d(8.0, 4.0, 0.0, new Rotation3d());
  }

  @Test
  void validMeasurementAccepted() {
    assertNull(Vision.checkRejection(fieldCenter(), makeInputs(1, 0.1), 1.0, 0.5));
  }

  @Test
  void highAmbiguitySingleTagRejected() {
    assertEquals(
        VisionRejectReason.TOO_AMBIGUOUS,
        Vision.checkRejection(fieldCenter(), makeInputs(1, 0.5), 0.0, 0.0));
  }

  @Test
  void highAmbiguityMultiTagAccepted() {
    // Ambiguity check only applies to single-tag results
    assertNull(Vision.checkRejection(fieldCenter(), makeInputs(2, 0.5), 0.0, 0.0));
  }

  @Test
  void poseOutsideFieldRejectedNegativeX() {
    Pose3d pose = new Pose3d(-1.0, 4.0, 0.0, new Rotation3d());
    assertEquals(
        VisionRejectReason.OUT_OF_FIELD, Vision.checkRejection(pose, makeInputs(1, 0.1), 0.0, 0.0));
  }

  @Test
  void poseOutsideFieldRejectedLargeX() {
    Pose3d pose = new Pose3d(18.0, 4.0, 0.0, new Rotation3d());
    assertEquals(
        VisionRejectReason.OUT_OF_FIELD, Vision.checkRejection(pose, makeInputs(1, 0.1), 0.0, 0.0));
  }

  @Test
  void poseOutsideFieldRejectedNegativeY() {
    Pose3d pose = new Pose3d(8.0, -1.0, 0.0, new Rotation3d());
    assertEquals(
        VisionRejectReason.OUT_OF_FIELD, Vision.checkRejection(pose, makeInputs(1, 0.1), 0.0, 0.0));
  }

  @Test
  void poseOutsideFieldRejectedLargeY() {
    Pose3d pose = new Pose3d(8.0, 9.0, 0.0, new Rotation3d());
    assertEquals(
        VisionRejectReason.OUT_OF_FIELD, Vision.checkRejection(pose, makeInputs(1, 0.1), 0.0, 0.0));
  }

  @Test
  void poseWithinMarginAccepted() {
    // Just inside the margin (0.5m)
    Pose3d pose = new Pose3d(-0.4, -0.4, 0.0, new Rotation3d());
    assertNull(Vision.checkRejection(pose, makeInputs(1, 0.1), 0.0, 0.0));
  }

  @Test
  void zErrorTooLargeRejected() {
    Pose3d pose = new Pose3d(8.0, 4.0, 1.0, new Rotation3d());
    assertEquals(
        VisionRejectReason.Z_ERROR, Vision.checkRejection(pose, makeInputs(1, 0.1), 0.0, 0.0));
  }

  @Test
  void zErrorNegativeRejected() {
    Pose3d pose = new Pose3d(8.0, 4.0, -0.8, new Rotation3d());
    assertEquals(
        VisionRejectReason.Z_ERROR, Vision.checkRejection(pose, makeInputs(1, 0.1), 0.0, 0.0));
  }

  @Test
  void spinningTooFastRejected() {
    assertEquals(
        VisionRejectReason.ANGULAR_VEL_TOO_HIGH,
        Vision.checkRejection(fieldCenter(), makeInputs(1, 0.1), 0.0, 2.5));
  }

  @Test
  void movingTooFastRejected() {
    assertEquals(
        VisionRejectReason.MOVING_TOO_FAST,
        Vision.checkRejection(fieldCenter(), makeInputs(1, 0.1), 3.5, 0.0));
  }

  @Test
  void firstRejectReasonReturned() {
    // High ambiguity + out of field — ambiguity is checked first
    Pose3d pose = new Pose3d(-5.0, 4.0, 0.0, new Rotation3d());
    assertEquals(
        VisionRejectReason.TOO_AMBIGUOUS,
        Vision.checkRejection(pose, makeInputs(1, 0.5), 0.0, 0.0));
  }
}
