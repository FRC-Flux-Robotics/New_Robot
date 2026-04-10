package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class CameraValidationCmdTest {

  // --- normalizeAngle ---

  @Test
  void normalizeAngleWithinRange() {
    assertEquals(0.5, CameraValidationCmd.normalizeAngle(0.5), 1e-9);
    assertEquals(-0.5, CameraValidationCmd.normalizeAngle(-0.5), 1e-9);
  }

  @Test
  void normalizeAngleWrapsPositive() {
    assertEquals(-Math.PI + 0.1, CameraValidationCmd.normalizeAngle(Math.PI + 0.1), 1e-9);
  }

  @Test
  void normalizeAngleWrapsNegative() {
    assertEquals(Math.PI - 0.1, CameraValidationCmd.normalizeAngle(-Math.PI - 0.1), 1e-9);
  }

  @Test
  void normalizeAngleMultipleWraps() {
    assertEquals(0.0, CameraValidationCmd.normalizeAngle(4 * Math.PI), 1e-9);
    assertEquals(0.0, CameraValidationCmd.normalizeAngle(-4 * Math.PI), 1e-9);
  }

  // --- computeStationMeans ---

  @Test
  void stationMeansAveragesSamples() {
    // 1 camera, 1 station, 2 rotations with data
    double[][][] poseX = {{{10.0, 6.0, 0, 0}}};
    double[][][] poseY = {{{4.0, 8.0, 0, 0}}};
    int[][][] counts = {{{2, 3, 0, 0}}}; // 10/2=5, 6/3=2
    double[][] stationX = {{0}};
    double[][] stationY = {{0}};

    CameraValidationCmd.computeStationMeans(poseX, poseY, counts, stationX, stationY, 1, 1, 4);

    // Rotation averages: X=[5, 2], Y=[2, 8/3]
    assertEquals(5.0, poseX[0][0][0], 1e-9);
    assertEquals(2.0, poseX[0][0][1], 1e-9);
    // Station mean = (5+2)/2 = 3.5
    assertEquals(3.5, stationX[0][0], 1e-9);
  }

  @Test
  void stationMeansNaNWhenNoData() {
    double[][][] poseX = {{{0, 0}}};
    double[][][] poseY = {{{0, 0}}};
    int[][][] counts = {{{0, 0}}};
    double[][] stationX = {{0}};
    double[][] stationY = {{0}};

    CameraValidationCmd.computeStationMeans(poseX, poseY, counts, stationX, stationY, 1, 1, 2);

    assertTrue(Double.isNaN(stationX[0][0]));
    assertTrue(Double.isNaN(stationY[0][0]));
  }

  // --- computeTranslationError ---

  @Test
  void translationErrorDetectsXOffset() {
    // Simulate a camera with +0.05m X offset in robot frame.
    // At heading=0, this shifts field pose by (+0.05, 0).
    // At heading=90deg, shifts by (0, +0.05).
    // At heading=180deg, shifts by (-0.05, 0).
    // At heading=270deg, shifts by (0, -0.05).
    // The mean across 4 rotations is (0, 0), so deviations are the shifts themselves.
    double offset = 0.05;
    double trueX = 3.0, trueY = 2.0;

    // Pre-averaged rotation poses (counts=1 means already averaged)
    double[][][] poseX = {
      {
        {
          trueX + offset, // heading 0
          trueX + 0, // heading 90 (offset goes to Y)
          trueX - offset, // heading 180
          trueX - 0 // heading 270
        }
      }
    };
    double[][][] poseY = {
      {
        {
          trueY + 0, // heading 0
          trueY + offset, // heading 90
          trueY - 0, // heading 180
          trueY - offset // heading 270
        }
      }
    };
    int[][][] counts = {{{1, 1, 1, 1}}};
    double[][] stationX = {{trueX}}; // mean is true position
    double[][] stationY = {{trueY}};

    double[][] result =
        CameraValidationCmd.computeTranslationError(
            0, poseX, poseY, counts, stationX, stationY, 0.0, 1, 4);

    assertNotNull(result);
    // Should detect the +0.05m X offset in robot frame
    assertEquals(offset, result[0][0], 1e-9);
    // Y offset should be ~0
    assertEquals(0.0, result[1][0], 1e-9);
  }

  @Test
  void translationErrorDetectsYOffset() {
    // Camera with +0.05m Y offset in robot frame.
    // At heading=0: field shift (0, +0.05)
    // At heading=90: field shift (-0.05, 0)
    // At heading=180: field shift (0, -0.05)
    // At heading=270: field shift (+0.05, 0)
    double offset = 0.05;
    double trueX = 3.0, trueY = 2.0;

    double[][][] poseX = {{{trueX + 0, trueX - offset, trueX - 0, trueX + offset}}};
    double[][][] poseY = {{{trueY + offset, trueY + 0, trueY - offset, trueY - 0}}};
    int[][][] counts = {{{1, 1, 1, 1}}};
    double[][] stationX = {{trueX}};
    double[][] stationY = {{trueY}};

    double[][] result =
        CameraValidationCmd.computeTranslationError(
            0, poseX, poseY, counts, stationX, stationY, 0.0, 1, 4);

    assertNotNull(result);
    assertEquals(0.0, result[0][0], 1e-9); // X offset ~0
    assertEquals(offset, result[1][0], 1e-9); // Y offset detected
  }

  @Test
  void translationErrorNullWhenNoData() {
    double[][][] poseX = {{{0, 0, 0, 0}}};
    double[][][] poseY = {{{0, 0, 0, 0}}};
    int[][][] counts = {{{0, 0, 0, 0}}};
    double[][] stationX = {{Double.NaN}};
    double[][] stationY = {{Double.NaN}};

    double[][] result =
        CameraValidationCmd.computeTranslationError(
            0, poseX, poseY, counts, stationX, stationY, 0.0, 1, 4);

    assertNull(result);
  }

  @Test
  void translationErrorWithNonZeroStartHeading() {
    // Same X offset test but starting at 45 degrees
    double offset = 0.05;
    double startHeading = Math.PI / 4.0;
    double trueX = 3.0, trueY = 2.0;
    double c0 = Math.cos(startHeading), s0 = Math.sin(startHeading);

    // At each rotation, the offset rotates with the robot heading
    double[] headings = new double[4];
    double[] shiftX = new double[4];
    double[] shiftY = new double[4];
    for (int r = 0; r < 4; r++) {
      headings[r] = startHeading + r * Math.PI / 2.0;
      shiftX[r] = offset * Math.cos(headings[r]);
      shiftY[r] = offset * Math.sin(headings[r]);
    }

    double[][][] poseX = {
      {{trueX + shiftX[0], trueX + shiftX[1], trueX + shiftX[2], trueX + shiftX[3]}}
    };
    double[][][] poseY = {
      {{trueY + shiftY[0], trueY + shiftY[1], trueY + shiftY[2], trueY + shiftY[3]}}
    };
    int[][][] counts = {{{1, 1, 1, 1}}};
    double[][] stationX = {{trueX}};
    double[][] stationY = {{trueY}};

    double[][] result =
        CameraValidationCmd.computeTranslationError(
            0, poseX, poseY, counts, stationX, stationY, startHeading, 1, 4);

    assertNotNull(result);
    assertEquals(offset, result[0][0], 1e-9);
    assertEquals(0.0, result[1][0], 1e-9);
  }

  @Test
  void translationErrorZeroForPerfectCamera() {
    // All rotations report the same pose — no offset
    double trueX = 3.0, trueY = 2.0;
    double[][][] poseX = {{{trueX, trueX, trueX, trueX}}};
    double[][][] poseY = {{{trueY, trueY, trueY, trueY}}};
    int[][][] counts = {{{1, 1, 1, 1}}};
    double[][] stationX = {{trueX}};
    double[][] stationY = {{trueY}};

    double[][] result =
        CameraValidationCmd.computeTranslationError(
            0, poseX, poseY, counts, stationX, stationY, 0.0, 1, 4);

    assertNotNull(result);
    assertEquals(0.0, result[0][0], 1e-9);
    assertEquals(0.0, result[1][0], 1e-9);
  }

  // --- computeYawError ---

  @Test
  void yawErrorZeroWhenDisplacementsMatch() {
    // Vision and odometry both move +1m in X
    double[][] visionX = {{0.0, 1.0}};
    double[][] visionY = {{0.0, 0.0}};
    Pose2d[] odomPoses = {new Pose2d(0, 0, new Rotation2d()), new Pose2d(1, 0, new Rotation2d())};

    double err = CameraValidationCmd.computeYawError(0, visionX, visionY, odomPoses, 2);
    assertEquals(0.0, err, 1e-9);
  }

  @Test
  void yawErrorDetectsAngularMismatch() {
    // Odometry says moved in +X, vision says moved at 10 degrees off
    double angleDeg = 10.0;
    double angleRad = Math.toRadians(angleDeg);
    double dist = 1.0;
    double[][] visionX = {{0.0, dist * Math.cos(angleRad)}};
    double[][] visionY = {{0.0, dist * Math.sin(angleRad)}};
    Pose2d[] odomPoses = {
      new Pose2d(0, 0, new Rotation2d()), new Pose2d(dist, 0, new Rotation2d())
    };

    double err = CameraValidationCmd.computeYawError(0, visionX, visionY, odomPoses, 2);
    assertEquals(angleDeg, err, 0.1);
  }

  @Test
  void yawErrorSkipsShortDisplacements() {
    // Displacement < 0.1m should be skipped
    double[][] visionX = {{0.0, 0.05}};
    double[][] visionY = {{0.0, 0.05}};
    Pose2d[] odomPoses = {
      new Pose2d(0, 0, new Rotation2d()), new Pose2d(0.05, 0, new Rotation2d())
    };

    double err = CameraValidationCmd.computeYawError(0, visionX, visionY, odomPoses, 2);
    assertEquals(0.0, err, 1e-9);
  }

  @Test
  void yawErrorZeroWithNaNStations() {
    double[][] visionX = {{Double.NaN, 1.0}};
    double[][] visionY = {{Double.NaN, 0.0}};
    Pose2d[] odomPoses = {new Pose2d(0, 0, new Rotation2d()), new Pose2d(1, 0, new Rotation2d())};

    double err = CameraValidationCmd.computeYawError(0, visionX, visionY, odomPoses, 2);
    assertEquals(0.0, err, 1e-9);
  }

  @Test
  void yawErrorAveragesMultipleStations() {
    // 3 stations, both displacements show ~5 degree error
    double angleDeg = 5.0;
    double angleRad = Math.toRadians(angleDeg);
    double dist = 1.0;
    double[][] visionX = {{0.0, dist * Math.cos(angleRad), 2 * dist * Math.cos(angleRad)}};
    double[][] visionY = {{0.0, dist * Math.sin(angleRad), 2 * dist * Math.sin(angleRad)}};
    Pose2d[] odomPoses = {
      new Pose2d(0, 0, new Rotation2d()),
      new Pose2d(dist, 0, new Rotation2d()),
      new Pose2d(2 * dist, 0, new Rotation2d())
    };

    double err = CameraValidationCmd.computeYawError(0, visionX, visionY, odomPoses, 3);
    assertEquals(angleDeg, err, 0.1);
  }
}
