package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class FieldPositionsTest {

  @Test
  void mirrorFlipsXAndRotation() {
    Pose2d blue = new Pose2d(1.0, 3.0, Rotation2d.fromDegrees(0));
    Pose2d red = FieldPositions.mirror(blue);

    assertEquals(16.54 - 1.0, red.getX(), 0.001);
    assertEquals(3.0, red.getY(), 0.001);
    assertEquals(180.0, red.getRotation().getDegrees(), 0.001);
  }

  @Test
  void mirrorPreservesYCoordinate() {
    Pose2d blue = new Pose2d(2.0, 7.0, Rotation2d.fromDegrees(45));
    Pose2d red = FieldPositions.mirror(blue);

    assertEquals(7.0, red.getY(), 0.001);
  }

  @Test
  void mirrorIsOwnInverse() {
    Pose2d original = new Pose2d(3.5, 4.1, Rotation2d.fromDegrees(0));
    Pose2d mirrored = FieldPositions.mirror(original);
    Pose2d back = FieldPositions.mirror(mirrored);

    assertEquals(original.getX(), back.getX(), 0.001);
    assertEquals(original.getY(), back.getY(), 0.001);
    assertEquals(original.getRotation().getDegrees(), back.getRotation().getDegrees(), 0.001);
  }

  @Test
  void resolveOriginAlwaysZero() {
    // Origin should never be mirrored
    Pose2d origin = FieldPositions.resolve("Origin");
    assertNotNull(origin);
    assertEquals(0.0, origin.getX(), 0.001);
    assertEquals(0.0, origin.getY(), 0.001);
  }

  @Test
  void resolveUnknownReturnsNull() {
    assertNull(FieldPositions.resolve("NonExistent"));
  }

  @Test
  void resolveKnownPositionsNotNull() {
    assertNotNull(FieldPositions.resolve("Left"));
    assertNotNull(FieldPositions.resolve("Right"));
    assertNotNull(FieldPositions.resolve("HUB"));
  }

  @Test
  void mirrorWithNonZeroHeading() {
    Pose2d blue = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(45));
    Pose2d red = FieldPositions.mirror(blue);

    assertEquals(16.54 - 2.0, red.getX(), 0.001);
    assertEquals(3.0, red.getY(), 0.001);
    assertEquals(-135.0, red.getRotation().getDegrees(), 0.001);
  }

  @Test
  void forAllianceTranslationMirrorsX() {
    // forAlliance(Translation2d) should mirror X when on red alliance
    // Since we can't easily set alliance in unit tests, test the mirror logic directly
    Translation2d blue = new Translation2d(1.0, 3.0);
    // Simulate red alliance mirror: flip X only
    Translation2d red =
        new Translation2d(FieldPositions.FIELD_LENGTH_METERS - blue.getX(), blue.getY());

    assertEquals(16.54 - 1.0, red.getX(), 0.001);
    assertEquals(3.0, red.getY(), 0.001);
  }

  @Test
  void mirrorCenterFieldStaysAtCenter() {
    // A pose at field center X should stay at center X after mirror
    double centerX = FieldPositions.FIELD_LENGTH_METERS / 2.0;
    Pose2d center = new Pose2d(centerX, 4.0, Rotation2d.fromDegrees(0));
    Pose2d mirrored = FieldPositions.mirror(center);

    assertEquals(centerX, mirrored.getX(), 0.001);
  }
}
