package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class AutoDriveConfigTest {

  @Test
  void defaultsMatchOriginalConstants() {
    AutoDriveConfig config = AutoDriveConfig.defaults();
    assertEquals(5.0, config.headingKP);
    assertEquals(5.0, config.autoTranslationKP);
    assertEquals(0.0, config.autoTranslationKI);
    assertEquals(0.0, config.autoTranslationKD);
    assertEquals(5.0, config.autoRotationKP);
    assertEquals(0.0, config.autoRotationKI);
    assertEquals(0.0, config.autoRotationKD);
    assertEquals(2.0, config.driveToPoseMaxVelMps);
    assertEquals(2.0, config.driveToPoseMaxAccelMps2);
    assertEquals(2.0, config.driveToPoseRotToleranceDeg);
    assertEquals(2.5, config.pathMaxAccelMps2);
    assertEquals(Math.PI, config.pathMaxAngularAccelRadPerSec2, 0.001);
  }

  @Test
  void builderOverridesDefaults() {
    AutoDriveConfig config =
        AutoDriveConfig.builder()
            .headingKP(8.0)
            .autoTranslationPID(3.0, 0.1, 0.5)
            .autoRotationPID(4.0, 0.2, 0.3)
            .driveToPoseMaxVel(3.0)
            .driveToPoseMaxAccel(4.0)
            .driveToPoseRotTolerance(5.0)
            .pathMaxAccel(3.5)
            .pathMaxAngularAccel(2.0)
            .build();
    assertEquals(8.0, config.headingKP);
    assertEquals(3.0, config.autoTranslationKP);
    assertEquals(0.1, config.autoTranslationKI);
    assertEquals(0.5, config.autoTranslationKD);
    assertEquals(4.0, config.autoRotationKP);
    assertEquals(0.2, config.autoRotationKI);
    assertEquals(0.3, config.autoRotationKD);
    assertEquals(3.0, config.driveToPoseMaxVelMps);
    assertEquals(4.0, config.driveToPoseMaxAccelMps2);
    assertEquals(5.0, config.driveToPoseRotToleranceDeg);
    assertEquals(3.5, config.pathMaxAccelMps2);
    assertEquals(2.0, config.pathMaxAngularAccelRadPerSec2);
  }

  @Test
  void rejectsNonPositiveHeadingKP() {
    assertThrows(IllegalArgumentException.class, () -> AutoDriveConfig.builder().headingKP(0));
    assertThrows(IllegalArgumentException.class, () -> AutoDriveConfig.builder().headingKP(-1));
  }

  @Test
  void rejectsNegativeTranslationKP() {
    assertThrows(
        IllegalArgumentException.class,
        () -> AutoDriveConfig.builder().autoTranslationPID(-1, 0, 0));
  }

  @Test
  void rejectsNegativeRotationKP() {
    assertThrows(
        IllegalArgumentException.class, () -> AutoDriveConfig.builder().autoRotationPID(-1, 0, 0));
  }

  @Test
  void rejectsNonPositiveVelocity() {
    assertThrows(
        IllegalArgumentException.class, () -> AutoDriveConfig.builder().driveToPoseMaxVel(0));
  }

  @Test
  void rejectsNonPositiveAccel() {
    assertThrows(
        IllegalArgumentException.class, () -> AutoDriveConfig.builder().driveToPoseMaxAccel(0));
  }

  @Test
  void rejectsNonPositiveRotTolerance() {
    assertThrows(
        IllegalArgumentException.class, () -> AutoDriveConfig.builder().driveToPoseRotTolerance(0));
  }

  @Test
  void rejectsNonPositivePathAccel() {
    assertThrows(IllegalArgumentException.class, () -> AutoDriveConfig.builder().pathMaxAccel(0));
  }

  @Test
  void rejectsNonPositivePathAngularAccel() {
    assertThrows(
        IllegalArgumentException.class, () -> AutoDriveConfig.builder().pathMaxAngularAccel(0));
  }

  @Test
  void allowsZeroTranslationKP() {
    assertDoesNotThrow(() -> AutoDriveConfig.builder().autoTranslationPID(0, 0, 0));
  }

  @Test
  void allowsZeroRotationKP() {
    assertDoesNotThrow(() -> AutoDriveConfig.builder().autoRotationPID(0, 0, 0));
  }
}
