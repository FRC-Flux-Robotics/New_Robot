package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import org.junit.jupiter.api.Test;

class DrivetrainConfigTest {

  private ModuleConfig testModule(double xPos, double yPos) {
    return new ModuleConfig(1, 2, 3, 0.0, xPos, yPos, false, false);
  }

  private DrivetrainConfig.Builder validBuilder() {
    return new DrivetrainConfig.Builder()
        .canBusName("test")
        .pigeonId(1)
        .frontLeft(testModule(11.75, 11.75))
        .frontRight(testModule(11.75, -11.75))
        .backLeft(testModule(-11.75, 11.75))
        .backRight(testModule(-11.75, -11.75))
        .driveGearRatio(6.39)
        .steerGearRatio(12.1)
        .couplingRatio(4.5)
        .wheelRadiusInches(2.0)
        .maxSpeedMps(4.5)
        .maxAngularRateRadPerSec(Math.PI)
        .steerGains(new PIDGains(100, 0, 0.5))
        .driveGains(new PIDGains(0.1, 0, 0, 0, 0.12, 0))
        .driveStatorCurrentLimit(60)
        .driveSupplyCurrentLimit(120)
        .steerStatorCurrentLimit(0)
        .translationDeadband(0.02)
        .rotationDeadband(0.02)
        .trackWidthInches(23.5)
        .trackLengthInches(23.5);
  }

  @Test
  void builderCreatesValidConfig() {
    DrivetrainConfig config = validBuilder().build();

    assertEquals("test", config.canBusName);
    assertEquals(1, config.pigeonId);
    assertEquals(6.39, config.driveGearRatio, 0.001);
    assertEquals(12.1, config.steerGearRatio, 0.001);
    assertEquals(2.0, config.wheelRadiusInches, 0.001);
    assertEquals(4.5, config.maxSpeedMps, 0.001);
    assertEquals(60, config.driveStatorCurrentLimit, 0.001);
    assertEquals(120, config.driveSupplyCurrentLimit, 0.001);
    assertEquals(0, config.steerStatorCurrentLimit, 0.001);
    assertEquals(23.5, config.trackWidthInches, 0.001);
    assertNotNull(config.frontLeft);
    assertNotNull(config.frontRight);
    assertNotNull(config.backLeft);
    assertNotNull(config.backRight);
  }

  @Test
  void builderThrowsOnMissingCanBus() {
    var builder = validBuilder().canBusName(null);
    assertThrows(IllegalStateException.class, builder::build);
  }

  @Test
  void builderThrowsOnMissingPigeonId() {
    var builder = new DrivetrainConfig.Builder()
        .canBusName("test")
        // pigeonId not set (defaults to -1)
        .frontLeft(testModule(1, 1))
        .frontRight(testModule(1, -1))
        .backLeft(testModule(-1, 1))
        .backRight(testModule(-1, -1))
        .driveGearRatio(6.39)
        .steerGearRatio(12.1)
        .wheelRadiusInches(2.0)
        .maxSpeedMps(4.5)
        .maxAngularRateRadPerSec(Math.PI)
        .steerGains(new PIDGains(100, 0, 0.5))
        .driveGains(new PIDGains(0.1, 0, 0))
        .trackWidthInches(23.5)
        .trackLengthInches(23.5);
    assertThrows(IllegalStateException.class, builder::build);
  }

  @Test
  void builderThrowsOnMissingModules() {
    var builder = validBuilder().frontLeft(null);
    assertThrows(IllegalStateException.class, builder::build);
  }

  @Test
  void builderThrowsOnMissingGains() {
    var builder = validBuilder().steerGains(null);
    assertThrows(IllegalStateException.class, builder::build);
  }

  @Test
  void builderThrowsOnZeroGearRatio() {
    var builder = validBuilder().driveGearRatio(0);
    assertThrows(IllegalStateException.class, builder::build);
  }

  @Test
  void builderThrowsOnZeroWheelRadius() {
    var builder = validBuilder().wheelRadiusInches(0);
    assertThrows(IllegalStateException.class, builder::build);
  }

  @Test
  void pidGainsStoresValues() {
    PIDGains gains = new PIDGains(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
    assertEquals(1.0, gains.kP);
    assertEquals(2.0, gains.kI);
    assertEquals(3.0, gains.kD);
    assertEquals(4.0, gains.kS);
    assertEquals(5.0, gains.kV);
    assertEquals(6.0, gains.kA);
  }

  @Test
  void pidGainsShortConstructorDefaultsToZero() {
    PIDGains gains = new PIDGains(1.0, 2.0, 3.0);
    assertEquals(1.0, gains.kP);
    assertEquals(2.0, gains.kI);
    assertEquals(3.0, gains.kD);
    assertEquals(0.0, gains.kS);
    assertEquals(0.0, gains.kV);
    assertEquals(0.0, gains.kA);
  }

  @Test
  void speedCoefficientDefaultsToOne() {
    DrivetrainConfig config = validBuilder().build();
    assertEquals(1.0, config.speedCoefficient, 0.001);
  }

  @Test
  void speedCoefficientStoresCustomValue() {
    DrivetrainConfig config = validBuilder().speedCoefficient(0.5).build();
    assertEquals(0.5, config.speedCoefficient, 0.001);
  }

  @Test
  void speedCoefficientThrowsOnZero() {
    var builder = validBuilder().speedCoefficient(0);
    assertThrows(IllegalStateException.class, builder::build);
  }

  @Test
  void speedCoefficientThrowsOnGreaterThanOne() {
    var builder = validBuilder().speedCoefficient(1.5);
    assertThrows(IllegalStateException.class, builder::build);
  }

  @Test
  void noCamerasDefaultsToEmptyList() {
    DrivetrainConfig config = validBuilder().build();
    assertNotNull(config.cameras);
    assertTrue(config.cameras.isEmpty());
  }

  @Test
  void singleCameraInList() {
    var transform = new Transform3d(new Translation3d(0.3, 0, 0.25), new Rotation3d());
    DrivetrainConfig config = validBuilder()
        .camera(new CameraConfig("Cam1", transform))
        .build();
    assertEquals(1, config.cameras.size());
    assertEquals("Cam1", config.cameras.get(0).name());
  }

  @Test
  void multipleCamerasInList() {
    var transform = new Transform3d(new Translation3d(0.3, 0, 0.25), new Rotation3d());
    DrivetrainConfig config = validBuilder()
        .camera(new CameraConfig("Cam1", transform))
        .camera(new CameraConfig("Cam2", transform))
        .build();
    assertEquals(2, config.cameras.size());
    assertEquals("Cam1", config.cameras.get(0).name());
    assertEquals("Cam2", config.cameras.get(1).name());
  }

  @Test
  void camerasListIsUnmodifiable() {
    var transform = new Transform3d(new Translation3d(0.3, 0, 0.25), new Rotation3d());
    DrivetrainConfig config = validBuilder()
        .camera(new CameraConfig("Cam1", transform))
        .build();
    assertThrows(UnsupportedOperationException.class,
        () -> config.cameras.add(new CameraConfig("Cam2", transform)));
  }

  @Test
  void motionMagicExpoDefaultValues() {
    DrivetrainConfig config = validBuilder().build();
    assertEquals(0.12, config.steerMotionMagicExpoKv, 0.001);
    assertEquals(0.1, config.steerMotionMagicExpoKa, 0.001);
  }

  @Test
  void motionMagicExpoCustomValues() {
    DrivetrainConfig config = validBuilder()
        .steerMotionMagicExpoKv(0.2)
        .steerMotionMagicExpoKa(0.05)
        .build();
    assertEquals(0.2, config.steerMotionMagicExpoKv, 0.001);
    assertEquals(0.05, config.steerMotionMagicExpoKa, 0.001);
  }

  @Test
  void headingGainsDefaultValues() {
    DrivetrainConfig config = validBuilder().build();
    assertNotNull(config.headingGains);
    assertEquals(5.0, config.headingGains.kP, 0.001);
    assertEquals(0.0, config.headingGains.kI, 0.001);
    assertEquals(0.0, config.headingGains.kD, 0.001);
  }

  @Test
  void headingGainsCustomValues() {
    DrivetrainConfig config = validBuilder()
        .headingGains(new PIDGains(8.0, 0.1, 0.5))
        .build();
    assertEquals(8.0, config.headingGains.kP, 0.001);
    assertEquals(0.1, config.headingGains.kI, 0.001);
    assertEquals(0.5, config.headingGains.kD, 0.001);
  }

  @Test
  void moduleConfigStoresValues() {
    ModuleConfig config = new ModuleConfig(7, 8, 23, 0.125, 11.75, -11.75, true, false);
    assertEquals(7, config.driveMotorId);
    assertEquals(8, config.steerMotorId);
    assertEquals(23, config.encoderId);
    assertEquals(0.125, config.encoderOffset, 0.001);
    assertEquals(11.75, config.xPositionInches, 0.001);
    assertEquals(-11.75, config.yPositionInches, 0.001);
    assertTrue(config.invertDrive);
    assertFalse(config.invertSteer);
  }
}
