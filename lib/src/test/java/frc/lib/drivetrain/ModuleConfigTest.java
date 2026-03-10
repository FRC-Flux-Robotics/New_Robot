package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ModuleConfigTest {

  @Test
  void constructsWithValidConfig() {
    ModuleConfig config = new ModuleConfig(7, 8, 23, 0.124, 0.2921, 0.2921, false, false, false);
    assertEquals(7, config.driveMotorId);
    assertEquals(8, config.steerMotorId);
    assertEquals(23, config.encoderId);
    assertEquals(0.124, config.encoderOffsetRotations, 0.001);
    assertEquals(0.2921, config.xPositionMeters, 0.0001);
    assertEquals(0.2921, config.yPositionMeters, 0.0001);
    assertFalse(config.invertDrive);
    assertFalse(config.invertSteer);
    assertFalse(config.invertEncoder);
  }

  @Test
  void storesInversionFlags() {
    ModuleConfig config = new ModuleConfig(7, 8, 23, 0.0, 0.2921, 0.2921, true, true, true);
    assertTrue(config.invertDrive);
    assertTrue(config.invertSteer);
    assertTrue(config.invertEncoder);
  }

  @Test
  void rejectsDriveMotorIdTooLow() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new ModuleConfig(-1, 8, 23, 0.0, 0.2921, 0.2921, false, false, false));
  }

  @Test
  void rejectsDriveMotorIdTooHigh() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new ModuleConfig(63, 8, 23, 0.0, 0.2921, 0.2921, false, false, false));
  }

  @Test
  void rejectsSteerMotorIdOutOfRange() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new ModuleConfig(7, -1, 23, 0.0, 0.2921, 0.2921, false, false, false));
    assertThrows(
        IllegalArgumentException.class,
        () -> new ModuleConfig(7, 63, 23, 0.0, 0.2921, 0.2921, false, false, false));
  }

  @Test
  void rejectsEncoderIdOutOfRange() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new ModuleConfig(7, 8, -1, 0.0, 0.2921, 0.2921, false, false, false));
    assertThrows(
        IllegalArgumentException.class,
        () -> new ModuleConfig(7, 8, 63, 0.0, 0.2921, 0.2921, false, false, false));
  }

  @Test
  void rejectsEncoderOffsetTooLow() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new ModuleConfig(7, 8, 23, -1.1, 0.2921, 0.2921, false, false, false));
  }

  @Test
  void rejectsEncoderOffsetTooHigh() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new ModuleConfig(7, 8, 23, 1.1, 0.2921, 0.2921, false, false, false));
  }

  @Test
  void acceptsBoundaryCanIds() {
    assertDoesNotThrow(() -> new ModuleConfig(0, 0, 0, 0.0, 0, 0, false, false, false));
    assertDoesNotThrow(() -> new ModuleConfig(62, 62, 62, 0.0, 0, 0, false, false, false));
  }

  @Test
  void toStringCompactFormat() {
    ModuleConfig config = new ModuleConfig(7, 8, 23, 0.124, 0.2921, 0.2921, false, false, false);
    String str = config.toString();
    assertEquals(
        "ModuleConfig(drive=7, steer=8, encoder=23, offset=0.124, pos=(0.2921, 0.2921), inv=FFF)",
        str);
  }

  @Test
  void toStringShowsInversions() {
    ModuleConfig config = new ModuleConfig(1, 2, 3, 0.0, 0.5, -0.5, true, false, true);
    String str = config.toString();
    assertEquals(
        "ModuleConfig(drive=1, steer=2, encoder=3, offset=0.0, pos=(0.5, -0.5), inv=TFT)", str);
  }

  @Test
  void acceptsBoundaryEncoderOffsets() {
    assertDoesNotThrow(() -> new ModuleConfig(7, 8, 23, -1.0, 0.2921, 0.2921, false, false, false));
    assertDoesNotThrow(() -> new ModuleConfig(7, 8, 23, 1.0, 0.2921, 0.2921, false, false, false));
  }
}
