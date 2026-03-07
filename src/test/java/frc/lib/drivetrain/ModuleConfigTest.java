package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ModuleConfigTest {

    @Test
    void constructsWithValidConfig() {
        ModuleConfig config = new ModuleConfig(7, 8, 23, 0.124, 11.5, 11.5, false, false, false);
        assertEquals(7, config.driveMotorId);
        assertEquals(8, config.steerMotorId);
        assertEquals(23, config.encoderId);
        assertEquals(0.124, config.encoderOffsetRotations, 0.001);
        assertEquals(11.5, config.xPositionInches);
        assertEquals(11.5, config.yPositionInches);
        assertFalse(config.invertDrive);
        assertFalse(config.invertSteer);
        assertFalse(config.invertEncoder);
    }

    @Test
    void storesInversionFlags() {
        ModuleConfig config = new ModuleConfig(7, 8, 23, 0.0, 11.5, 11.5, true, true, true);
        assertTrue(config.invertDrive);
        assertTrue(config.invertSteer);
        assertTrue(config.invertEncoder);
    }

    @Test
    void rejectsDriveMotorIdTooLow() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new ModuleConfig(-1, 8, 23, 0.0, 11.5, 11.5, false, false, false));
    }

    @Test
    void rejectsDriveMotorIdTooHigh() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new ModuleConfig(63, 8, 23, 0.0, 11.5, 11.5, false, false, false));
    }

    @Test
    void rejectsSteerMotorIdOutOfRange() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new ModuleConfig(7, -1, 23, 0.0, 11.5, 11.5, false, false, false));
        assertThrows(
                IllegalArgumentException.class,
                () -> new ModuleConfig(7, 63, 23, 0.0, 11.5, 11.5, false, false, false));
    }

    @Test
    void rejectsEncoderIdOutOfRange() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new ModuleConfig(7, 8, -1, 0.0, 11.5, 11.5, false, false, false));
        assertThrows(
                IllegalArgumentException.class,
                () -> new ModuleConfig(7, 8, 63, 0.0, 11.5, 11.5, false, false, false));
    }

    @Test
    void rejectsEncoderOffsetTooLow() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new ModuleConfig(7, 8, 23, -1.1, 11.5, 11.5, false, false, false));
    }

    @Test
    void rejectsEncoderOffsetTooHigh() {
        assertThrows(
                IllegalArgumentException.class,
                () -> new ModuleConfig(7, 8, 23, 1.1, 11.5, 11.5, false, false, false));
    }

    @Test
    void acceptsBoundaryCanIds() {
        assertDoesNotThrow(() -> new ModuleConfig(0, 0, 0, 0.0, 0, 0, false, false, false));
        assertDoesNotThrow(() -> new ModuleConfig(62, 62, 62, 0.0, 0, 0, false, false, false));
    }

    @Test
    void acceptsBoundaryEncoderOffsets() {
        assertDoesNotThrow(() -> new ModuleConfig(7, 8, 23, -1.0, 11.5, 11.5, false, false, false));
        assertDoesNotThrow(() -> new ModuleConfig(7, 8, 23, 1.0, 11.5, 11.5, false, false, false));
    }
}
