package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class DrivetrainConfigTest {

    private static final ModuleConfig FL =
            new ModuleConfig(7, 8, 23, 0.124, 0.2921, 0.2921, false, false, false);
    private static final ModuleConfig FR =
            new ModuleConfig(1, 2, 20, -0.291, 0.2921, -0.2921, true, false, false);
    private static final ModuleConfig BL =
            new ModuleConfig(5, 6, 22, 0.048, -0.2921, 0.2921, false, false, false);
    private static final ModuleConfig BR =
            new ModuleConfig(3, 4, 21, -0.371, -0.2921, -0.2921, true, false, false);
    private static final PIDGains STEER_GAINS = new PIDGains(100, 0, 0.5, 0.1, 1.5, 0);
    private static final PIDGains DRIVE_GAINS = new PIDGains(0.1, 0, 0, 0, 0.124, 0);

    private static DrivetrainConfig.Builder validBuilder() {
        return DrivetrainConfig.builder()
                .canBus("CANdace")
                .pigeonId(24)
                .frontLeft(FL)
                .frontRight(FR)
                .backLeft(BL)
                .backRight(BR)
                .gearing(6.39, 12.1, 4.5, 0.0508)
                .speed(4.99, 0.75 * 2 * Math.PI)
                .steerPID(STEER_GAINS)
                .drivePID(DRIVE_GAINS)
                .currentLimits(40, 35, 20, 120)
                .deadband(0.1, 0.1);
    }

    @Test
    void buildsWithAllFields() {
        DrivetrainConfig config = validBuilder().build();
        assertEquals("CANdace", config.canBus);
        assertEquals(24, config.pigeonId);
        assertSame(FL, config.frontLeft);
        assertSame(FR, config.frontRight);
        assertSame(BL, config.backLeft);
        assertSame(BR, config.backRight);
        assertEquals(6.39, config.driveGearRatio);
        assertEquals(12.1, config.steerGearRatio);
        assertEquals(4.5, config.couplingRatio);
        assertEquals(0.0508, config.wheelRadiusMeters, 0.0001);
        assertEquals(4.99, config.maxSpeedMps);
        assertEquals(0.75 * 2 * Math.PI, config.maxAngularRateRadPerSec, 0.001);
        assertSame(STEER_GAINS, config.steerGains);
        assertSame(DRIVE_GAINS, config.driveGains);
        assertEquals(40, config.driveStatorCurrentLimit);
        assertEquals(35, config.driveSupplyCurrentLimit);
        assertEquals(20, config.steerStatorCurrentLimit);
        assertEquals(120, config.slipCurrentAmps);
        assertEquals(0.1, config.translationDeadband);
        assertEquals(0.1, config.rotationDeadband);
    }

    // --- Missing required fields ---

    @Test
    void rejectsMissingCanBus() {
        DrivetrainConfig.Builder b = validBuilder();
        b.canBus(null);
        assertThrows(IllegalStateException.class, b::build);
    }

    @Test
    void rejectsMissingPigeonId() {
        assertThrows(IllegalStateException.class, () -> DrivetrainConfig.builder()
                .canBus("CANdace")
                .frontLeft(FL).frontRight(FR).backLeft(BL).backRight(BR)
                .gearing(6.39, 12.1, 4.5, 0.0508)
                .speed(4.99, 4.71)
                .steerPID(STEER_GAINS).drivePID(DRIVE_GAINS)
                .currentLimits(40, 35, 20, 120)
                .build());
    }

    @Test
    void rejectsMissingModule() {
        assertThrows(IllegalStateException.class, () -> validBuilder().frontLeft(null).build());
        assertThrows(IllegalStateException.class, () -> validBuilder().frontRight(null).build());
        assertThrows(IllegalStateException.class, () -> validBuilder().backLeft(null).build());
        assertThrows(IllegalStateException.class, () -> validBuilder().backRight(null).build());
    }

    @Test
    void rejectsMissingGearing() {
        assertThrows(IllegalStateException.class, () -> DrivetrainConfig.builder()
                .canBus("CANdace")
                .pigeonId(24)
                .frontLeft(FL).frontRight(FR).backLeft(BL).backRight(BR)
                .speed(4.99, 4.71)
                .steerPID(STEER_GAINS).drivePID(DRIVE_GAINS)
                .currentLimits(40, 35, 20, 120)
                .build());
    }

    @Test
    void rejectsMissingSpeed() {
        assertThrows(IllegalStateException.class, () -> DrivetrainConfig.builder()
                .canBus("CANdace")
                .pigeonId(24)
                .frontLeft(FL).frontRight(FR).backLeft(BL).backRight(BR)
                .gearing(6.39, 12.1, 4.5, 0.0508)
                .steerPID(STEER_GAINS).drivePID(DRIVE_GAINS)
                .currentLimits(40, 35, 20, 120)
                .build());
    }

    @Test
    void rejectsMissingPIDGains() {
        assertThrows(IllegalStateException.class, () -> validBuilder().steerPID(null).build());
        assertThrows(IllegalStateException.class, () -> validBuilder().drivePID(null).build());
    }

    @Test
    void rejectsMissingCurrentLimits() {
        assertThrows(IllegalStateException.class, () -> DrivetrainConfig.builder()
                .canBus("CANdace")
                .pigeonId(24)
                .frontLeft(FL).frontRight(FR).backLeft(BL).backRight(BR)
                .gearing(6.39, 12.1, 4.5, 0.0508)
                .speed(4.99, 4.71)
                .steerPID(STEER_GAINS).drivePID(DRIVE_GAINS)
                .build());
    }

    // --- Invalid values ---

    @Test
    void rejectsPigeonIdOutOfRange() {
        assertThrows(IllegalArgumentException.class, () -> validBuilder().pigeonId(-1));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().pigeonId(63));
    }

    @Test
    void rejectsInvalidGearing() {
        assertThrows(IllegalArgumentException.class, () -> validBuilder().gearing(0, 12.1, 4.5, 2.0));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().gearing(6.39, 0, 4.5, 2.0));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().gearing(6.39, 12.1, -1, 2.0));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().gearing(6.39, 12.1, 4.5, 0));
    }

    @Test
    void rejectsInvalidSpeed() {
        assertThrows(IllegalArgumentException.class, () -> validBuilder().speed(0, 4.71));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().speed(4.99, 0));
    }

    @Test
    void rejectsInvalidCurrentLimits() {
        assertThrows(IllegalArgumentException.class, () -> validBuilder().currentLimits(0, 35, 20, 120));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().currentLimits(40, 0, 20, 120));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().currentLimits(40, 35, 0, 120));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().currentLimits(40, 35, 20, 0));
    }

    @Test
    void rejectsInvalidDeadband() {
        assertThrows(IllegalArgumentException.class, () -> validBuilder().deadband(-0.1, 0.1));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().deadband(1.1, 0.1));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().deadband(0.1, -0.1));
        assertThrows(IllegalArgumentException.class, () -> validBuilder().deadband(0.1, 1.1));
    }

    // --- CTRE object construction ---

    @Test
    void buildsDrivetrainConstants() {
        DrivetrainConfig config = validBuilder().build();
        SwerveDrivetrainConstants constants = config.toSwerveDrivetrainConstants();
        assertEquals("CANdace", constants.CANBusName);
        assertEquals(24, constants.Pigeon2Id);
    }

    @Test
    void buildsModuleConstantsFactory() {
        DrivetrainConfig config = validBuilder().build();
        var factory = config.toModuleConstantsFactory();
        assertNotNull(factory);
    }

    @Test
    void createsModuleConstants() {
        DrivetrainConfig config = validBuilder().build();
        SwerveModuleConstants<?, ?, ?> mc = config.createModuleConstants(FL);
        assertEquals(7, mc.DriveMotorId);
        assertEquals(8, mc.SteerMotorId);
        assertEquals(23, mc.EncoderId);
        assertEquals(0.124, mc.EncoderOffset, 0.001);
        assertEquals(0.2921, mc.LocationX, 0.001);
        assertEquals(0.2921, mc.LocationY, 0.001);
        assertFalse(mc.DriveMotorInverted);
        assertFalse(mc.SteerMotorInverted);
        assertFalse(mc.EncoderInverted);
    }

    @Test
    void createsModuleConstantsWithInversion() {
        DrivetrainConfig config = validBuilder().build();
        SwerveModuleConstants<?, ?, ?> mc = config.createModuleConstants(FR);
        assertEquals(1, mc.DriveMotorId);
        assertEquals(2, mc.SteerMotorId);
        assertEquals(20, mc.EncoderId);
        assertTrue(mc.DriveMotorInverted);
    }

    @Test
    void acceptsBoundaryPigeonId() {
        assertDoesNotThrow(() -> validBuilder().pigeonId(0).build());
        assertDoesNotThrow(() -> validBuilder().pigeonId(62).build());
    }

    // --- Deadband defaults to zero if not set ---

    @Test
    void deadbandDefaultsToZero() {
        DrivetrainConfig config = DrivetrainConfig.builder()
                .canBus("CANdace")
                .pigeonId(24)
                .frontLeft(FL).frontRight(FR).backLeft(BL).backRight(BR)
                .gearing(6.39, 12.1, 4.5, 0.0508)
                .speed(4.99, 4.71)
                .steerPID(STEER_GAINS).drivePID(DRIVE_GAINS)
                .currentLimits(40, 35, 20, 120)
                .build();
        assertEquals(0.0, config.translationDeadband);
        assertEquals(0.0, config.rotationDeadband);
    }

    // --- Camera configuration ---

    @Test
    void buildsWithNoCameras() {
        DrivetrainConfig config = validBuilder().build();
        assertNotNull(config.cameras);
        assertTrue(config.cameras.isEmpty());
    }

    @Test
    void buildsWithCameras() {
        Transform3d t1 = new Transform3d(new Translation3d(0.3, 0, 0.5), new Rotation3d());
        Transform3d t2 = new Transform3d(new Translation3d(-0.3, 0, 0.5), new Rotation3d());
        DrivetrainConfig config = validBuilder()
                .camera("front", t1)
                .camera("back", t2)
                .build();
        assertEquals(2, config.cameras.size());
        assertEquals("front", config.cameras.get(0).name());
        assertSame(t1, config.cameras.get(0).robotToCamera());
        assertEquals("back", config.cameras.get(1).name());
        assertSame(t2, config.cameras.get(1).robotToCamera());
    }

    @Test
    void rejectsNullCameraName() {
        Transform3d t = new Transform3d(new Translation3d(), new Rotation3d());
        assertThrows(IllegalArgumentException.class, () -> validBuilder().camera(null, t));
    }

    @Test
    void rejectsBlankCameraName() {
        Transform3d t = new Transform3d(new Translation3d(), new Rotation3d());
        assertThrows(IllegalArgumentException.class, () -> validBuilder().camera("", t));
    }

    @Test
    void rejectsNullCameraTransform() {
        assertThrows(IllegalArgumentException.class, () -> validBuilder().camera("cam", null));
    }

    @Test
    void camerasListIsImmutable() {
        Transform3d t = new Transform3d(new Translation3d(0.3, 0, 0.5), new Rotation3d());
        DrivetrainConfig config = validBuilder().camera("front", t).build();
        assertThrows(UnsupportedOperationException.class, () ->
                config.cameras.add(new CameraConfig("extra", t)));
    }
}
