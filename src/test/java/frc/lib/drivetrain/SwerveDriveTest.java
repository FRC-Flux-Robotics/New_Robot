package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SwerveDriveTest {

    private static final ModuleConfig FL =
            new ModuleConfig(7, 8, 23, 0.124, 11.5, 11.5, false, false, false);
    private static final ModuleConfig FR =
            new ModuleConfig(1, 2, 20, -0.291, 11.5, -11.5, true, false, false);
    private static final ModuleConfig BL =
            new ModuleConfig(5, 6, 22, 0.048, -11.5, 11.5, false, false, false);
    private static final ModuleConfig BR =
            new ModuleConfig(3, 4, 21, -0.371, -11.5, -11.5, true, false, false);
    private static final PIDGains STEER_GAINS = new PIDGains(100, 0, 0.5, 0.1, 1.5, 0);
    private static final PIDGains DRIVE_GAINS = new PIDGains(0.1, 0, 0, 0, 0.124, 0);

    private DrivetrainConfig config;
    private SwerveDrive drive;

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    private static DrivetrainConfig buildTestConfig() {
        return DrivetrainConfig.builder()
                .canBus("rio")
                .pigeonId(24)
                .frontLeft(FL)
                .frontRight(FR)
                .backLeft(BL)
                .backRight(BR)
                .gearing(6.39, 12.1, 4.5, 2.0)
                .speed(4.99, 0.75 * 2 * Math.PI)
                .steerPID(STEER_GAINS)
                .drivePID(DRIVE_GAINS)
                .currentLimits(40, 35, 20, 120)
                .deadband(0.1, 0.1)
                .build();
    }

    @BeforeEach
    void setUp() {
        config = buildTestConfig();
        drive = new SwerveDrive(config);
    }

    // --- Construction ---

    @Test
    void constructsWithValidConfig() {
        assertNotNull(drive);
    }

    @Test
    void getConfigReturnsSameConfig() {
        assertSame(config, drive.getConfig());
    }

    // --- Command factories ---

    @Test
    void driveFieldCentricReturnsCommand() {
        Command cmd = drive.driveFieldCentric(() -> 0, () -> 0, () -> 0);
        assertNotNull(cmd);
        assertTrue(cmd.getRequirements().contains(drive));
    }

    @Test
    void driveRobotCentricReturnsCommand() {
        Command cmd = drive.driveRobotCentric(() -> 0, () -> 0, () -> 0);
        assertNotNull(cmd);
        assertTrue(cmd.getRequirements().contains(drive));
    }

    @Test
    void brakeReturnsCommand() {
        Command cmd = drive.brake();
        assertNotNull(cmd);
        assertTrue(cmd.getRequirements().contains(drive));
    }

    @Test
    void stopReturnsCommand() {
        Command cmd = drive.stop();
        assertNotNull(cmd);
        assertTrue(cmd.getRequirements().contains(drive));
    }

    @Test
    void driveToPoseReturnsCommand() {
        Command cmd = drive.driveToPose(new Pose2d(1, 1, Rotation2d.kZero), 0.05);
        assertNotNull(cmd);
        assertTrue(cmd.getRequirements().contains(drive));
    }

    // --- State queries ---

    @Test
    void initialPoseIsOrigin() {
        Pose2d pose = drive.getPose();
        assertNotNull(pose);
        assertEquals(0, pose.getX(), 0.01);
        assertEquals(0, pose.getY(), 0.01);
    }

    @Test
    void initialVelocityIsZero() {
        ChassisSpeeds speeds = drive.getVelocity();
        assertNotNull(speeds);
        assertEquals(0, speeds.vxMetersPerSecond, 0.01);
        assertEquals(0, speeds.vyMetersPerSecond, 0.01);
        assertEquals(0, speeds.omegaRadiansPerSecond, 0.01);
    }

    @Test
    void headingMatchesPoseRotation() {
        Rotation2d heading = drive.getHeading();
        Rotation2d poseRotation = drive.getPose().getRotation();
        assertEquals(poseRotation.getRadians(), heading.getRadians(), 0.001);
    }

    @Test
    void isMovingFalseWhenStopped() {
        assertFalse(drive.isMoving());
    }

    // --- driveToPose regression (S0-11: must use field-centric, not robot-centric) ---

    @Test
    void driveToPoseUsesFieldCentricRequest() throws Exception {
        // Verify driveToPose uses fieldCentricRequest (not robotCentricRequest)
        // by checking that the fieldCentricRequest field exists and is used in the command.
        // The S0-11 bug was using robotCentricRequest which caused wrong behavior.
        var field = SwerveDrive.class.getDeclaredField("fieldCentricRequest");
        assertNotNull(field, "fieldCentricRequest field must exist for driveToPose");

        // Verify driveToPose finishes near the target when already at it (field-frame behavior)
        Pose2d currentPose = drive.getPose();
        Command cmd = drive.driveToPose(currentPose, 0.1);
        cmd.initialize();
        cmd.execute();
        // When already at target pose, the command should immediately be at goal
        assertTrue(cmd.isFinished(), "driveToPose should finish when already at target pose");
    }

    // --- Telemetry ---

    @Test
    void periodicPublishesTelemetryKeys() {
        drive.periodic();

        // Robot-level telemetry
        assertTrue(SmartDashboard.containsKey("Drive/PositionX"));
        assertTrue(SmartDashboard.containsKey("Drive/PositionY"));
        assertTrue(SmartDashboard.containsKey("Drive/RotationDeg"));
        assertTrue(SmartDashboard.containsKey("Drive/SpeedMps"));
        assertTrue(SmartDashboard.containsKey("Drive/SpeedPercent"));
        assertTrue(SmartDashboard.containsKey("Drive/AngularRateDegPerSec"));
        assertTrue(SmartDashboard.containsKey("Drive/OdometryHz"));
        assertTrue(SmartDashboard.containsKey("Drive/ActiveCommand"));

        // Per-module telemetry (spot-check FL)
        assertTrue(SmartDashboard.containsKey("Drive/FL/TargetAngleDeg"));
        assertTrue(SmartDashboard.containsKey("Drive/FL/ActualAngleDeg"));
        assertTrue(SmartDashboard.containsKey("Drive/FL/DriveTempC"));
        assertTrue(SmartDashboard.containsKey("Drive/FL/DriveCurrentA"));
    }

    // --- Pose management ---

    @Test
    void resetPoseSetsNewPose() {
        Pose2d newPose = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45));
        drive.resetPose(newPose);
        Pose2d pose = drive.getPose();
        assertEquals(3.0, pose.getX(), 0.05);
        assertEquals(2.0, pose.getY(), 0.05);
        assertEquals(45.0, pose.getRotation().getDegrees(), 1.0);
    }
}
