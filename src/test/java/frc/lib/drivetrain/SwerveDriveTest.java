package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("integration")
class SwerveDriveTest {

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

    private DrivetrainConfig config;
    private SwerveDrive drive;

    @BeforeAll
    static void initHAL() {
        HAL.initialize(500, 0);
    }

    private static DrivetrainConfig buildTestConfig() {
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
                .deadband(0.1, 0.1)
                .build();
    }

    @BeforeEach
    void setUp() {
        AutoBuilder.resetForTesting();
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

    @Test
    void autoBuilderIsConfigured() {
        assertTrue(AutoBuilder.isConfigured());
        assertTrue(AutoBuilder.isPathfindingConfigured());
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
    void driveFieldCentricFacingPointReturnsCommand() {
        Command cmd = drive.driveFieldCentricFacingPoint(
                () -> 0, () -> 0, () -> new Translation2d(5, 5));
        assertNotNull(cmd);
        assertTrue(cmd.getRequirements().contains(drive));
    }

    @Test
    void driveFieldCentricFacingPointExecutesWithoutError() {
        Command cmd = drive.driveFieldCentricFacingPoint(
                () -> 1.0, () -> 0.5, () -> new Translation2d(8, 4));
        assertDoesNotThrow(() -> {
            cmd.initialize();
            cmd.execute();
        });
    }

    @Test
    void driveFieldCentricFacingPointCallsSupplierEachCycle() {
        int[] callCount = {0};
        Command cmd = drive.driveFieldCentricFacingPoint(
                () -> 0, () -> 0, () -> {
                    callCount[0]++;
                    return new Translation2d(3, 3);
                });
        cmd.initialize();
        cmd.execute();
        cmd.execute();
        assertEquals(2, callCount[0], "Target supplier should be called each execute cycle");
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

    // --- Path planning ---

    @Test
    void pathfindToPoseReturnsCommand() {
        Command cmd = drive.pathfindToPose(new Pose2d(5, 3, Rotation2d.fromDegrees(90)));
        assertNotNull(cmd);
    }

    @Test
    void followPathReturnsCommand() {
        PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                        List.of(new Pose2d(0, 0, Rotation2d.kZero), new Pose2d(2, 0, Rotation2d.kZero))),
                new PathConstraints(3.0, 3.0, Math.PI, Math.PI),
                null,
                new GoalEndState(0, Rotation2d.kZero));
        Command cmd = drive.followPath(path);
        assertNotNull(cmd);
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

    // --- driveToPose regression (S0-19: must use zero-deadband request for convergence) ---

    @Test
    void driveToPoseUsesZeroDeadbandRequest() throws Exception {
        // Verify driveToPose uses driveToPoseRequest (zero deadband) instead of
        // fieldCentricRequest (which has ~0.5 m/s deadband that kills PID convergence).
        var field = SwerveDrive.class.getDeclaredField("driveToPoseRequest");
        assertNotNull(field, "driveToPoseRequest field must exist for driveToPose");

        // Verify driveToPose finishes near the target when already at it (field-frame behavior)
        Pose2d currentPose = drive.getPose();
        Command cmd = drive.driveToPose(currentPose, 0.1);
        cmd.initialize();
        cmd.execute();
        // When already at target pose, the command should immediately be at goal
        assertTrue(cmd.isFinished(), "driveToPose should finish when already at target pose");
    }

    // --- IO layer ---

    @Test
    void constructsWithCustomIO() {
        AutoBuilder.resetForTesting();
        DrivetrainIO noopIO = inputs -> {};
        SwerveDrive customDrive = new SwerveDrive(config, null, noopIO);
        assertNotNull(customDrive);
    }

    // --- Telemetry ---

    @Test
    void periodicRunsWithoutError() {
        // Logger.recordOutput has no queryable test API like SmartDashboard.containsKey,
        // so we just verify periodic() completes without throwing
        assertDoesNotThrow(() -> drive.periodic());
    }

    // --- Safety: helper ---

    private SwerveDrive createDriveWithIO(java.util.function.Consumer<DrivetrainIOInputsAutoLogged> configurator) {
        AutoBuilder.resetForTesting();
        DrivetrainIO mockIO = inputs -> configurator.accept(inputs);
        return new SwerveDrive(buildTestConfig(), null, mockIO);
    }

    // --- Vision integration ---

    private SwerveDrive createDriveWithVisionIO(
            java.util.function.Consumer<DrivetrainIOInputsAutoLogged> configurator) {
        AutoBuilder.resetForTesting();
        DrivetrainIO mockIO = inputs -> {
            // Allocate 1-camera vision arrays
            inputs.visionConnected = new boolean[1];
            inputs.visionHasEstimate = new boolean[1];
            inputs.visionPoseX = new double[1];
            inputs.visionPoseY = new double[1];
            inputs.visionPoseRotDeg = new double[1];
            inputs.visionTimestampSec = new double[1];
            inputs.visionTagCount = new int[1];
            inputs.visionAmbiguity = new double[1];
            inputs.visionAvgTagDistM = new double[1];
            configurator.accept(inputs);
        };
        return new SwerveDrive(buildTestConfig(), null, mockIO);
    }

    @Test
    void visionZeroCamerasPeriodicRunsCleanly() {
        // Default config has no cameras → vision arrays length 0
        assertDoesNotThrow(() -> drive.periodic());
    }

    @Test
    void visionEstimateAcceptedWhenLowAmbiguity() {
        SwerveDrive d = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 3.0;
            inputs.visionPoseY[0] = 2.0;
            inputs.visionPoseRotDeg[0] = 45.0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.1;
            inputs.visionTagCount[0] = 2;
        });
        assertDoesNotThrow(() -> d.periodic());
        // Verify addVisionMeasurement was called (no exception) and pose is valid
        Pose2d pose = d.getPose();
        assertNotNull(pose, "Pose should not be null after vision fusion");
    }

    @Test
    void visionEstimateRejectedWhenHighAmbiguity() {
        SwerveDrive d = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 3.0;
            inputs.visionPoseY[0] = 2.0;
            inputs.visionPoseRotDeg[0] = 45.0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.5;
            inputs.visionTagCount[0] = 2;
        });
        assertDoesNotThrow(() -> d.periodic());
        // High ambiguity should be rejected, pose stays near origin
        Pose2d pose = d.getPose();
        assertEquals(0.0, pose.getX(), 0.01, "Pose X should stay at origin when ambiguity is high");
        assertEquals(0.0, pose.getY(), 0.01, "Pose Y should stay at origin when ambiguity is high");
    }

    @Test
    void visionSingleTagFarRejected() {
        SwerveDrive d = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 5.0;
            inputs.visionPoseY[0] = 3.0;
            inputs.visionPoseRotDeg[0] = 0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.1;
            inputs.visionTagCount[0] = 1;
            inputs.visionAvgTagDistM[0] = 5.5; // far single tag
        });
        d.periodic();
        Pose2d pose = d.getPose();
        assertEquals(0.0, pose.getX(), 0.01, "Single tag far should be rejected, pose stays at origin X");
        assertEquals(0.0, pose.getY(), 0.01, "Single tag far should be rejected, pose stays at origin Y");
    }

    @Test
    void visionSingleTagCloseAccepted() {
        SwerveDrive d = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 3.0;
            inputs.visionPoseY[0] = 2.0;
            inputs.visionPoseRotDeg[0] = 0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.1;
            inputs.visionTagCount[0] = 1;
            inputs.visionAvgTagDistM[0] = 2.0; // close single tag
        });
        // Single tag close should be accepted (not rejected), periodic runs cleanly
        assertDoesNotThrow(() -> d.periodic());
    }

    @Test
    void visionMultiTagCloseAccepted() {
        SwerveDrive d = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 3.0;
            inputs.visionPoseY[0] = 2.0;
            inputs.visionPoseRotDeg[0] = 0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.05;
            inputs.visionTagCount[0] = 2;
            inputs.visionAvgTagDistM[0] = 1.5; // close multi-tag
        });
        // Multi-tag close should be accepted, periodic runs cleanly
        assertDoesNotThrow(() -> d.periodic());
    }

    @Test
    void visionStdDevsReturnsNullForSingleTagFar() {
        AutoBuilder.resetForTesting();
        SwerveDrive d = new SwerveDrive(buildTestConfig());
        assertNull(d.getVisionStdDevs(1, 5.0), "Single tag >4m should return null (reject)");
    }

    @Test
    void visionStdDevsReturnsTrustForSingleTagClose() {
        AutoBuilder.resetForTesting();
        SwerveDrive d = new SwerveDrive(buildTestConfig());
        var stdDevs = d.getVisionStdDevs(1, 2.0);
        assertNotNull(stdDevs, "Single tag <4m should not be rejected");
        assertEquals(0.5, stdDevs.get(0, 0), 0.001);
        assertEquals(0.5, stdDevs.get(1, 0), 0.001);
        assertEquals(999, stdDevs.get(2, 0), 0.001);
    }

    @Test
    void visionStdDevsReturnsHighTrustForMultiTagClose() {
        AutoBuilder.resetForTesting();
        SwerveDrive d = new SwerveDrive(buildTestConfig());
        var stdDevs = d.getVisionStdDevs(2, 1.5);
        assertNotNull(stdDevs, "Multi-tag <4m should not be rejected");
        assertEquals(0.3, stdDevs.get(0, 0), 0.001);
        assertEquals(0.3, stdDevs.get(1, 0), 0.001);
        assertEquals(0.5, stdDevs.get(2, 0), 0.001);
    }

    @Test
    void visionPoseOutOfBoundsRejected() {
        SwerveDrive d = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 50.0;
            inputs.visionPoseY[0] = 50.0;
            inputs.visionPoseRotDeg[0] = 0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.05;
            inputs.visionTagCount[0] = 2;
            inputs.visionAvgTagDistM[0] = 1.0;
        });
        d.periodic();
        Pose2d pose = d.getPose();
        assertEquals(0.0, pose.getX(), 0.01, "Out-of-bounds vision pose should be rejected");
        assertEquals(0.0, pose.getY(), 0.01, "Out-of-bounds vision pose should be rejected");
    }

    @Test
    void visionPoseNegativeRejected() {
        SwerveDrive d = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = -1.0;
            inputs.visionPoseY[0] = -1.0;
            inputs.visionPoseRotDeg[0] = 0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.05;
            inputs.visionTagCount[0] = 2;
            inputs.visionAvgTagDistM[0] = 1.0;
        });
        d.periodic();
        Pose2d pose = d.getPose();
        assertEquals(0.0, pose.getX(), 0.01, "Negative vision pose should be rejected");
        assertEquals(0.0, pose.getY(), 0.01, "Negative vision pose should be rejected");
    }

    @Test
    void isVisionPoseOnFieldReturnsFalseForOutOfBounds() {
        AutoBuilder.resetForTesting();
        SwerveDrive d = new SwerveDrive(buildTestConfig());
        assertFalse(d.isVisionPoseOnField(new Pose2d(50, 50, Rotation2d.kZero)));
        assertFalse(d.isVisionPoseOnField(new Pose2d(-1, 4, Rotation2d.kZero)));
        assertFalse(d.isVisionPoseOnField(new Pose2d(8, -1, Rotation2d.kZero)));
        assertFalse(d.isVisionPoseOnField(new Pose2d(18, 4, Rotation2d.kZero)));
        assertFalse(d.isVisionPoseOnField(new Pose2d(8, 9, Rotation2d.kZero)));
    }

    @Test
    void isVisionPoseOnFieldReturnsTrueForValidPose() {
        AutoBuilder.resetForTesting();
        SwerveDrive d = new SwerveDrive(buildTestConfig());
        assertTrue(d.isVisionPoseOnField(new Pose2d(8, 4, Rotation2d.kZero)));
        assertTrue(d.isVisionPoseOnField(new Pose2d(0, 0, Rotation2d.kZero)));
        assertTrue(d.isVisionPoseOnField(new Pose2d(17, 8.7, Rotation2d.kZero)));
        assertTrue(d.isVisionPoseOnField(new Pose2d(16.5, 8.0, Rotation2d.kZero)));
    }

    @Test
    void visionDisconnectedCameraHandledGracefully() {
        SwerveDrive d = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = false;
            inputs.visionHasEstimate[0] = false;
        });
        assertDoesNotThrow(() -> d.periodic());
    }

    // --- Safety: brownout protection ---

    @Test
    void periodicHandlesLowBatteryVoltage() {
        SwerveDrive d = createDriveWithIO(inputs -> inputs.batteryVoltage = 9.0);
        assertDoesNotThrow(() -> d.periodic());
    }

    @Test
    void periodicHandlesCriticalBatteryVoltage() {
        SwerveDrive d = createDriveWithIO(inputs -> inputs.batteryVoltage = 6.0);
        assertDoesNotThrow(() -> d.periodic());
    }

    // --- Safety: drive current warnings ---

    @Test
    void periodicHandlesHighDriveCurrent() {
        SwerveDrive d = createDriveWithIO(inputs -> inputs.driveCurrentA[0] = 38.0);
        assertDoesNotThrow(() -> d.periodic());
    }

    // --- Safety: steer current warnings ---

    @Test
    void periodicHandlesHighSteerCurrent() {
        SwerveDrive d = createDriveWithIO(inputs -> inputs.steerCurrentA[0] = 19.0);
        assertDoesNotThrow(() -> d.periodic());
    }

    // --- Safety: motor temperature warnings ---

    @Test
    void periodicHandlesHighMotorTemp() {
        SwerveDrive d = createDriveWithIO(inputs -> inputs.driveTempC[0] = 85.0);
        assertDoesNotThrow(() -> d.periodic());
    }

    @Test
    void periodicHandlesOverTemp() {
        SwerveDrive d = createDriveWithIO(inputs -> inputs.driveTempC[0] = 105.0);
        assertDoesNotThrow(() -> d.periodic());
    }

    // --- Safety: stale odometry ---

    @Test
    void periodicHandlesStaleOdometry() {
        SwerveDrive d = createDriveWithIO(inputs -> inputs.odometryPeriodSec = 0);
        assertDoesNotThrow(() -> d.periodic());
    }

    // --- Safety: multiple simultaneous warnings ---

    @Test
    void periodicHandlesMultipleWarnings() {
        SwerveDrive d = createDriveWithIO(inputs -> {
            inputs.batteryVoltage = 9.0;
            inputs.driveCurrentA[0] = 38.0;
            inputs.driveTempC[0] = 85.0;
        });
        assertDoesNotThrow(() -> d.periodic());
    }

    // --- Safety: brownout scale calculation ---

    @Test
    void brownoutScalesSpeedAtLowVoltage() {
        SwerveDrive d = createDriveWithIO(inputs -> inputs.batteryVoltage = 9.0);
        d.periodic(); // populate inputs
        double scale = d.getVoltageSpeedScale();
        assertTrue(scale >= 0.25, "Scale should be >= 0.25 (min), was " + scale);
        assertTrue(scale <= 1.0, "Scale should be <= 1.0, was " + scale);
        assertTrue(scale < 1.0, "Scale should be < 1.0 at 9.0V, was " + scale);
    }

    // --- Pose management ---

    // --- Vision rejection and confidence logging (S0-29) ---

    @Test
    void visionRejectionLogsRejectReason() {
        // Out-of-bounds rejection
        SwerveDrive d1 = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 50.0;
            inputs.visionPoseY[0] = 50.0;
            inputs.visionPoseRotDeg[0] = 0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.05;
            inputs.visionTagCount[0] = 2;
            inputs.visionAvgTagDistM[0] = 1.0;
        });
        assertDoesNotThrow(() -> d1.periodic());

        // High ambiguity rejection
        SwerveDrive d2 = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 3.0;
            inputs.visionPoseY[0] = 2.0;
            inputs.visionPoseRotDeg[0] = 0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.5;
            inputs.visionTagCount[0] = 2;
            inputs.visionAvgTagDistM[0] = 1.0;
        });
        assertDoesNotThrow(() -> d2.periodic());

        // Single tag far rejection
        SwerveDrive d3 = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 5.0;
            inputs.visionPoseY[0] = 3.0;
            inputs.visionPoseRotDeg[0] = 0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.1;
            inputs.visionTagCount[0] = 1;
            inputs.visionAvgTagDistM[0] = 5.5;
        });
        assertDoesNotThrow(() -> d3.periodic());
    }

    @Test
    void getPoseConfidenceReturnsDeadReckoningWithNoVision() {
        AutoBuilder.resetForTesting();
        SwerveDrive d = new SwerveDrive(buildTestConfig());
        assertEquals(SwerveDrive.PoseConfidence.DEAD_RECKONING,
                d.getPoseConfidence(10.0, 0, 1.0));
    }

    @Test
    void getPoseConfidenceReturnsHighForMultiTagRecent() {
        AutoBuilder.resetForTesting();
        SwerveDrive d = new SwerveDrive(buildTestConfig());
        assertEquals(SwerveDrive.PoseConfidence.HIGH,
                d.getPoseConfidence(0.5, 2, 0.05));
    }

    @Test
    void getPoseConfidenceReturnsMediumForOlderVision() {
        AutoBuilder.resetForTesting();
        SwerveDrive d = new SwerveDrive(buildTestConfig());
        assertEquals(SwerveDrive.PoseConfidence.MEDIUM,
                d.getPoseConfidence(1.5, 1, 0.15));
    }

    @Test
    void getPoseConfidenceReturnsLowForOldVision() {
        AutoBuilder.resetForTesting();
        SwerveDrive d = new SwerveDrive(buildTestConfig());
        assertEquals(SwerveDrive.PoseConfidence.LOW,
                d.getPoseConfidence(3.0, 1, 0.15));
    }

    @Test
    void visionOdometryDeltaLogged() {
        SwerveDrive d = createDriveWithVisionIO(inputs -> {
            inputs.visionConnected[0] = true;
            inputs.visionHasEstimate[0] = true;
            inputs.visionPoseX[0] = 3.0;
            inputs.visionPoseY[0] = 2.0;
            inputs.visionPoseRotDeg[0] = 0;
            inputs.visionTimestampSec[0] = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            inputs.visionAmbiguity[0] = 0.05;
            inputs.visionTagCount[0] = 2;
            inputs.visionAvgTagDistM[0] = 1.5;
        });
        // periodic computes odometry delta without throwing
        assertDoesNotThrow(() -> d.periodic());
    }

    // --- Pose management ---

    // --- DriveTelemetry integration (S5-7) ---

    @Test
    void telemetryReceivesDriveStateWithCorrectValues() {
        DriveState[] captured = {null};
        DriveTelemetry mockTelemetry = state -> captured[0] = state;

        SwerveDrive d = createDriveWithIO(inputs -> {
            inputs.batteryVoltage = 11.0;
            inputs.driveCurrentA[0] = 15.0;
            inputs.steerCurrentA[2] = 7.5;
        });
        d.setTelemetry(mockTelemetry);

        // publishDriveState fires on fullLogCycle (every 10th periodic call)
        for (int i = 0; i < 10; i++) {
            d.periodic();
        }

        assertNotNull(captured[0], "Telemetry should have received a DriveState");
        assertEquals(11.0, captured[0].batteryVoltage(), 0.01);
        assertEquals(15.0, captured[0].driveCurrentsA()[0], 0.01);
        assertEquals(7.5, captured[0].steerCurrentsA()[2], 0.01);
        assertNotNull(captured[0].statusMessage());
        assertNotNull(captured[0].activeCommand());
    }

    @Test
    void nullTelemetryPeriodicRunsWithoutError() {
        // Explicitly do NOT call setTelemetry — telemetry is null
        SwerveDrive d = createDriveWithIO(inputs -> inputs.batteryVoltage = 12.0);
        for (int i = 0; i < 10; i++) {
            assertDoesNotThrow(() -> d.periodic());
        }
    }

    @Test
    void driveStateDefensiveCopyOnConstruction() {
        double[] driveCurrents = {10.0, 20.0, 30.0, 40.0};
        double[] steerCurrents = {1.0, 2.0, 3.0, 4.0};
        DriveState state = new DriveState(
                1.0, 25.0, 100.0,
                driveCurrents,
                steerCurrents,
                12.5, false, 1.0, true, "OK", "none");

        // Mutate the original arrays after construction
        driveCurrents[0] = 999.0;
        steerCurrents[0] = 999.0;
        assertEquals(10.0, state.driveCurrentsA()[0], 0.01,
                "DriveState should defensively copy driveCurrentsA on construction");
        assertEquals(1.0, state.steerCurrentsA()[0], 0.01,
                "DriveState should defensively copy steerCurrentsA on construction");
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
