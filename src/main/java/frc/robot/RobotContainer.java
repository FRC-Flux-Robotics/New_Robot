package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.SwerveDrive;
import frc.robot.Constants.OperatorConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class RobotContainer {
    // TODO: Update with actual 2026 game field coordinates
    private static final Translation2d SPEAKER_POSITION = new Translation2d(0.0, 5.55);

    // Go-to-tag constants
    private static final int GO_TO_TAG_ID = 3;
    private static final double TAG_STOP_DISTANCE_M = 0.10;
    private static final double TAG_FORWARD_KP = 1.5;
    private static final double TAG_LATERAL_KP = 2.0;
    private static final double TAG_ROTATION_KP = 3.0;
    private static final double TAG_MAX_SPEED_MPS = 1.0;
    private static final double TAG_MAX_OMEGA_RAD = 1.5;

    public final SwerveDrive drivetrain;
    private final PhotonCamera tagCamera;

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final SlewRateLimiter translationXLimiter = new SlewRateLimiter(6.0);
    private final SlewRateLimiter translationYLimiter = new SlewRateLimiter(6.0);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(8.0);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer(DrivetrainConfig config, AprilTagFieldLayout fieldLayout) {
        drivetrain = new SwerveDrive(config, fieldLayout);

        // Use first camera from config for tag tracking, or null if none
        if (!config.cameras.isEmpty()) {
            tagCamera = new PhotonCamera(config.cameras.get(0).name());
        } else {
            tagCamera = null;
        }

        autoChooser.setDefaultOption("Drive Forward", Autos.driveForward(drivetrain));
        autoChooser.addOption("Forward Turn Back", Autos.forwardTurnBack(drivetrain));
        autoChooser.addOption("Do Nothing", Commands.none());
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        double maxSpeed = drivetrain.getConfig().maxSpeedMps;
        double maxAngularRate = drivetrain.getConfig().maxAngularRateRadPerSec;

        // Default command: field-centric driving with left stick (translate) and right stick (rotate)
        // Note: X is forward and Y is left per WPILib convention, joystick Y is inverted
        drivetrain.setDefaultCommand(
                drivetrain.driveFieldCentric(
                        () -> translationXLimiter.calculate(
                                applyInputCurve(-driverController.getLeftY())) * maxSpeed,
                        () -> translationYLimiter.calculate(
                                applyInputCurve(-driverController.getLeftX())) * maxSpeed,
                        () -> rotationLimiter.calculate(
                                applyInputCurve(-driverController.getRightX())) * maxAngularRate));

        // Idle while disabled to apply neutral mode
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // Left trigger = aim at target while driving (field-centric facing point)
        driverController.leftTrigger(0.5).whileTrue(
                drivetrain.driveFieldCentricFacingPoint(
                        () -> translationXLimiter.calculate(
                                applyInputCurve(-driverController.getLeftY())) * maxSpeed,
                        () -> translationYLimiter.calculate(
                                applyInputCurve(-driverController.getLeftX())) * maxSpeed,
                        () -> SPEAKER_POSITION));

        // A button = drive to tag 3 using vision (hold to keep driving)
        if (tagCamera != null) {
            driverController.a().whileTrue(goToTag());
        }

        // X button = brake (lock wheels in X pattern)
        driverController.x().whileTrue(drivetrain.brake());

        // Emergency stop: both bumpers = brake + report
        driverController.leftBumper().and(driverController.rightBumper())
                .whileTrue(
                        drivetrain.brake()
                                .beforeStarting(() -> {
                                    DriverStation.reportWarning("EMERGENCY STOP ACTIVATED", false);
                                    Logger.recordOutput("Drive/EmergencyStop", true);
                                })
                                .finallyDo(() -> Logger.recordOutput("Drive/EmergencyStop", false))
                                .withName("EmergencyStop"));

        // Right bumper = reset field-centric heading
        driverController.rightBumper().and(driverController.leftBumper().negate()).onTrue(
                drivetrain.runOnce(() -> drivetrain.resetHeading()));
    }

    private Command goToTag() {
        return drivetrain.driveRobotCentric(
                () -> computeTagDrive()[0],
                () -> computeTagDrive()[1],
                () -> computeTagDrive()[2]
        ).withName("GoToTag" + GO_TO_TAG_ID);
    }

    // Cache per-cycle to avoid querying camera 3 times per loop
    private double[] cachedTagDrive = new double[3];
    private long cachedTagDriveFrame = -1;

    private double[] computeTagDrive() {
        long frame = Logger.getTimestamp();
        if (frame == cachedTagDriveFrame) {
            return cachedTagDrive;
        }
        cachedTagDriveFrame = frame;

        var results = tagCamera.getAllUnreadResults();
        if (results.isEmpty()) {
            cachedTagDrive[0] = 0;
            cachedTagDrive[1] = 0;
            cachedTagDrive[2] = 0;
            return cachedTagDrive;
        }

        var latest = results.get(results.size() - 1);
        PhotonTrackedTarget tag = null;
        for (var target : latest.getTargets()) {
            if (target.getFiducialId() == GO_TO_TAG_ID) {
                tag = target;
                break;
            }
        }

        if (tag == null) {
            cachedTagDrive[0] = 0;
            cachedTagDrive[1] = 0;
            cachedTagDrive[2] = 0;
            Logger.recordOutput("Drive/GoToTag/Visible", false);
            return cachedTagDrive;
        }

        Transform3d camToTag = tag.getBestCameraToTarget();
        double forwardDist = camToTag.getX(); // forward distance in camera frame
        double lateralOff = camToTag.getY();  // positive = tag is left
        double yawRad = Math.toRadians(tag.getYaw()); // positive = tag is left

        // P-control: drive forward until TAG_STOP_DISTANCE_M, center laterally, face tag
        double vx = MathUtil.clamp(TAG_FORWARD_KP * (forwardDist - TAG_STOP_DISTANCE_M),
                -TAG_MAX_SPEED_MPS, TAG_MAX_SPEED_MPS);
        double vy = MathUtil.clamp(TAG_LATERAL_KP * lateralOff,
                -TAG_MAX_SPEED_MPS, TAG_MAX_SPEED_MPS);
        double omega = MathUtil.clamp(TAG_ROTATION_KP * yawRad,
                -TAG_MAX_OMEGA_RAD, TAG_MAX_OMEGA_RAD);

        // Stop forward motion if close enough
        if (forwardDist < TAG_STOP_DISTANCE_M) {
            vx = 0;
        }

        Logger.recordOutput("Drive/GoToTag/Visible", true);
        Logger.recordOutput("Drive/GoToTag/DistanceM", forwardDist);
        Logger.recordOutput("Drive/GoToTag/LateralM", lateralOff);
        Logger.recordOutput("Drive/GoToTag/YawDeg", tag.getYaw());

        cachedTagDrive[0] = vx;
        cachedTagDrive[1] = vy;
        cachedTagDrive[2] = omega;
        return cachedTagDrive;
    }

    private static double applyInputCurve(double value) {
        return Math.copySign(value * value, value);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
