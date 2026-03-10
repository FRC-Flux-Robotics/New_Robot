package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.SwerveDrive;
import org.littletonrobotics.junction.Logger;
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
  private final boolean hasTagCamera;

  private final CommandXboxController driverController = new CommandXboxController(0);

  private SlewRateLimiter translationXLimiter;
  private SlewRateLimiter translationYLimiter;
  private SlewRateLimiter rotationLimiter;
  private double currentAccelLimit;
  private double currentRotAccelLimit;
  private boolean slowMode = false;

  // Cached DriverPreferences values (refreshed once per cycle)
  private double cachedDeadband;
  private double cachedDriveExpo;
  private double cachedRotationExpo;
  private double cachedMaxSpeedScale;
  private double cachedMaxRotationScale;
  private double cachedSlowModeScale;
  private double cachedAccelLimit;
  private double cachedRotAccelLimit;
  private long cachedPrefsFrame = -1;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer(DrivetrainConfig config, AprilTagFieldLayout fieldLayout) {
    DriverPreferences.init();

    currentAccelLimit = DriverPreferences.accelLimit();
    currentRotAccelLimit = DriverPreferences.rotAccelLimit();
    translationXLimiter = new SlewRateLimiter(currentAccelLimit);
    translationYLimiter = new SlewRateLimiter(currentAccelLimit);
    rotationLimiter = new SlewRateLimiter(currentRotAccelLimit);

    drivetrain = new SwerveDrive(config, fieldLayout);
    drivetrain.setTelemetry(new DriverDashboard());

    hasTagCamera = !config.cameras.isEmpty();

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
            () -> {
              refreshPrefsCache();
              updateSlewRates();
              double speedScale = slowMode ? cachedSlowModeScale : cachedMaxSpeedScale;
              return translationXLimiter.calculate(getClampedStick()[0]) * maxSpeed * speedScale;
            },
            () -> {
              double speedScale = slowMode ? cachedSlowModeScale : cachedMaxSpeedScale;
              return translationYLimiter.calculate(getClampedStick()[1]) * maxSpeed * speedScale;
            },
            () -> {
              double rotScale = slowMode ? cachedSlowModeScale : cachedMaxRotationScale;
              return rotationLimiter.calculate(
                      InputProcessing.applyInputCurve(
                          -driverController.getRightX(), cachedDeadband, cachedRotationExpo))
                  * maxAngularRate
                  * rotScale;
            }));

    // Idle while disabled to apply neutral mode
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Left trigger = aim at target while driving (field-centric facing point)
    driverController
        .leftTrigger(0.5)
        .whileTrue(
            drivetrain.driveFieldCentricFacingPoint(
                () ->
                    translationXLimiter.calculate(getClampedStick()[0])
                        * maxSpeed
                        * cachedMaxSpeedScale,
                () ->
                    translationYLimiter.calculate(getClampedStick()[1])
                        * maxSpeed
                        * cachedMaxSpeedScale,
                () -> SPEAKER_POSITION));

    // Right trigger = slow mode (hold)
    driverController
        .rightTrigger(0.5)
        .onTrue(Commands.runOnce(() -> slowMode = true))
        .onFalse(Commands.runOnce(() -> slowMode = false));

    // A button = drive to and align with tag 3 using vision (hold to keep driving)
    if (hasTagCamera) {
      driverController.a().whileTrue(goToTag());
    }

    // X button = brake (lock wheels in X pattern)
    driverController.x().whileTrue(drivetrain.brake());

    // Emergency stop: both bumpers = brake + report
    driverController
        .leftBumper()
        .and(driverController.rightBumper())
        .whileTrue(
            drivetrain
                .brake()
                .beforeStarting(
                    () -> {
                      DriverStation.reportWarning("EMERGENCY STOP ACTIVATED", false);
                      Logger.recordOutput("Drive/EmergencyStop", true);
                    })
                .finallyDo(() -> Logger.recordOutput("Drive/EmergencyStop", false))
                .withName("EmergencyStop"));

    // Right bumper = reset field-centric heading
    driverController
        .rightBumper()
        .and(driverController.leftBumper().negate())
        .onTrue(drivetrain.runOnce(() -> drivetrain.resetHeading()));

    // SysId characterization: back/start + X/Y (run one routine per log session)
    driverController
        .back()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    driverController
        .back()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    driverController
        .start()
        .and(driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    driverController
        .start()
        .and(driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }

  private final SwerveRequest.RobotCentric goToTagRequest =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double lastTagResultTime = -1;
  private static final double TAG_STALE_TIMEOUT_S = 0.5;

  private Command goToTag() {
    return drivetrain
        .run(
            () -> {
              var latest = drivetrain.getLatestCameraResult(0);
              if (latest == null) {
                if (lastTagResultTime >= 0
                    && Timer.getFPGATimestamp() - lastTagResultTime > TAG_STALE_TIMEOUT_S) {
                  drivetrain.setControl(
                      goToTagRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
                  Logger.recordOutput("Drive/GoToTag/Visible", false);
                }
                return;
              }
              lastTagResultTime = Timer.getFPGATimestamp();

              PhotonTrackedTarget tag = null;
              for (var target : latest.getTargets()) {
                if (target.getFiducialId() == GO_TO_TAG_ID) {
                  tag = target;
                  break;
                }
              }

              if (tag == null) {
                drivetrain.setControl(
                    goToTagRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
                Logger.recordOutput("Drive/GoToTag/Visible", false);
                return;
              }

              Transform3d camToTag = tag.getBestCameraToTarget();
              double forwardDist = camToTag.getX();
              double lateralOff = camToTag.getY();
              double yawRad = Math.toRadians(tag.getYaw());

              double vx =
                  MathUtil.clamp(
                      TAG_FORWARD_KP * (forwardDist - TAG_STOP_DISTANCE_M),
                      -TAG_MAX_SPEED_MPS,
                      TAG_MAX_SPEED_MPS);
              double vy =
                  MathUtil.clamp(
                      -TAG_LATERAL_KP * lateralOff, -TAG_MAX_SPEED_MPS, TAG_MAX_SPEED_MPS);
              double omega =
                  MathUtil.clamp(-TAG_ROTATION_KP * yawRad, -TAG_MAX_OMEGA_RAD, TAG_MAX_OMEGA_RAD);

              if (forwardDist < TAG_STOP_DISTANCE_M) {
                vx = 0;
              }

              Logger.recordOutput("Drive/GoToTag/Visible", true);
              Logger.recordOutput("Drive/GoToTag/DistanceM", forwardDist);
              Logger.recordOutput("Drive/GoToTag/LateralM", lateralOff);
              Logger.recordOutput("Drive/GoToTag/YawDeg", tag.getYaw());

              drivetrain.setControl(
                  goToTagRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
            })
        .withName("GoToTag" + GO_TO_TAG_ID);
  }

  // Cache clamped joystick per cycle to avoid redundant reads and hypot calculations
  private double[] cachedStick = new double[2];
  private long cachedStickFrame = -1;

  private double[] getClampedStick() {
    long frame = Logger.getTimestamp();
    if (frame == cachedStickFrame) return cachedStick;
    cachedStickFrame = frame;
    double x =
        InputProcessing.applyInputCurve(
            -driverController.getLeftY(), cachedDeadband, cachedDriveExpo);
    double y =
        InputProcessing.applyInputCurve(
            -driverController.getLeftX(), cachedDeadband, cachedDriveExpo);
    double[] clamped = InputProcessing.clampStickMagnitude(x, y);
    cachedStick[0] = clamped[0];
    cachedStick[1] = clamped[1];
    return cachedStick;
  }

  /** Refreshes cached DriverPreferences values once per cycle. */
  private void refreshPrefsCache() {
    long frame = Logger.getTimestamp();
    if (frame == cachedPrefsFrame) return;
    cachedPrefsFrame = frame;
    cachedDeadband = DriverPreferences.deadband();
    cachedDriveExpo = DriverPreferences.driveExpo();
    cachedRotationExpo = DriverPreferences.rotationExpo();
    cachedMaxSpeedScale = DriverPreferences.maxSpeedScale();
    cachedMaxRotationScale = DriverPreferences.maxRotationScale();
    cachedSlowModeScale = DriverPreferences.slowModeScale();
    cachedAccelLimit = DriverPreferences.accelLimit();
    cachedRotAccelLimit = DriverPreferences.rotAccelLimit();
  }

  /** Recreates slew rate limiters if the driver changed accel limits on the dashboard. */
  private void updateSlewRates() {
    if (cachedAccelLimit != currentAccelLimit) {
      currentAccelLimit = cachedAccelLimit;
      translationXLimiter = new SlewRateLimiter(cachedAccelLimit);
      translationYLimiter = new SlewRateLimiter(cachedAccelLimit);
    }
    if (cachedRotAccelLimit != currentRotAccelLimit) {
      currentRotAccelLimit = cachedRotAccelLimit;
      rotationLimiter = new SlewRateLimiter(cachedRotAccelLimit);
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
