package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DriveInterface;
import frc.lib.drivetrain.SwerveDrive;
import frc.lib.vision.VisionIO;

/** Owns subsystems, default commands, button bindings, and auto chooser. */
public class RobotContainer {

  // Legacy max angular rate: 0.75 rotations/sec = 4.712 rad/s
  private static final double LEGACY_MAX_ANGULAR_RATE = 0.75 * 2 * Math.PI;

  private final DriveInterface m_drive;
  private final Vision m_vision; // null if no camera configured

  protected final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandXboxController m_sysIdController = new CommandXboxController(1);

  // Slew rate limiters (used when Drive/SlewRate is ON)
  private SlewRateLimiter m_xLimiter;
  private SlewRateLimiter m_yLimiter;
  private SlewRateLimiter m_rotLimiter;
  private boolean m_prevSlewEnabled = false;

  // Dashboard-tunable sensitivity (used when Drive/TunableSens is ON)
  private final SensitivityTuner m_driveSensitivity =
      new SensitivityTuner("Drive_", 0.09, 0.6, 0.1, 0.3, 0.8);
  private final SensitivityTuner m_rotSensitivity =
      new SensitivityTuner("Rot_", 0.09, 0.6, 0.1, 0.5, 1.0);

  // Fixed sensitivity matching legacy (used when Drive/TunableSens is OFF)
  private final PiecewiseSensitivity m_legacyDriveSens =
      new PiecewiseSensitivity(0.09, 0.6, 0.1, 0.3, 0.8);
  private final PiecewiseSensitivity m_legacyRotSens =
      new PiecewiseSensitivity(0.09, 0.6, 0.1, 0.5, 1.0);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<String> m_posePresetChooser = new SendableChooser<>();
  private final Field2d m_field = new Field2d();

  private double m_lastDeadband = 0.1;

  public RobotContainer(DriveInterface drive, VisionIO[] visionIOs, CameraConfig[] cameras) {
    m_drive = drive;

    if (visionIOs.length > 0) {
      m_vision = new Vision(visionIOs, cameras, m_drive);
    } else {
      m_vision = null;
    }

    DriverPreferences.init();
    FieldPositions.init();

    m_xLimiter = new SlewRateLimiter(DriverPreferences.accelLimit());
    m_yLimiter = new SlewRateLimiter(DriverPreferences.accelLimit());
    m_rotLimiter = new SlewRateLimiter(DriverPreferences.rotAccelLimit());

    configureDriveStyle();
    configureDefaultCommand();
    configureButtonBindings();
    configureSysIdBindings();
    configureAutoChooser();
    configurePoseReset();

    // Apply initial deadband (legacy 10%)
    m_drive.setDeadband(0.1, 0.1);

    // Warmup PathPlanner to avoid latency spike on first pathfinding command
    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureDriveStyle() {
    // All defaults match legacy behavior (OFF / legacy values)
    SmartDashboard.putBoolean("Drive/SlewRate", false);
    SmartDashboard.putBoolean("Drive/SlowMode", false);
    SmartDashboard.putBoolean("Drive/StickClamp", false);
    SmartDashboard.putBoolean("Drive/TunableSens", false);
    SmartDashboard.putBoolean("Drive/SnapToAngle", false);
    SmartDashboard.putNumber("Drive/Deadband", 0.1);
    SmartDashboard.putNumber("Drive/MaxRotRate", LEGACY_MAX_ANGULAR_RATE);
  }

  private void configureDefaultCommand() {
    m_drive.setDefaultCommand(
        Commands.run(
            () -> {
              boolean tunableSens = SmartDashboard.getBoolean("Drive/TunableSens", false);
              boolean slewEnabled = SmartDashboard.getBoolean("Drive/SlewRate", false);
              boolean slowMode = SmartDashboard.getBoolean("Drive/SlowMode", false);
              boolean stickClamp = SmartDashboard.getBoolean("Drive/StickClamp", false);
              double maxRotRate =
                  SmartDashboard.getNumber("Drive/MaxRotRate", LEGACY_MAX_ANGULAR_RATE);

              // 1. Sensitivity curves
              double xInput, yInput, rotInput;
              if (tunableSens) {
                xInput = m_driveSensitivity.transfer(-m_controller.getLeftY());
                yInput = m_driveSensitivity.transfer(-m_controller.getLeftX());
                rotInput = m_rotSensitivity.transfer(-m_controller.getRightX());
              } else {
                xInput = m_legacyDriveSens.transfer(-m_controller.getLeftY());
                yInput = m_legacyDriveSens.transfer(-m_controller.getLeftX());
                rotInput = m_legacyRotSens.transfer(-m_controller.getRightX());
              }

              // 2. Stick clamping to unit circle (optional)
              if (stickClamp) {
                double[] clamped = InputProcessing.clampStickMagnitude(xInput, yInput);
                xInput = clamped[0];
                yInput = clamped[1];
              }

              // 3. Slew rate limiting (optional, reset on enable transition)
              if (slewEnabled) {
                if (!m_prevSlewEnabled) {
                  m_xLimiter.reset(0);
                  m_yLimiter.reset(0);
                  m_rotLimiter.reset(0);
                }
                xInput = m_xLimiter.calculate(xInput);
                yInput = m_yLimiter.calculate(yInput);
                rotInput = m_rotLimiter.calculate(rotInput);
              }
              m_prevSlewEnabled = slewEnabled;

              // 4. Speed scaling — slow mode via right trigger (optional)
              double speedScale = 1.0;
              double rotScale = 1.0;
              if (slowMode && m_controller.getRightTriggerAxis() > 0.5) {
                double slowScale = DriverPreferences.slowModeScale();
                speedScale = slowScale;
                rotScale = slowScale;
              }

              // 5. Final velocities
              double xSpeed = xInput * m_drive.getMaxSpeed() * speedScale;
              double ySpeed = yInput * m_drive.getMaxSpeed() * speedScale;
              double rot = rotInput * maxRotRate * rotScale;

              m_drive.drive(xSpeed, ySpeed, rot, true, 0.02);
            },
            m_drive));
  }

  private void configureButtonBindings() {
    // Idle motors when robot is disabled (universal)
    RobotModeTriggers.disabled().whileTrue(Commands.run(() -> m_drive.setIdle(), m_drive));

    // X button: X-pattern brake (universal)
    m_controller.x().whileTrue(Commands.run(() -> m_drive.setBrake(), m_drive));

    // Overridable per-button groups (FuelRobotContainer replaces these)
    configureDriverYButton();
    configureDriverBumpers();
    configureDriverStartButton();
    configureDriverDPad();
  }

  /** Y button — default: DriveToTag if vision, else reset heading. */
  protected void configureDriverYButton() {
    if (m_vision != null) {
      m_controller.y().whileTrue(new DriveToTag(m_vision, m_drive));
    } else {
      m_controller.y().onTrue(Commands.runOnce(() -> m_drive.resetHeading(), m_drive));
    }
  }

  /** Bumpers — default: both reset heading. */
  protected void configureDriverBumpers() {
    m_controller.rightBumper().onTrue(Commands.runOnce(() -> m_drive.resetHeading(), m_drive));
    m_controller.leftBumper().onTrue(Commands.runOnce(() -> m_drive.resetHeading(), m_drive));
  }

  /** Start button — default: reset pose to origin. */
  protected void configureDriverStartButton() {
    m_controller.start().onTrue(Commands.runOnce(() -> m_drive.resetPose(new Pose2d()), m_drive));
  }

  /** D-pad — default: snap-to-angle (gated by Drive/SnapToAngle toggle). */
  protected void configureDriverDPad() {
    m_controller
        .povUp()
        .and(() -> SmartDashboard.getBoolean("Drive/SnapToAngle", false))
        .whileTrue(Commands.run(() -> snapToAngle(Rotation2d.fromDegrees(0)), m_drive));
    m_controller
        .povRight()
        .and(() -> SmartDashboard.getBoolean("Drive/SnapToAngle", false))
        .whileTrue(Commands.run(() -> snapToAngle(Rotation2d.fromDegrees(270)), m_drive));
    m_controller
        .povDown()
        .and(() -> SmartDashboard.getBoolean("Drive/SnapToAngle", false))
        .whileTrue(Commands.run(() -> snapToAngle(Rotation2d.fromDegrees(180)), m_drive));
    m_controller
        .povLeft()
        .and(() -> SmartDashboard.getBoolean("Drive/SnapToAngle", false))
        .whileTrue(Commands.run(() -> snapToAngle(Rotation2d.fromDegrees(90)), m_drive));
  }

  private void configureSysIdBindings() {
    if (!(m_drive instanceof SwerveDrive swerve)) return;

    // SysId controller (port 1): A/B = dynamic, X/Y = quasistatic
    m_sysIdController.a().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_sysIdController.b().whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    m_sysIdController.x().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_sysIdController.y().whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    // Left bumper: wheel radius characterization (spin in place, measure radius)
    m_sysIdController.leftBumper().whileTrue(swerve.wheelRadiusCharacterization());
  }

  private void configureAutoChooser() {
    m_autoChooser.setDefaultOption("None", Autos.none());
    m_autoChooser.addOption("Drive Forward", Autos.driveForward(m_drive));
    m_autoChooser.addOption("Forward-Turn-Back", Autos.forwardTurnBack(m_drive));
    m_autoChooser.addOption("PathPlanner Test", Autos.pathPlannerTest(m_drive));
    m_autoChooser.addOption("Precision Square", Autos.precisionSquare(m_drive));
    if (m_vision != null) {
      m_autoChooser.addOption("Drive to Nearest Tag", Autos.driveToNearestTag(m_vision, m_drive));
    }
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void configurePoseReset() {
    // Alliance-neutral preset positions — resolved for current alliance at apply time
    m_posePresetChooser.setDefaultOption("Origin", "Origin");
    m_posePresetChooser.addOption("Left", "Left");
    m_posePresetChooser.addOption("Right", "Right");
    m_posePresetChooser.addOption("HUB", "HUB");

    SmartDashboard.putData("Pose/Preset", m_posePresetChooser);
    SmartDashboard.putBoolean("Pose/ApplyPreset", false);

    // Manual X/Y/heading entry (alliance-relative by default)
    SmartDashboard.putNumber("Pose/ResetX", 0.0);
    SmartDashboard.putNumber("Pose/ResetY", 0.0);
    SmartDashboard.putNumber("Pose/ResetHeading", 0.0);
    SmartDashboard.putBoolean("Pose/AllianceRelative", true);
    SmartDashboard.putBoolean("Pose/ResetTrigger", false);

    // Vision enable/disable toggle
    SmartDashboard.putBoolean("Vision/Enable", true);

    // Field2d visualization
    SmartDashboard.putData("Field", m_field);
  }

  private void snapToAngle(Rotation2d targetAngle) {
    boolean tunableSens = SmartDashboard.getBoolean("Drive/TunableSens", false);
    double xInput, yInput;
    if (tunableSens) {
      xInput = m_driveSensitivity.transfer(-m_controller.getLeftY());
      yInput = m_driveSensitivity.transfer(-m_controller.getLeftX());
    } else {
      xInput = m_legacyDriveSens.transfer(-m_controller.getLeftY());
      yInput = m_legacyDriveSens.transfer(-m_controller.getLeftX());
    }

    if (SmartDashboard.getBoolean("Drive/StickClamp", false)) {
      double[] clamped = InputProcessing.clampStickMagnitude(xInput, yInput);
      xInput = clamped[0];
      yInput = clamped[1];
    }

    if (SmartDashboard.getBoolean("Drive/SlewRate", false)) {
      xInput = m_xLimiter.calculate(xInput);
      yInput = m_yLimiter.calculate(yInput);
    }

    double speedScale = 1.0;
    if (SmartDashboard.getBoolean("Drive/SlowMode", false)
        && m_controller.getRightTriggerAxis() > 0.5) {
      speedScale = DriverPreferences.slowModeScale();
    }

    m_drive.driveFieldCentricFacingAngle(
        xInput * m_drive.getMaxSpeed() * speedScale,
        yInput * m_drive.getMaxSpeed() * speedScale,
        targetAngle,
        0.02);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  /** Access the drive subsystem (for subclasses like FuelRobotContainer). */
  protected DriveInterface getDrive() {
    return m_drive;
  }

  public void periodic() {
    // Update operator perspective from alliance (FMS or dashboard)
    m_drive.setOperatorForward(FieldPositions.operatorForward());

    // Publish current pose
    Pose2d pose = m_drive.getPose();
    m_field.setRobotPose(pose);
    SmartDashboard.putNumber("Pose/X", pose.getX());
    SmartDashboard.putNumber("Pose/Y", pose.getY());
    SmartDashboard.putNumber("Pose/Heading", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Battery", RobotController.getBatteryVoltage());

    // Update deadband when dashboard value changes
    double deadband = SmartDashboard.getNumber("Drive/Deadband", 0.1);
    if (deadband != m_lastDeadband) {
      m_drive.setDeadband(deadband, deadband);
      m_lastDeadband = deadband;
    }

    // Vision enable/disable from dashboard
    if (m_vision != null) {
      m_vision.setEnabled(SmartDashboard.getBoolean("Vision/Enable", true));

      // Show vision pose on Field2d for comparison
      if (m_vision.hasTargets()) {
        m_field.getObject("Vision Pose").setPose(m_vision.getLastVisionPose());
      }
    }

    // Dashboard preset reset — resolves alliance-neutral name to correct pose
    if (SmartDashboard.getBoolean("Pose/ApplyPreset", false)) {
      String selected = m_posePresetChooser.getSelected();
      if (selected != null) {
        Pose2d preset = FieldPositions.resolve(selected);
        if (preset != null) {
          m_drive.resetPose(preset);
        }
      }
      SmartDashboard.putBoolean("Pose/ApplyPreset", false);
    }

    // Dashboard manual reset — optionally alliance-relative
    if (SmartDashboard.getBoolean("Pose/ResetTrigger", false)) {
      double x = SmartDashboard.getNumber("Pose/ResetX", 0.0);
      double y = SmartDashboard.getNumber("Pose/ResetY", 0.0);
      double heading = SmartDashboard.getNumber("Pose/ResetHeading", 0.0);
      Pose2d resetPose = new Pose2d(x, y, Rotation2d.fromDegrees(heading));
      if (SmartDashboard.getBoolean("Pose/AllianceRelative", true)) {
        resetPose = FieldPositions.forAlliance(resetPose);
      }
      m_drive.resetPose(resetPose);
      SmartDashboard.putBoolean("Pose/ResetTrigger", false);
    }
  }
}
