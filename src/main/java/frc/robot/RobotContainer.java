package frc.robot;

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

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.lib.drivetrain.DriveInterface;
import frc.lib.drivetrain.SwerveDrive;
import frc.lib.vision.VisionIO;

/** Owns subsystems, default commands, button bindings, and auto chooser. */
public class RobotContainer {

  private final DriveInterface m_drive;
  private final Vision m_vision; // null if no camera configured

  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandXboxController m_sysIdController = new CommandXboxController(1);

  private SlewRateLimiter m_xLimiter;
  private SlewRateLimiter m_yLimiter;
  private SlewRateLimiter m_rotLimiter;

  private final SensitivityTuner m_driveSensitivity =
      new SensitivityTuner("Drive_", 0.09, 0.6, 0.1, 0.3, 0.8);
  private final SensitivityTuner m_rotSensitivity =
      new SensitivityTuner("Rot_", 0.09, 0.6, 0.1, 0.5, 1.0);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<String> m_posePresetChooser = new SendableChooser<>();
  private final Field2d m_field = new Field2d();

  public RobotContainer(DriveInterface drive, VisionIO[] visionIOs) {
    m_drive = drive;

    if (visionIOs.length > 0) {
      m_vision = new Vision(visionIOs, m_drive);
    } else {
      m_vision = null;
    }

    DriverPreferences.init();
    FieldPositions.init();

    m_xLimiter = new SlewRateLimiter(DriverPreferences.accelLimit());
    m_yLimiter = new SlewRateLimiter(DriverPreferences.accelLimit());
    m_rotLimiter = new SlewRateLimiter(DriverPreferences.rotAccelLimit());

    configureDefaultCommand();
    configureButtonBindings();
    configureSysIdBindings();
    configureAutoChooser();
    configurePoseReset();

    // Warmup PathPlanner to avoid latency spike on first pathfinding command
    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureDefaultCommand() {
    m_drive.setDefaultCommand(
        Commands.run(
            () -> {
              // Apply piecewise sensitivity curves (deadzone + two-segment linear)
              double xInput = m_driveSensitivity.transfer(-m_controller.getLeftY());
              double yInput = m_driveSensitivity.transfer(-m_controller.getLeftX());
              double rotInput = m_rotSensitivity.transfer(-m_controller.getRightX());

              // Clamp stick magnitude to unit circle
              double[] clamped = InputProcessing.clampStickMagnitude(xInput, yInput);
              xInput = clamped[0];
              yInput = clamped[1];

              // Slew rate limiting
              xInput = m_xLimiter.calculate(xInput);
              yInput = m_yLimiter.calculate(yInput);
              rotInput = m_rotLimiter.calculate(rotInput);

              // Scale by max speed and driver preferences
              double speedScale = DriverPreferences.maxSpeedScale();
              double rotScale = DriverPreferences.maxRotationScale();

              // Slow mode: right trigger > 50%
              if (m_controller.getRightTriggerAxis() > 0.5) {
                double slowScale = DriverPreferences.slowModeScale();
                speedScale *= slowScale;
                rotScale *= slowScale;
              }

              double xSpeed = xInput * m_drive.getMaxSpeed() * speedScale;
              double ySpeed = yInput * m_drive.getMaxSpeed() * speedScale;
              double rot = rotInput * m_drive.getMaxAngularSpeed() * rotScale;

              m_drive.drive(xSpeed, ySpeed, rot, true, 0.02);
            },
            m_drive));
  }

  private void configureButtonBindings() {
    // Idle motors when robot is disabled
    RobotModeTriggers.disabled().whileTrue(
        Commands.run(() -> m_drive.setIdle(), m_drive));

    // X button: X-pattern brake (hold position)
    m_controller.x().whileTrue(
        Commands.run(() -> m_drive.setBrake(), m_drive));

    // Right bumper: reset heading
    m_controller.rightBumper().onTrue(
        Commands.runOnce(() -> m_drive.resetHeading(), m_drive));

    // Start button: reset pose to origin
    m_controller.start().onTrue(
        Commands.runOnce(() -> m_drive.resetPose(new Pose2d()), m_drive));

    // Y button: drive to nearest AprilTag (only if vision available)
    if (m_vision != null) {
      m_controller.y().whileTrue(new DriveToTag(m_vision, m_drive));
    }

    // D-pad snap-to-angle: hold D-pad to face cardinal heading
    m_controller.povUp().whileTrue(
        Commands.run(() -> snapToAngle(Rotation2d.fromDegrees(0)), m_drive));
    m_controller.povRight().whileTrue(
        Commands.run(() -> snapToAngle(Rotation2d.fromDegrees(270)), m_drive));
    m_controller.povDown().whileTrue(
        Commands.run(() -> snapToAngle(Rotation2d.fromDegrees(180)), m_drive));
    m_controller.povLeft().whileTrue(
        Commands.run(() -> snapToAngle(Rotation2d.fromDegrees(90)), m_drive));
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
      m_autoChooser.addOption("Drive to Nearest Tag",
          Autos.driveToNearestTag(m_vision, m_drive));
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
    double xInput = m_driveSensitivity.transfer(-m_controller.getLeftY());
    double yInput = m_driveSensitivity.transfer(-m_controller.getLeftX());
    double[] clamped = InputProcessing.clampStickMagnitude(xInput, yInput);
    xInput = m_xLimiter.calculate(clamped[0]);
    yInput = m_yLimiter.calculate(clamped[1]);

    double speedScale = DriverPreferences.maxSpeedScale();
    if (m_controller.getRightTriggerAxis() > 0.5) {
      speedScale *= DriverPreferences.slowModeScale();
    }

    m_drive.driveFieldCentricFacingAngle(
        xInput * m_drive.getMaxSpeed() * speedScale,
        yInput * m_drive.getMaxSpeed() * speedScale,
        targetAngle, 0.02);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
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
