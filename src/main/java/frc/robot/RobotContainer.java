package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.drivetrain.DriveInterface;

/** Owns subsystems, default commands, button bindings, and auto chooser. */
public class RobotContainer {

  private final DriveInterface m_drive;
  private final Vision m_vision; // null if no camera configured

  private final CommandXboxController m_controller = new CommandXboxController(0);

  private SlewRateLimiter m_xLimiter;
  private SlewRateLimiter m_yLimiter;
  private SlewRateLimiter m_rotLimiter;

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  private final SendableChooser<Pose2d> m_posePresetChooser = new SendableChooser<>();
  private final Field2d m_field = new Field2d();

  public RobotContainer(DriveInterface drive) {
    m_drive = drive;

    if (drive.getConfig().camera != null) {
      m_vision = new Vision(drive.getConfig().camera, m_drive);
    } else {
      m_vision = null;
    }

    DriverPreferences.init();

    m_xLimiter = new SlewRateLimiter(DriverPreferences.accelLimit());
    m_yLimiter = new SlewRateLimiter(DriverPreferences.accelLimit());
    m_rotLimiter = new SlewRateLimiter(DriverPreferences.rotAccelLimit());

    configureDefaultCommand();
    configureButtonBindings();
    configureAutoChooser();
    configurePoseReset();
  }

  private void configureDefaultCommand() {
    m_drive.setDefaultCommand(
        Commands.run(
            () -> {
              double deadband = DriverPreferences.deadband();
              double driveExpo = DriverPreferences.driveExpo();
              double rotExpo = DriverPreferences.rotationExpo();

              // Apply expo curve with deadband
              double xInput = InputProcessing.applyInputCurve(
                  -m_controller.getLeftY(), deadband, driveExpo);
              double yInput = InputProcessing.applyInputCurve(
                  -m_controller.getLeftX(), deadband, driveExpo);
              double rotInput = InputProcessing.applyInputCurve(
                  -m_controller.getRightX(), deadband, rotExpo);

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
    // X button: brake (stop all motors)
    m_controller.x().whileTrue(
        Commands.run(() -> m_drive.drive(0, 0, 0, true, 0.02), m_drive));

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
    // Preset positions
    m_posePresetChooser.setDefaultOption("Origin", new Pose2d(0, 0, new Rotation2d()));
    m_posePresetChooser.addOption("Blue Left", new Pose2d(1.0, 7.0, new Rotation2d()));
    m_posePresetChooser.addOption("Blue Right", new Pose2d(1.0, 1.0, new Rotation2d()));
    m_posePresetChooser.addOption("Red Left", new Pose2d(15.5, 7.0, Rotation2d.fromDegrees(180)));
    m_posePresetChooser.addOption("Red Right", new Pose2d(15.5, 1.0, Rotation2d.fromDegrees(180)));
    m_posePresetChooser.addOption("Blue HUB", new Pose2d(3.5, 4.1, new Rotation2d()));
    m_posePresetChooser.addOption("Red HUB", new Pose2d(13.0, 4.1, Rotation2d.fromDegrees(180)));

    SmartDashboard.putData("Pose/Preset", m_posePresetChooser);
    SmartDashboard.putBoolean("Pose/ApplyPreset", false);

    // Manual X/Y/heading entry
    SmartDashboard.putNumber("Pose/ResetX", 0.0);
    SmartDashboard.putNumber("Pose/ResetY", 0.0);
    SmartDashboard.putNumber("Pose/ResetHeading", 0.0);
    SmartDashboard.putBoolean("Pose/ResetTrigger", false);

    // Vision enable/disable toggle
    SmartDashboard.putBoolean("Vision/Enable", true);

    // Field2d visualization
    SmartDashboard.putData("Field", m_field);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void periodic() {
    // Publish current pose
    Pose2d pose = m_drive.getPose();
    m_field.setRobotPose(pose);
    SmartDashboard.putNumber("Pose/X", pose.getX());
    SmartDashboard.putNumber("Pose/Y", pose.getY());
    SmartDashboard.putNumber("Pose/Heading", pose.getRotation().getDegrees());

    // Vision enable/disable from dashboard
    if (m_vision != null) {
      m_vision.setEnabled(SmartDashboard.getBoolean("Vision/Enable", true));

      // Show vision pose on Field2d for comparison
      if (m_vision.hasTargets()) {
        m_field.getObject("Vision Pose").setPose(m_vision.getLastVisionPose());
      }
    }

    // Dashboard preset reset
    if (SmartDashboard.getBoolean("Pose/ApplyPreset", false)) {
      Pose2d preset = m_posePresetChooser.getSelected();
      if (preset != null) {
        m_drive.resetPose(preset);
      }
      SmartDashboard.putBoolean("Pose/ApplyPreset", false);
    }

    // Dashboard manual reset
    if (SmartDashboard.getBoolean("Pose/ResetTrigger", false)) {
      double x = SmartDashboard.getNumber("Pose/ResetX", 0.0);
      double y = SmartDashboard.getNumber("Pose/ResetY", 0.0);
      double heading = SmartDashboard.getNumber("Pose/ResetHeading", 0.0);
      m_drive.resetPose(new Pose2d(x, y, Rotation2d.fromDegrees(heading)));
      SmartDashboard.putBoolean("Pose/ResetTrigger", false);
    }
  }
}
