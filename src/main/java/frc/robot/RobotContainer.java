package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DriveInterface;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.SwerveDrive;

/** Owns subsystems, default commands, button bindings, and auto chooser. */
public class RobotContainer {

  private final SwerveDrive m_swerveDrive;
  private final DriveInterface m_drive;
  private final TunableDashboard m_dashboard;
  private final Vision m_vision; // null if no camera configured

  private final CommandXboxController m_controller = new CommandXboxController(0);

  private SlewRateLimiter m_xLimiter;
  private SlewRateLimiter m_yLimiter;
  private SlewRateLimiter m_rotLimiter;

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  public RobotContainer(SwerveDrive swerveDrive, DrivetrainConfig config) {
    m_swerveDrive = swerveDrive;
    m_drive = swerveDrive;
    m_dashboard = new TunableDashboard(swerveDrive, config);

    if (config.camera != null) {
      m_vision = new Vision(config.camera, m_drive);
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
  }

  private void configureDefaultCommand() {
    m_swerveDrive.setDefaultCommand(
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
            m_swerveDrive));
  }

  private void configureButtonBindings() {
    // X button: brake (stop all motors)
    m_controller.x().whileTrue(
        Commands.run(() -> m_drive.drive(0, 0, 0, true, 0.02), m_swerveDrive));

    // Right bumper: reset heading
    m_controller.rightBumper().onTrue(
        Commands.runOnce(() -> m_drive.resetHeading(), m_swerveDrive));

    // Y button: drive to nearest AprilTag (only if vision available)
    if (m_vision != null) {
      m_controller.y().whileTrue(new DriveToTag(m_vision, m_swerveDrive, m_drive));
    }
  }

  private void configureAutoChooser() {
    m_autoChooser.setDefaultOption("None", Autos.none());
    m_autoChooser.addOption("Drive Forward", Autos.driveForward(m_swerveDrive, m_drive));
    m_autoChooser.addOption("Forward-Turn-Back", Autos.forwardTurnBack(m_swerveDrive, m_drive));
    if (m_vision != null) {
      m_autoChooser.addOption("Drive to Nearest Tag",
          Autos.driveToNearestTag(m_vision, m_swerveDrive, m_drive));
    }
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void periodic() {
    m_dashboard.periodic();
  }
}
