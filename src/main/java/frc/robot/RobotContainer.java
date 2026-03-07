package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.SwerveDrive;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
    public final SwerveDrive drivetrain;

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final SlewRateLimiter translationXLimiter = new SlewRateLimiter(6.0);
    private final SlewRateLimiter translationYLimiter = new SlewRateLimiter(6.0);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(8.0);

    public RobotContainer(DrivetrainConfig config) {
        drivetrain = new SwerveDrive(config);

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

        // X button = brake (lock wheels in X pattern)
        driverController.x().whileTrue(drivetrain.brake());

        // Right bumper = reset field-centric heading
        driverController.rightBumper().onTrue(
                drivetrain.runOnce(() -> drivetrain.resetHeading()));
    }

    private static double applyInputCurve(double value) {
        return Math.copySign(value * value, value);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
