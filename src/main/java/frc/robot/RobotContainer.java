package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public final CommandSwerveDrivetrain drivetrain;

    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public RobotContainer(RobotConfig config) {
        SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(config.driveCANBus)
                .withPigeon2Id(config.pigeonId)
                .withPigeon2Configs(config.pigeonConfigs);

        drivetrain = new CommandSwerveDrivetrain(
                config,
                drivetrainConstants,
                createModuleConstants(config.frontLeft),
                createModuleConstants(config.frontRight),
                createModuleConstants(config.backLeft),
                createModuleConstants(config.backRight));

        configureBindings();
    }

    private void configureBindings() {
        // Default command: field-centric driving with left stick (translate) and right stick (rotate)
        // Note: X is forward and Y is left per WPILib convention, joystick Y is inverted
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                        .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));

        // Idle while disabled to apply neutral mode
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // X button = brake (lock wheels in X pattern)
        driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));

        // Right bumper = reset field-centric heading
        driverController.rightBumper().onTrue(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            createModuleConstants(SwerveModuleConfig config) {
        return TunerConstants.ConstantCreator.createModuleConstants(
                config.steerMotorId,
                config.driveMotorId,
                config.encoderId,
                config.encoderOffset,
                config.xPos,
                config.yPos,
                config.invertSide,
                config.steerMotorInverted,
                config.encoderInverted);
    }
}
