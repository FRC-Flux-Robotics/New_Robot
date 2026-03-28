package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.drivetrain.DriveInterface;
import frc.lib.mechanism.MechanismConfig;
import frc.lib.mechanism.MechanismIO;
import frc.lib.mechanism.MechanismIODualTalonFX;
import frc.lib.mechanism.MechanismIOReplay;
import frc.lib.mechanism.MechanismIOTalonFX;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.lib.vision.VisionIO;
import frc.robot.MechanismConfigs.IndexerConstants;
import frc.robot.MechanismConfigs.IntakeConstants;
import frc.robot.MechanismConfigs.ShooterConstants;
import frc.robot.commands.*;

/**
 * FUEL robot container — adds mechanism subsystems and operator controller bindings on top of the
 * base RobotContainer drivetrain setup.
 */
public class FuelRobotContainer extends RobotContainer {
  private static final double TRIGGER_THRESHOLD = 0.5;
  private static final double DRIVER_TRIGGER_THRESHOLD = 0.05;

  private final VelocityMechanism intake;
  private final PositionMechanism tilter;
  private final VelocityMechanism indexer;
  private final VelocityMechanism feeder;
  private final VelocityMechanism shooter;
  private final PositionMechanism hood;
  private final RangeTable rangeTable;

  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  public FuelRobotContainer(DriveInterface drive, VisionIO[] visionIOs) {
    super(drive, visionIOs);

    intake = new VelocityMechanism(createIO(MechanismConfigs.INTAKE), MechanismConfigs.INTAKE);
    tilter = new PositionMechanism(createIO(MechanismConfigs.TILT), MechanismConfigs.TILT);
    indexer = new VelocityMechanism(createIO(MechanismConfigs.INDEXER), MechanismConfigs.INDEXER);
    feeder = new VelocityMechanism(createIO(MechanismConfigs.FEEDER), MechanismConfigs.FEEDER);
    shooter = new VelocityMechanism(createIO(MechanismConfigs.SHOOTER), MechanismConfigs.SHOOTER);
    hood = new PositionMechanism(createIO(MechanismConfigs.HOOD), MechanismConfigs.HOOD);

    rangeTable = new RangeTable();

    configureFuelDriverBindings();
    configureFuelBindings();
  }

  // --- Override base class driver bindings (no-ops, replaced by configureFuelDriverBindings) ---
  @Override
  protected void configureDriverYButton() {}

  @Override
  protected void configureDriverBumpers() {}

  @Override
  protected void configureDriverStartButton() {}

  @Override
  protected void configureDriverDPad() {}

  /** Driver controller bindings matching legacy FuelRobotContainer (two-controller mode). */
  private void configureFuelDriverBindings() {
    // Y: reset heading
    m_controller.y().onTrue(Commands.runOnce(() -> getDrive().resetHeading(), getDrive()));

    // Right Trigger: intake IN
    m_controller
        .rightTrigger(DRIVER_TRIGGER_THRESHOLD)
        .whileTrue(new VelocityCmd(intake, () -> IntakeConstants.InSpeed));

    // Right Bumper: intake OUT
    m_controller.rightBumper().whileTrue(new VelocityCmd(intake, () -> -IntakeConstants.OutSpeed));

    // A: deploy intake (4 sequential tilt jogs)
    Command jogIntakeOut =
        Commands.sequence(
            new RunCommand(() -> tilter.jogDown(), tilter), Commands.waitSeconds(0.2));
    m_controller
        .a()
        .onTrue(
            Commands.sequence(
                jogIntakeOut.asProxy(),
                jogIntakeOut.asProxy(),
                jogIntakeOut.asProxy(),
                jogIntakeOut.asProxy()));

    // POV Left/Right: manual intake tilt
    m_controller.povLeft().whileTrue(new RunCommand(() -> tilter.jogDown(), tilter));
    m_controller.povRight().whileTrue(new RunCommand(() -> tilter.jogUp(), tilter));

    // Left Bumper: seed field-centric heading
    m_controller
        .leftBumper()
        .whileTrue(Commands.runOnce(() -> getDrive().resetHeading(), getDrive()));

    // Back: reset encoders
    m_controller.back().onTrue(Commands.runOnce(() -> resetEncoders()));
  }

  private void configureFuelBindings() {
    CommandXboxController controller = m_operatorController;

    // Feeder: Right Trigger (backward = negative speed)
    controller
        .rightTrigger(TRIGGER_THRESHOLD)
        .whileTrue(new VelocityCmd(feeder, () -> -IndexerConstants.FeederSpeed));

    // Indexer: Right Bumper
    controller.rightBumper().whileTrue(new VelocityCmd(indexer, () -> IndexerConstants.Speed));

    // Range shoot: Left Bumper (15s timeout)
    controller
        .leftBumper()
        .whileTrue(
            new RangeShootCmd(
                shooter, hood, feeder, indexer, rangeTable, getDrive()::getPose, 15.0));

    // Shooter range presets: A/B/Y
    controller
        .a()
        .onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.ShortRange));
    controller
        .b()
        .onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.MidRange));
    controller
        .y()
        .onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.LongRange));

    // Shooter toggle: Start
    controller.start().toggleOnTrue(new ShootCommand(shooter, () -> ShooterConstants.Speed));

    // Shooter speed adjust: D-pad left/right + left bumper
    controller
        .povRight()
        .and(controller.leftBumper())
        .whileTrue(
            Commands.runOnce(
                () -> shooter.setVelocity(shooter.getTargetVelocity() + ShooterConstants.SpeedStep),
                shooter));
    controller
        .povLeft()
        .and(controller.leftBumper())
        .whileTrue(
            Commands.runOnce(
                () -> {
                  double newSpeed = shooter.getTargetVelocity() - ShooterConstants.SpeedStep;
                  if (newSpeed <= 0) {
                    shooter.stop();
                  } else {
                    shooter.setVelocity(newSpeed);
                  }
                },
                shooter));

    // Hood jog: D-pad up/down
    controller.povUp().whileTrue(new RunCommand(() -> hood.jogUp(), hood));
    controller.povDown().whileTrue(new RunCommand(() -> hood.jogDown(), hood));
  }

  private static MechanismIO createIO(MechanismConfig config) {
    if (Robot.mode == Robot.Mode.REPLAY) {
      return new MechanismIOReplay();
    }
    return config.secondMotorId >= 0
        ? new MechanismIODualTalonFX(config)
        : new MechanismIOTalonFX(config);
  }

  public void resetEncoders() {
    hood.resetPosition();
    tilter.resetPosition();
  }
}
