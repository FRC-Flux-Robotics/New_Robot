package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DriveInterface;
import frc.lib.mechanism.MechanismConfig;
import frc.lib.mechanism.MechanismIO;
import frc.lib.mechanism.MechanismIODualTalonFX;
import frc.lib.mechanism.MechanismIOReplay;
import frc.lib.mechanism.MechanismIOTalonFX;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.lib.vision.VisionIO;
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

  public FuelRobotContainer(DriveInterface drive, VisionIO[] visionIOs, CameraConfig[] cameras) {
    super(drive, visionIOs, cameras);

    intake = new VelocityMechanism(createIO(MechanismConfigs.INTAKE), MechanismConfigs.INTAKE);
    tilter = new PositionMechanism(createIO(MechanismConfigs.TILT), MechanismConfigs.TILT);
    indexer = new VelocityMechanism(createIO(MechanismConfigs.INDEXER), MechanismConfigs.INDEXER);
    feeder = new VelocityMechanism(createIO(MechanismConfigs.FEEDER), MechanismConfigs.FEEDER);
    shooter = new VelocityMechanism(createIO(MechanismConfigs.SHOOTER), MechanismConfigs.SHOOTER);
    hood = new PositionMechanism(createIO(MechanismConfigs.HOOD), MechanismConfigs.HOOD);

    rangeTable = new RangeTable();

    MechanismTuning.init();
    registerNamedCommands();
    configureFuelDriverBindings();
    configureFuelBindings();
  }

  /**
   * Registers named commands for PathPlanner event markers. These names can be used in the
   * PathPlanner GUI as event markers on paths — PathPlanner will call the corresponding command
   * when the robot reaches that marker during path following.
   */
  private void registerNamedCommands() {
    // Intake: run intake + indexer to collect balls
    NamedCommands.registerCommand(
        "startIntake",
        new VelocityCmd(intake, MechanismTuning::intakeInSpeed)
            .alongWith(new VelocityCmd(indexer, MechanismTuning::indexerSpeed)));

    NamedCommands.registerCommand(
        "stopIntake",
        Commands.runOnce(
            () -> {
              intake.stop();
              indexer.stop();
            },
            intake,
            indexer));

    // Shooter: spin up to tunable speed
    NamedCommands.registerCommand(
        "spinUpShooter", new ShootCommand(shooter, MechanismTuning::shooterSpeed));

    NamedCommands.registerCommand("stopShooter", Commands.runOnce(() -> shooter.stop(), shooter));

    // Feed: run feeder + indexer to push balls into shooter
    NamedCommands.registerCommand(
        "feed",
        new VelocityCmd(feeder, () -> -MechanismTuning.feederSpeed())
            .alongWith(new VelocityCmd(indexer, MechanismTuning::indexerSpeed)));

    // Range presets: set shooter speed + hood angle
    NamedCommands.registerCommand(
        "setShortRange", new SetShooterRangeCmd(shooter, hood, ShooterConstants.ShortRange));
    NamedCommands.registerCommand(
        "setMidRange", new SetShooterRangeCmd(shooter, hood, ShooterConstants.MidRange));
    NamedCommands.registerCommand(
        "setLongRange", new SetShooterRangeCmd(shooter, hood, ShooterConstants.LongRange));

    // Range-based auto-shoot (calculates from pose, 5s timeout for auto use)
    NamedCommands.registerCommand(
        "rangeShoot",
        new RangeShootCmd(shooter, hood, feeder, indexer, rangeTable, getDrive()::getPose, 5.0));

    // Stop all mechanisms
    NamedCommands.registerCommand(
        "stopAll",
        Commands.runOnce(
            () -> {
              intake.stop();
              indexer.stop();
              feeder.stop();
              shooter.stop();
            },
            intake,
            indexer,
            feeder,
            shooter));
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

  @Override
  protected void configureSysIdBindings() {} // Port 1 is operator controller, not SysId

  /** Driver controller bindings matching legacy FuelRobotContainer (two-controller mode). */
  private void configureFuelDriverBindings() {
    // Y: reset heading
    m_controller.y().onTrue(Commands.runOnce(() -> getDrive().resetHeading(), getDrive()));

    // Right Trigger: intake IN
    m_controller
        .rightTrigger(DRIVER_TRIGGER_THRESHOLD)
        .whileTrue(new VelocityCmd(intake, MechanismTuning::intakeInSpeed));

    // Right Bumper: intake OUT
    m_controller
        .rightBumper()
        .whileTrue(new VelocityCmd(intake, () -> -MechanismTuning.intakeOutSpeed()));

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
        .whileTrue(new VelocityCmd(feeder, () -> -MechanismTuning.feederSpeed()));

    // Indexer: Right Bumper
    controller.rightBumper().whileTrue(new VelocityCmd(indexer, MechanismTuning::indexerSpeed));

    // Range shoot: Left Bumper (15s timeout)
    controller
        .leftBumper()
        .whileTrue(
            new RangeShootCmd(
                shooter, hood, feeder, indexer, rangeTable, getDrive()::getPose, 15.0));

    // Shooter range presets: A/B/Y
    controller.a().onTrue(new SetShooterRangeCmd(shooter, hood, ShooterConstants.ShortRange));
    controller.b().onTrue(new SetShooterRangeCmd(shooter, hood, ShooterConstants.MidRange));
    controller.y().onTrue(new SetShooterRangeCmd(shooter, hood, ShooterConstants.LongRange));

    // Shooter toggle: Start
    controller.start().toggleOnTrue(new ShootCommand(shooter, MechanismTuning::shooterSpeed));

    // Shooter speed adjust: D-pad left/right + left bumper
    controller
        .povRight()
        .and(controller.leftBumper())
        .whileTrue(
            Commands.runOnce(
                () ->
                    shooter.setVelocity(
                        shooter.getTargetVelocity() + MechanismTuning.shooterSpeedStep()),
                shooter));
    controller
        .povLeft()
        .and(controller.leftBumper())
        .whileTrue(
            Commands.runOnce(
                () -> {
                  double newSpeed =
                      shooter.getTargetVelocity() - MechanismTuning.shooterSpeedStep();
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

  @Override
  public void periodic() {
    super.periodic();
    MechanismTuning.periodic();
  }

  public void resetEncoders() {
    hood.resetPosition();
    tilter.resetPosition();
  }
}
