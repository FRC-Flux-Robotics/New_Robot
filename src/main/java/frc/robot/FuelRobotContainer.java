package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
 * FUEL robot container — Scott's controller layout.
 *
 * <p>Driver (port 0): driving + intake. Operator (port 1): mechanisms + shooting.
 */
public class FuelRobotContainer extends RobotContainer {
  private static final double TRIGGER_THRESHOLD = 0.5;
  private static final double DRIVER_TRIGGER_THRESHOLD = 0.05;

  /** Hub position for auto-aim. */
  private static final Translation2d HUB_POSITION = new Translation2d(3.25, 4.05);

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

    NamedCommands.registerCommand(
        "spinUpShooter", new ShootCommand(shooter, MechanismTuning::shooterSpeed));

    NamedCommands.registerCommand("stopShooter", Commands.runOnce(() -> shooter.stop(), shooter));

    NamedCommands.registerCommand(
        "feed",
        new VelocityCmd(feeder, () -> -MechanismTuning.feederSpeed())
            .alongWith(new VelocityCmd(indexer, MechanismTuning::indexerSpeed)));

    NamedCommands.registerCommand(
        "setShortRange", new SetShooterRangeCmd(shooter, hood, ShooterConstants.ShortRange));
    NamedCommands.registerCommand(
        "setMidRange", new SetShooterRangeCmd(shooter, hood, ShooterConstants.MidRange));
    NamedCommands.registerCommand(
        "setLongRange", new SetShooterRangeCmd(shooter, hood, ShooterConstants.LongRange));

    NamedCommands.registerCommand(
        "rangeShoot",
        new RangeShootCmd(shooter, hood, feeder, indexer, rangeTable, getDrive()::getPose, 5.0));

    NamedCommands.registerCommand(
        "deployIntake",
        Commands.sequence(
            Commands.runOnce(() -> tilter.setPosition(MechanismTuning.tiltDeploy()), tilter),
            Commands.waitUntil(() -> tilter.atTarget()).withTimeout(0.5),
            Commands.runOnce(() -> tilter.stop(), tilter)));

    NamedCommands.registerCommand(
        "stowIntake",
        Commands.sequence(
            Commands.runOnce(() -> tilter.setPosition(MechanismTuning.tiltStow()), tilter),
            Commands.waitUntil(() -> tilter.atTarget()).withTimeout(0.5)));

    NamedCommands.registerCommand(
        "stopAll",
        Commands.runOnce(
            () -> {
              intake.stop();
              indexer.stop();
              feeder.stop();
              shooter.stop();
              tilter.stop();
            },
            intake,
            indexer,
            feeder,
            shooter,
            tilter));
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

  // ========================================================================
  // Scott's Controller Layout
  // ========================================================================

  /**
   * Driver controller (port 0) — Scott's layout.
   *
   * <pre>
   * Left Stick      → Field-centric translate
   * Right Stick X   → Rotate
   * Left Trigger    → Brake (X-pattern)
   * Right Trigger   → Intake forwards
   * A               → Deploy intake (tilt down)
   * B               → Retract intake (tilt up)
   * Right Bumper    → Intake reverse
   * D-pad Right     → Intake tilt up (manual jog)
   * D-pad Left      → Intake tilt down (manual jog)
   * D-pad Down      → Reset heading
   * Back            → Reset encoders
   * </pre>
   */
  private void configureFuelDriverBindings() {
    // Left Trigger: brake (X-pattern)
    m_controller
        .leftTrigger(DRIVER_TRIGGER_THRESHOLD)
        .whileTrue(Commands.run(() -> getDrive().setBrake(), getDrive()));

    // Right Trigger: intake IN
    m_controller
        .rightTrigger(DRIVER_TRIGGER_THRESHOLD)
        .whileTrue(new VelocityCmd(intake, MechanismTuning::intakeInSpeed));

    // Right Bumper: intake OUT (reverse)
    m_controller
        .rightBumper()
        .whileTrue(new VelocityCmd(intake, () -> -MechanismTuning.intakeOutSpeed()));

    // A: deploy intake (4 sequential tilt jogs down)
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

    // B: retract intake (4 sequential tilt jogs up)
    Command jogIntakeIn =
        Commands.sequence(new RunCommand(() -> tilter.jogUp(), tilter), Commands.waitSeconds(0.2));
    m_controller
        .b()
        .onTrue(
            Commands.sequence(
                jogIntakeIn.asProxy(),
                jogIntakeIn.asProxy(),
                jogIntakeIn.asProxy(),
                jogIntakeIn.asProxy()));

    // D-pad Right: intake tilt up (manual jog)
    m_controller.povRight().whileTrue(new RunCommand(() -> tilter.jogUp(), tilter));

    // D-pad Left: intake tilt down (manual jog)
    m_controller.povLeft().whileTrue(new RunCommand(() -> tilter.jogDown(), tilter));

    // D-pad Down: reset heading
    m_controller.povDown().onTrue(Commands.runOnce(() -> getDrive().resetHeading(), getDrive()));

    // Back: reset encoders
    m_controller.back().onTrue(Commands.runOnce(() -> resetEncoders()));
  }

  /**
   * Operator controller (port 1) — Scott's layout.
   *
   * <pre>
   * Right Trigger   → Feeder forwards
   * Right Bumper    → Indexer forwards
   * Left Trigger    → Auto-aim at HUB (hold) — faces Hub while driver translates
   * A               → Toggle Shooter/Hood Short Range (press start, press stop)
   * B               → Toggle Shooter/Hood Medium Range (press start, press stop)
   * Y               → Toggle Shooter/Hood Long Range (press start, press stop)
   * D-pad Up        → Hood up (manual jog)
   * D-pad Down      → Hood down (manual jog)
   * D-pad Right     → Feeder + Indexer forwards
   * D-pad Left      → Feeder + Indexer reverse
   * Left Bumper     → Drive override (operator sticks control robot)
   * </pre>
   */
  private void configureFuelBindings() {
    CommandXboxController controller = m_operatorController;

    // Right Trigger: feeder forwards
    controller
        .rightTrigger(TRIGGER_THRESHOLD)
        .whileTrue(new VelocityCmd(feeder, () -> -MechanismTuning.feederSpeed()));

    // Right Bumper: indexer forwards
    controller.rightBumper().whileTrue(new VelocityCmd(indexer, MechanismTuning::indexerSpeed));

    // Left Trigger: auto-aim at HUB (driver controls translation, robot faces Hub)
    controller
        .leftTrigger(TRIGGER_THRESHOLD)
        .whileTrue(
            Commands.run(
                () -> {
                  Pose2d pose = getDrive().getPose();
                  Translation2d toHub = HUB_POSITION.minus(pose.getTranslation());
                  Rotation2d angleToHub = toHub.getAngle();

                  // Use driver translation sticks, but auto-rotate to face Hub
                  double xInput = -m_controller.getLeftY();
                  double yInput = -m_controller.getLeftX();
                  getDrive()
                      .driveFieldCentricFacingAngle(
                          xInput * getDrive().getMaxSpeed(),
                          yInput * getDrive().getMaxSpeed(),
                          angleToHub,
                          0.02);
                },
                getDrive()));

    // Shooter range toggles: A/B/Y (press to start, press again to stop & retract hood)
    controller.a().toggleOnTrue(new SetShooterRangeCmd(shooter, hood, ShooterConstants.ShortRange));
    controller.b().toggleOnTrue(new SetShooterRangeCmd(shooter, hood, ShooterConstants.MidRange));
    controller.y().toggleOnTrue(new SetShooterRangeCmd(shooter, hood, ShooterConstants.LongRange));

    // D-pad Up: hood up (manual jog)
    controller.povUp().whileTrue(new RunCommand(() -> hood.jogUp(), hood));

    // D-pad Down: hood down (manual jog)
    controller.povDown().whileTrue(new RunCommand(() -> hood.jogDown(), hood));

    // D-pad Right: feeder + indexer forwards
    controller
        .povRight()
        .whileTrue(
            new VelocityCmd(feeder, () -> -MechanismTuning.feederSpeed())
                .alongWith(new VelocityCmd(indexer, MechanismTuning::indexerSpeed)));

    // D-pad Left: feeder + indexer reverse
    controller
        .povLeft()
        .whileTrue(
            new VelocityCmd(feeder, MechanismTuning::feederSpeed)
                .alongWith(new VelocityCmd(indexer, () -> -MechanismTuning.indexerSpeed())));

    // Left Bumper: operator drive override (operator sticks control robot)
    controller
        .leftBumper()
        .whileTrue(
            Commands.run(
                () -> {
                  double xSpeed = -controller.getLeftY() * getDrive().getMaxSpeed();
                  double ySpeed = -controller.getLeftX() * getDrive().getMaxSpeed();
                  double rot = -controller.getRightX() * 0.75 * 2 * Math.PI;
                  getDrive().drive(xSpeed, ySpeed, rot, true, 0.02);
                },
                getDrive()));
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
