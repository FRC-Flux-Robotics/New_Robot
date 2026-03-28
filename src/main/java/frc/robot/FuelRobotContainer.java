package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.drivetrain.DriveInterface;
import frc.lib.vision.VisionIO;
import frc.robot.FuelConstants.IndexerConstants;
import frc.robot.FuelConstants.IntakeConstants;
import frc.robot.FuelConstants.ShooterConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.PositionMech;
import frc.robot.subsystems.VelocityMech;
import frc.robot.subsystems.VelocityMech2;

/**
 * FUEL robot container — adds mechanism subsystems and operator controller bindings
 * on top of the base RobotContainer drivetrain setup.
 */
public class FuelRobotContainer extends RobotContainer {
  private static final double TRIGGER_THRESHOLD = 0.5;
  private static final double DRIVER_TRIGGER_THRESHOLD = 0.05;

  private final CANBus canBus = new CANBus(FuelConstants.MECHANISM_CAN_BUS);

  private final VelocityMech intake;
  private final PositionMech tilter;
  private final VelocityMech indexer;
  private final VelocityMech feeder;
  private final VelocityMech2 shooter;
  private final PositionMech hood;
  private final RangeTable rangeTable;

  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  public FuelRobotContainer(DriveInterface drive, VisionIO[] visionIOs) {
    super(drive, visionIOs);

    intake = new VelocityMech(canBus, "Intake", IntakeConstants.MotorId);
    tilter = new PositionMech(canBus, "Tilter", IntakeConstants.TiltMotorId);
    indexer = new VelocityMech(canBus, "Indexer", IndexerConstants.IndexerId);
    feeder = new VelocityMech(canBus, "Feeder", IndexerConstants.FeederId);
    shooter = new VelocityMech2(canBus, "Shooter", ShooterConstants.RightMotorId, ShooterConstants.LeftMotorId);
    hood = new PositionMech(canBus, "Hood", ShooterConstants.HoodMotorId);

    rangeTable = new RangeTable();

    configureFuelDriverBindings();
    configureFuelBindings();
    storeParameters();
  }

  // --- Override base class driver bindings (no-ops, replaced by configureFuelDriverBindings) ---
  @Override protected void configureDriverYButton() {}
  @Override protected void configureDriverBumpers() {}
  @Override protected void configureDriverStartButton() {}
  @Override protected void configureDriverDPad() {}

  /** Driver controller bindings matching legacy FuelRobotContainer (two-controller mode). */
  private void configureFuelDriverBindings() {
    // Y: reset heading
    m_controller.y().onTrue(
        Commands.runOnce(() -> getDrive().resetHeading(), getDrive()));

    // Right Trigger: intake IN
    m_controller.rightTrigger(DRIVER_TRIGGER_THRESHOLD).whileTrue(
        new VelocityCmd(intake, () -> IntakeConstants.InSpeed, FuelConstants.Forward));

    // Right Bumper: intake OUT
    m_controller.rightBumper().whileTrue(
        new VelocityCmd(intake, () -> IntakeConstants.OutSpeed, FuelConstants.Backward));

    // A: deploy intake (4 sequential tilt jogs)
    Command jogIntakeOut = Commands.sequence(
        new RunCommand(() -> tilter.jogDown(IntakeConstants.TiltStep), tilter),
        Commands.waitSeconds(0.2));
    m_controller.a().onTrue(Commands.sequence(
        jogIntakeOut.asProxy(),
        jogIntakeOut.asProxy(),
        jogIntakeOut.asProxy(),
        jogIntakeOut.asProxy()));

    // POV Left/Right: manual intake tilt
    m_controller.povLeft().whileTrue(
        new RunCommand(() -> tilter.jogDown(IntakeConstants.TiltStep), tilter));
    m_controller.povRight().whileTrue(
        new RunCommand(() -> tilter.jogUp(IntakeConstants.TiltStep), tilter));

    // Left Bumper: seed field-centric heading
    m_controller.leftBumper().whileTrue(
        Commands.runOnce(() -> getDrive().resetHeading(), getDrive()));

    // Back: reset encoders
    m_controller.back().onTrue(
        Commands.runOnce(() -> resetEncoders()));
  }

  private void configureFuelBindings() {
    CommandXboxController controller = m_operatorController;

    // Intake control (on driver controller)
    // Right Trigger - Run intake roller IN
    // Right Bumper - Run intake roller OUT
    // (Note: these override the sysId controller on port 1 — FUEL uses port 1 for operator)

    // Operator controller bindings (two-controller mode from legacy):

    // Feeder: Right Trigger
    controller.rightTrigger(TRIGGER_THRESHOLD).whileTrue(
        new VelocityCmd(feeder, () -> IndexerConstants.FeederSpeed, FuelConstants.Backward));

    // Indexer: Right Bumper
    controller.rightBumper().whileTrue(
        new VelocityCmd(indexer, () -> IndexerConstants.Speed, FuelConstants.Forward));

    // Range shoot: Left Bumper
    controller.leftBumper().whileTrue(
        new RangeShootCmd(shooter, hood, feeder, indexer, rangeTable, getDrive()::getPose));

    // Shooter range presets: A/B/Y
    controller.a().onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.ShortRange));
    controller.b().onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.MidRange));
    controller.y().onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.LongRange));

    // Shooter toggle: Start
    controller.start().toggleOnTrue(
        new ShootCommand(shooter, () -> ShooterConstants.Speed, FuelConstants.Forward));

    // Shooter speed adjust: D-pad left/right + left bumper
    controller.povRight().and(controller.leftBumper()).whileTrue(
        Commands.runOnce(() -> shooter.speedUp(ShooterConstants.SpeedStep), shooter));
    controller.povLeft().and(controller.leftBumper()).whileTrue(
        Commands.runOnce(() -> shooter.speedDown(ShooterConstants.SpeedStep), shooter));

    // Hood jog: D-pad up/down
    controller.povUp().whileTrue(new RunCommand(() -> hood.jogUp(ShooterConstants.HoodStep), hood));
    controller.povDown().whileTrue(new RunCommand(() -> hood.jogDown(ShooterConstants.HoodStep), hood));
  }

  /** Publish all mechanism PID params to SmartDashboard. */
  public void storeParameters() {
    intake.putParams();
    tilter.putParams();
    indexer.putParams();
    feeder.putParams();
    shooter.putParams();
    hood.putParams();
  }

  /** Fetch all mechanism PID params from SmartDashboard. */
  public void fetchParameters() {
    intake.getParams();
    tilter.getParams();
    indexer.getParams();
    feeder.getParams();
    shooter.getParams();
    hood.getParams();
  }

  public void resetEncoders() {
    hood.resetEncoders();
    tilter.resetEncoders();
  }
}
