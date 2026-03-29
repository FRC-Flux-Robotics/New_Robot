package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import frc.lib.drivetrain.PIDGains;
import frc.lib.mechanism.ControlMode;
import frc.lib.mechanism.MechanismConfig;
import frc.lib.mechanism.MechanismIO;
import frc.lib.mechanism.MechanismIOInputsAutoLogged;
import frc.lib.mechanism.PositionMechanism;
import frc.lib.mechanism.VelocityMechanism;
import frc.robot.MechanismTuning;
import frc.robot.RangeTable;
import org.junit.jupiter.api.Test;

class MechanismCommandsTest {

  private static final PIDGains TEST_PID = new PIDGains(1.0, 0.0, 0.0, 0.0, 0.12, 0.0);

  /** Stub IO that tracks calls for velocity mechanisms. */
  private static class VelocityStubIO implements MechanismIO {
    double velocity = 0.0;
    String lastCall = "";
    double lastArg = 0.0;

    @Override
    public void updateInputs(MechanismIOInputsAutoLogged inputs) {
      inputs.velocityRPM = velocity;
      inputs.motorConnected = true;
    }

    @Override
    public void setVelocity(double velocityRPS) {
      lastCall = "setVelocity";
      lastArg = velocityRPS;
    }

    @Override
    public void stop() {
      lastCall = "stop";
    }
  }

  /** Stub IO that tracks calls for position mechanisms. */
  private static class PositionStubIO implements MechanismIO {
    double position = 0.0;
    String lastCall = "";
    double lastArg = 0.0;

    @Override
    public void updateInputs(MechanismIOInputsAutoLogged inputs) {
      inputs.positionRotations = position;
      inputs.motorConnected = true;
    }

    @Override
    public void setPosition(double positionRotations) {
      lastCall = "setPosition";
      lastArg = positionRotations;
    }

    @Override
    public void stop() {
      lastCall = "stop";
    }
  }

  private static VelocityMechanism makeVelocityMech(VelocityStubIO io) {
    MechanismConfig config =
        new MechanismConfig.Builder()
            .name("TestVelocity")
            .motorId(1)
            .canBus("")
            .pidGains(TEST_PID)
            .controlMode(ControlMode.VELOCITY)
            .jogStep(5.0)
            .build();
    return new VelocityMechanism(io, config);
  }

  private static PositionMechanism makePositionMech(PositionStubIO io) {
    MechanismConfig config =
        new MechanismConfig.Builder()
            .name("TestPosition")
            .motorId(2)
            .canBus("")
            .pidGains(TEST_PID)
            .controlMode(ControlMode.POSITION)
            .jogStep(0.5)
            .build();
    return new PositionMechanism(io, config);
  }

  // --- VelocityCmd tests ---

  @Test
  void velocityCmd_setsVelocityOnInitAndExecute() {
    VelocityStubIO io = new VelocityStubIO();
    VelocityMechanism mech = makeVelocityMech(io);
    VelocityCmd cmd = new VelocityCmd(mech, () -> 42.0);

    cmd.initialize();
    assertEquals("setVelocity", io.lastCall);
    assertEquals(42.0 / 60.0, io.lastArg, 0.001); // IO receives RPS

    io.lastCall = "";
    cmd.execute();
    assertEquals("setVelocity", io.lastCall);
    assertEquals(42.0 / 60.0, io.lastArg, 0.001); // IO receives RPS
  }

  @Test
  void velocityCmd_stopsOnEnd() {
    VelocityStubIO io = new VelocityStubIO();
    VelocityMechanism mech = makeVelocityMech(io);
    VelocityCmd cmd = new VelocityCmd(mech, () -> 50.0);

    cmd.initialize();
    cmd.end(false);

    assertEquals("stop", io.lastCall);
    assertEquals(0.0, mech.getTargetVelocity());
  }

  @Test
  void velocityCmd_noTimeoutNeverFinishes() {
    VelocityStubIO io = new VelocityStubIO();
    VelocityMechanism mech = makeVelocityMech(io);
    VelocityCmd cmd = new VelocityCmd(mech, () -> 50.0); // no timeout

    cmd.initialize();
    assertFalse(cmd.isFinished());
  }

  // --- ShootCommand tests ---

  @Test
  void shootCommand_startsWhenStopped() {
    VelocityStubIO io = new VelocityStubIO();
    VelocityMechanism mech = makeVelocityMech(io);
    ShootCommand cmd = new ShootCommand(mech, () -> 80.0);

    // Target is 0 (stopped), so initialize should start shooter
    cmd.initialize();

    assertEquals("setVelocity", io.lastCall);
    assertEquals(80.0 / 60.0, io.lastArg, 0.001); // IO receives RPS
    assertTrue(cmd.isFinished());
  }

  @Test
  void shootCommand_stopsWhenRunning() {
    VelocityStubIO io = new VelocityStubIO();
    VelocityMechanism mech = makeVelocityMech(io);

    // Pre-set velocity so target != 0
    mech.setVelocity(80.0);
    assertEquals(80.0, mech.getTargetVelocity());

    ShootCommand cmd = new ShootCommand(mech, () -> 80.0);
    cmd.initialize();

    assertEquals("stop", io.lastCall);
    assertEquals(0.0, mech.getTargetVelocity());
    assertTrue(cmd.isFinished());
  }

  // --- SetShooterRangeCmd tests ---

  @Test
  void setShooterRangeCmd_setsSpeedAndElevation() {
    VelocityStubIO shooterIO = new VelocityStubIO();
    PositionStubIO hoodIO = new PositionStubIO();
    VelocityMechanism shooter = makeVelocityMech(shooterIO);
    PositionMechanism hood = makePositionMech(hoodIO);

    MechanismTuning.init();

    SetShooterRangeCmd cmd = new SetShooterRangeCmd(shooter, hood, 0);
    cmd.initialize();

    // Range index 0 should set speed and elevation from tunable short-range defaults
    double expectedSpeed = MechanismTuning.speedShort();
    double expectedElevation = MechanismTuning.hoodShort();

    assertEquals("setVelocity", shooterIO.lastCall);
    assertEquals(expectedSpeed / 60.0, shooterIO.lastArg, 0.001); // IO receives RPS
    assertEquals("setPosition", hoodIO.lastCall);
    assertEquals(expectedElevation, hoodIO.lastArg, 0.001);
    assertTrue(cmd.isFinished());
  }

  // --- RangeShootCmd tests ---

  @Test
  void rangeShootCmd_stopsFeederAndIndexerOnEnd() {
    VelocityStubIO shooterIO = new VelocityStubIO();
    PositionStubIO hoodIO = new PositionStubIO();
    VelocityStubIO feederIO = new VelocityStubIO();
    VelocityStubIO indexerIO = new VelocityStubIO();

    // Each mechanism needs unique CAN IDs
    VelocityMechanism shooter = makeVelocityMech(shooterIO);
    PositionMechanism hood = makePositionMech(hoodIO);
    VelocityMechanism feeder = makeVelocityMechWithId(feederIO, 3, "Feeder");
    VelocityMechanism indexer = makeVelocityMechWithId(indexerIO, 4, "Indexer");

    RangeTable rangeTable = new RangeTable();
    RangeShootCmd cmd =
        new RangeShootCmd(
            shooter,
            hood,
            feeder,
            indexer,
            rangeTable,
            () -> new edu.wpi.first.math.geometry.Pose2d(),
            5.0);

    // Simulate that feeder/indexer were running
    feeder.setVelocity(40.0);
    indexer.setVelocity(45.0);

    cmd.end(false);

    assertEquals("stop", feederIO.lastCall);
    assertEquals("stop", indexerIO.lastCall);
    assertEquals(0.0, feeder.getTargetVelocity());
    assertEquals(0.0, indexer.getTargetVelocity());
  }

  @Test
  void rangeShootCmd_noTimeoutNeverFinishes() {
    VelocityStubIO shooterIO = new VelocityStubIO();
    PositionStubIO hoodIO = new PositionStubIO();
    VelocityStubIO feederIO = new VelocityStubIO();
    VelocityStubIO indexerIO = new VelocityStubIO();

    VelocityMechanism shooter = makeVelocityMech(shooterIO);
    PositionMechanism hood = makePositionMech(hoodIO);
    VelocityMechanism feeder = makeVelocityMechWithId(feederIO, 3, "Feeder");
    VelocityMechanism indexer = makeVelocityMechWithId(indexerIO, 4, "Indexer");

    RangeShootCmd cmd =
        new RangeShootCmd(
            shooter,
            hood,
            feeder,
            indexer,
            new RangeTable(),
            () -> new edu.wpi.first.math.geometry.Pose2d(),
            0); // no timeout

    cmd.initialize();
    assertFalse(cmd.isFinished());
  }

  private static VelocityMechanism makeVelocityMechWithId(
      VelocityStubIO io, int motorId, String name) {
    MechanismConfig config =
        new MechanismConfig.Builder()
            .name(name)
            .motorId(motorId)
            .canBus("")
            .pidGains(TEST_PID)
            .controlMode(ControlMode.VELOCITY)
            .jogStep(5.0)
            .build();
    return new VelocityMechanism(io, config);
  }
}
