package frc.lib.mechanism;

import static org.junit.jupiter.api.Assertions.*;

import frc.lib.drivetrain.PIDGains;
import org.junit.jupiter.api.Test;

class PositionMechanismTest {

  private static final PIDGains TEST_PID = new PIDGains(1.0, 0.0, 0.0, 0.0, 0.12, 0.0);
  private static final double JOG_STEP = 0.5;

  /** Stub IO that tracks method calls and exposes settable position. */
  private static class StubIO implements MechanismIO {
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
    public void setMotionMagicPosition(double positionRotations) {
      lastCall = "setMotionMagicPosition";
      lastArg = positionRotations;
    }

    @Override
    public void stop() {
      lastCall = "stop";
    }

    @Override
    public void resetPosition() {
      lastCall = "resetPosition";
    }
  }

  private static MechanismConfig.Builder validPositionBuilder() {
    return new MechanismConfig.Builder()
        .name("TestPosition")
        .motorId(1)
        .canBus("")
        .pidGains(TEST_PID)
        .controlMode(ControlMode.POSITION)
        .jogStep(JOG_STEP);
  }

  @Test
  void rejectsNonPositionControlMode() {
    StubIO io = new StubIO();
    MechanismConfig config =
        new MechanismConfig.Builder()
            .name("Bad")
            .motorId(1)
            .canBus("")
            .pidGains(TEST_PID)
            .controlMode(ControlMode.VELOCITY)
            .build();

    assertThrows(IllegalArgumentException.class, () -> new PositionMechanism(io, config));
  }

  @Test
  void setPositionUpdatesTarget() {
    StubIO io = new StubIO();
    PositionMechanism mech = new PositionMechanism(io, validPositionBuilder().build());

    mech.setPosition(5.0);

    assertEquals(5.0, mech.getTargetPosition());
    assertEquals("setPosition", io.lastCall);
    assertEquals(5.0, io.lastArg);
  }

  @Test
  void setPositionUsesMotionMagicWhenConfigured() {
    StubIO io = new StubIO();
    MechanismConfig config =
        validPositionBuilder()
            .motionMagicCruiseVelocity(10.0)
            .motionMagicAcceleration(20.0)
            .build();
    PositionMechanism mech = new PositionMechanism(io, config);

    mech.setPosition(3.0);

    assertEquals("setMotionMagicPosition", io.lastCall);
    assertEquals(3.0, io.lastArg);
  }

  @Test
  void atTargetWhenWithinTolerance() {
    StubIO io = new StubIO();
    PositionMechanism mech = new PositionMechanism(io, validPositionBuilder().build());

    // Set target to 5.0, position exactly at target
    mech.setPosition(5.0);
    io.position = 5.0;
    mech.periodic();

    assertTrue(mech.atTarget());

    // Position within jogStep tolerance
    io.position = 5.0 + JOG_STEP;
    mech.periodic();

    assertTrue(mech.atTarget());
  }

  @Test
  void notAtTargetWhenOutsideTolerance() {
    StubIO io = new StubIO();
    PositionMechanism mech = new PositionMechanism(io, validPositionBuilder().build());

    mech.setPosition(5.0);
    io.position = 5.0 + JOG_STEP + 0.1;
    mech.periodic();

    assertFalse(mech.atTarget());
  }

  @Test
  void jogUpIncrementsPosition() {
    StubIO io = new StubIO();
    PositionMechanism mech = new PositionMechanism(io, validPositionBuilder().build());

    io.position = 2.0;
    mech.periodic();
    mech.jogUp();

    assertEquals(2.0 + JOG_STEP, mech.getTargetPosition());
  }

  @Test
  void jogDownDecrementsPosition() {
    StubIO io = new StubIO();
    PositionMechanism mech = new PositionMechanism(io, validPositionBuilder().build());

    io.position = 2.0;
    mech.periodic();
    mech.jogDown();

    assertEquals(2.0 - JOG_STEP, mech.getTargetPosition());
  }

  @Test
  void stopCallsIoStop() {
    StubIO io = new StubIO();
    PositionMechanism mech = new PositionMechanism(io, validPositionBuilder().build());

    mech.stop();

    assertEquals("stop", io.lastCall);
  }

  @Test
  void defaultToleranceWhenNoJogStep() {
    StubIO io = new StubIO();
    MechanismConfig config = validPositionBuilder().jogStep(0.0).build();
    PositionMechanism mech = new PositionMechanism(io, config);

    // Target is 0.0 (default), position at 1.0 should be at tolerance boundary
    io.position = 1.0;
    mech.periodic();

    assertTrue(mech.atTarget());

    // Just outside default tolerance of 1.0
    io.position = 1.1;
    mech.periodic();

    assertFalse(mech.atTarget());
  }
}
