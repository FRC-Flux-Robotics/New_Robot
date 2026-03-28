package frc.lib.mechanism;

import static org.junit.jupiter.api.Assertions.*;

import frc.lib.drivetrain.PIDGains;
import org.junit.jupiter.api.Test;

class VelocityMechanismTest {

  private static final PIDGains TEST_PID = new PIDGains(1.0, 0.0, 0.0, 0.0, 0.12, 0.0);
  private static final double JOG_STEP = 10.0;

  /** Stub IO that tracks method calls and exposes settable velocity. */
  private static class StubIO implements MechanismIO {
    double velocity = 0.0;
    String lastCall = "";
    double lastArg = 0.0;

    @Override
    public void updateInputs(MechanismIOInputsAutoLogged inputs) {
      inputs.velocityRPS = velocity;
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

  private static MechanismConfig.Builder validVelocityBuilder() {
    return new MechanismConfig.Builder()
        .name("TestVelocity")
        .motorId(1)
        .canBus("")
        .pidGains(TEST_PID)
        .controlMode(ControlMode.VELOCITY)
        .jogStep(JOG_STEP);
  }

  @Test
  void rejectsNonVelocityControlMode() {
    StubIO io = new StubIO();
    MechanismConfig config =
        new MechanismConfig.Builder()
            .name("Bad")
            .motorId(1)
            .canBus("")
            .pidGains(TEST_PID)
            .controlMode(ControlMode.POSITION)
            .build();

    assertThrows(IllegalArgumentException.class, () -> new VelocityMechanism(io, config));
  }

  @Test
  void setVelocityUpdatesTarget() {
    StubIO io = new StubIO();
    VelocityMechanism mech = new VelocityMechanism(io, validVelocityBuilder().build());

    mech.setVelocity(50.0);

    assertEquals(50.0, mech.getTargetVelocity());
    assertEquals("setVelocity", io.lastCall);
    assertEquals(50.0, io.lastArg);
  }

  @Test
  void atTargetWhenWithinTolerance() {
    StubIO io = new StubIO();
    VelocityMechanism mech = new VelocityMechanism(io, validVelocityBuilder().build());

    mech.setVelocity(50.0);
    io.velocity = 50.0;
    mech.periodic();

    assertTrue(mech.atTarget());

    // At tolerance boundary
    io.velocity = 50.0 + JOG_STEP;
    mech.periodic();

    assertTrue(mech.atTarget());
  }

  @Test
  void notAtTargetWhenOutsideTolerance() {
    StubIO io = new StubIO();
    VelocityMechanism mech = new VelocityMechanism(io, validVelocityBuilder().build());

    mech.setVelocity(50.0);
    io.velocity = 50.0 + JOG_STEP + 0.1;
    mech.periodic();

    assertFalse(mech.atTarget());
  }

  @Test
  void stopCallsIoStopAndResetsTarget() {
    StubIO io = new StubIO();
    VelocityMechanism mech = new VelocityMechanism(io, validVelocityBuilder().build());

    mech.setVelocity(50.0);
    mech.stop();

    assertEquals("stop", io.lastCall);
    assertEquals(0.0, mech.getTargetVelocity());
  }

  @Test
  void defaultToleranceWhenNoJogStep() {
    StubIO io = new StubIO();
    MechanismConfig config = validVelocityBuilder().jogStep(0.0).build();
    VelocityMechanism mech = new VelocityMechanism(io, config);

    // Target is 0.0 (default), velocity at 5.0 should be at default tolerance boundary
    io.velocity = 5.0;
    mech.periodic();

    assertTrue(mech.atTarget());

    // Just outside default tolerance of 5.0
    io.velocity = 5.1;
    mech.periodic();

    assertFalse(mech.atTarget());
  }
}
