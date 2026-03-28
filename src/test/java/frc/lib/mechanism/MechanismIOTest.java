package frc.lib.mechanism;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class MechanismIOTest {

  /** Stub that populates inputs with known values. */
  private static class StubMechanismIO implements MechanismIO {
    double position = 10.0;
    double velocity = 5.0;
    double statorCurrent = 30.0;
    double supplyCurrent = 25.0;
    double voltage = 11.5;
    double temp = 45.0;

    @Override
    public void updateInputs(MechanismIOInputsAutoLogged inputs) {
      inputs.positionRotations = position;
      inputs.velocityRPS = velocity;
      inputs.statorCurrentA = statorCurrent;
      inputs.supplyCurrentA = supplyCurrent;
      inputs.appliedVoltage = voltage;
      inputs.tempCelsius = temp;
      inputs.motorConnected = true;
    }
  }

  @Test
  void stubImplementsUpdateInputs() {
    StubMechanismIO stub = new StubMechanismIO();
    MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();

    stub.updateInputs(inputs);

    assertEquals(10.0, inputs.positionRotations);
    assertEquals(5.0, inputs.velocityRPS);
    assertEquals(30.0, inputs.statorCurrentA);
    assertEquals(25.0, inputs.supplyCurrentA);
    assertEquals(11.5, inputs.appliedVoltage);
    assertEquals(45.0, inputs.tempCelsius);
    assertTrue(inputs.motorConnected);
  }

  @Test
  void defaultInputsAreZero() {
    MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();

    assertEquals(0.0, inputs.positionRotations);
    assertEquals(0.0, inputs.velocityRPS);
    assertEquals(0.0, inputs.statorCurrentA);
    assertEquals(0.0, inputs.supplyCurrentA);
    assertEquals(0.0, inputs.appliedVoltage);
    assertEquals(0.0, inputs.tempCelsius);
    assertFalse(inputs.motorConnected);
  }

  @Test
  void replayIsNoOp() {
    MechanismIO replay = new MechanismIOReplay();
    MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();

    replay.updateInputs(inputs);

    assertEquals(0.0, inputs.positionRotations);
    assertEquals(0.0, inputs.velocityRPS);
    assertEquals(0.0, inputs.statorCurrentA);
    assertEquals(0.0, inputs.supplyCurrentA);
    assertEquals(0.0, inputs.appliedVoltage);
    assertEquals(0.0, inputs.tempCelsius);
    assertFalse(inputs.motorConnected);
  }

  @Test
  void multipleUpdatesOverwriteValues() {
    StubMechanismIO stub = new StubMechanismIO();
    MechanismIOInputsAutoLogged inputs = new MechanismIOInputsAutoLogged();

    stub.updateInputs(inputs);
    assertEquals(10.0, inputs.positionRotations);

    stub.position = 20.0;
    stub.velocity = 8.0;
    stub.updateInputs(inputs);

    assertEquals(20.0, inputs.positionRotations);
    assertEquals(8.0, inputs.velocityRPS);
    // Unchanged fields retain their previous values from the stub
    assertEquals(30.0, inputs.statorCurrentA);
  }
}
