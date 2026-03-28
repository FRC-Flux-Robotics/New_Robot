package frc.lib.mechanism;

import static org.junit.jupiter.api.Assertions.*;

import frc.lib.drivetrain.PIDGains;
import org.junit.jupiter.api.Test;

class MechanismConfigTest {

  private static final PIDGains TEST_PID = new PIDGains(0.5, 0.0, 0.0, 0.01, 0.12, 0.0);

  /** Returns a builder with all required fields set. */
  private static MechanismConfig.Builder validBuilder() {
    return new MechanismConfig.Builder()
        .name("Intake")
        .motorId(1)
        .canBus("")
        .pidGains(TEST_PID)
        .controlMode(ControlMode.VELOCITY);
  }

  @Test
  void buildsWithRequiredFieldsOnly() {
    MechanismConfig config = validBuilder().build();
    assertEquals("Intake", config.name);
    assertEquals(1, config.motorId);
    assertEquals("", config.canBus);
    assertEquals(TEST_PID, config.pidGains);
    assertEquals(ControlMode.VELOCITY, config.controlMode);
  }

  @Test
  void buildsWithAllFields() {
    PIDGains pid = new PIDGains(2.4, 0.0, 0.1);
    MechanismConfig config =
        new MechanismConfig.Builder()
            .name("Shooter")
            .motorId(10)
            .canBus("Drivetrain")
            .pidGains(pid)
            .controlMode(ControlMode.VELOCITY)
            .secondMotorId(11)
            .counterRotating(true)
            .statorCurrentLimit(60)
            .supplyCurrentLimit(40)
            .peakVoltage(11.0)
            .gearRatio(2.5)
            .inverted(true)
            .neutralModeBrake(false)
            .jogStep(0.5)
            .softLimitForward(20.0)
            .softLimitReverse(-5.0)
            .motionMagicCruiseVelocity(80.0)
            .motionMagicAcceleration(160.0)
            .build();

    assertEquals("Shooter", config.name);
    assertEquals(10, config.motorId);
    assertEquals("Drivetrain", config.canBus);
    assertEquals(pid, config.pidGains);
    assertEquals(ControlMode.VELOCITY, config.controlMode);
    assertEquals(11, config.secondMotorId);
    assertTrue(config.counterRotating);
    assertEquals(60, config.statorCurrentLimit);
    assertEquals(40, config.supplyCurrentLimit);
    assertEquals(11.0, config.peakVoltage);
    assertEquals(2.5, config.gearRatio);
    assertTrue(config.inverted);
    assertFalse(config.neutralModeBrake);
    assertEquals(0.5, config.jogStep);
    assertEquals(20.0, config.softLimitForward);
    assertEquals(-5.0, config.softLimitReverse);
    assertEquals(80.0, config.motionMagicCruiseVelocity);
    assertEquals(160.0, config.motionMagicAcceleration);
  }

  @Test
  void defaultValues() {
    MechanismConfig config = validBuilder().build();
    assertEquals(-1, config.secondMotorId);
    assertFalse(config.counterRotating);
    assertEquals(0, config.statorCurrentLimit);
    assertEquals(0, config.supplyCurrentLimit);
    assertEquals(12.0, config.peakVoltage);
    assertEquals(1.0, config.gearRatio);
    assertFalse(config.inverted);
    assertTrue(config.neutralModeBrake);
    assertEquals(0, config.jogStep);
    assertEquals(Double.MAX_VALUE, config.softLimitForward);
    assertEquals(-Double.MAX_VALUE, config.softLimitReverse);
    assertEquals(0, config.motionMagicCruiseVelocity);
    assertEquals(0, config.motionMagicAcceleration);
  }

  @Test
  void requiresName() {
    assertThrows(IllegalStateException.class, () -> validBuilder().name(null).build());
    assertThrows(IllegalStateException.class, () -> validBuilder().name("").build());
  }

  @Test
  void requiresMotorId() {
    // Default motorId is -1, which should fail validation
    assertThrows(
        IllegalStateException.class,
        () ->
            new MechanismConfig.Builder()
                .name("Test")
                .canBus("")
                .pidGains(TEST_PID)
                .controlMode(ControlMode.VELOCITY)
                .build());
  }

  @Test
  void requiresCanBus() {
    assertThrows(IllegalStateException.class, () -> validBuilder().canBus(null).build());
  }

  @Test
  void requiresPidGains() {
    assertThrows(IllegalStateException.class, () -> validBuilder().pidGains(null).build());
  }

  @Test
  void requiresControlMode() {
    assertThrows(IllegalStateException.class, () -> validBuilder().controlMode(null).build());
  }

  @Test
  void rejectsInvalidMotorId() {
    assertThrows(IllegalStateException.class, () -> validBuilder().motorId(-1).build());
    assertThrows(IllegalStateException.class, () -> validBuilder().motorId(63).build());
  }

  @Test
  void rejectsInvalidSecondMotorId() {
    // secondMotorId > 62
    assertThrows(IllegalStateException.class, () -> validBuilder().secondMotorId(63).build());
    // secondMotorId == motorId
    assertThrows(IllegalStateException.class, () -> validBuilder().secondMotorId(1).build());
  }

  @Test
  void rejectsNegativeGearRatio() {
    assertThrows(IllegalStateException.class, () -> validBuilder().gearRatio(0).build());
    assertThrows(IllegalStateException.class, () -> validBuilder().gearRatio(-1.0).build());
  }

  @Test
  void rejectsInvalidSoftLimits() {
    // forward == reverse
    assertThrows(
        IllegalStateException.class,
        () -> validBuilder().softLimitForward(10.0).softLimitReverse(10.0).build());
    // forward < reverse
    assertThrows(
        IllegalStateException.class,
        () -> validBuilder().softLimitForward(5.0).softLimitReverse(10.0).build());
  }

  @Test
  void isDualMotor() {
    assertFalse(validBuilder().build().isDualMotor());
    assertTrue(validBuilder().secondMotorId(2).build().isDualMotor());
  }

  @Test
  void hasMotionMagic() {
    assertFalse(validBuilder().build().hasMotionMagic());
    assertTrue(validBuilder().motionMagicCruiseVelocity(80.0).build().hasMotionMagic());
  }

  @Test
  void hasSoftLimits() {
    assertFalse(validBuilder().build().hasSoftLimits());
    assertTrue(validBuilder().softLimitForward(20.0).build().hasSoftLimits());
    assertTrue(validBuilder().softLimitReverse(-5.0).build().hasSoftLimits());
  }

  @Test
  void hasCurrentLimits() {
    MechanismConfig config = validBuilder().build();
    assertFalse(config.hasStatorCurrentLimit());
    assertFalse(config.hasSupplyCurrentLimit());

    MechanismConfig withLimits =
        validBuilder().statorCurrentLimit(60).supplyCurrentLimit(40).build();
    assertTrue(withLimits.hasStatorCurrentLimit());
    assertTrue(withLimits.hasSupplyCurrentLimit());
  }
}
