package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class DiagnosticsConfigTest {

  @Test
  void defaultsMatchOriginalConstants() {
    DiagnosticsConfig config = DiagnosticsConfig.defaults();
    assertEquals(80.0, config.motorTempWarnC);
    assertEquals(100.0, config.motorTempErrorC);
    assertEquals(0.9, config.currentWarnFraction);
    assertEquals(30.0, config.alignmentErrorWarnDeg);
    assertEquals(50.0, config.odometryMinHz);
    assertEquals(2.0, config.diagnosticCooldownSec);
    assertEquals(10.5, config.brownoutStartV);
    assertEquals(7.0, config.brownoutMinV);
    assertEquals(0.25, config.brownoutMinScale);
  }

  @Test
  void builderOverridesDefaults() {
    DiagnosticsConfig config =
        DiagnosticsConfig.builder()
            .motorTempWarn(70.0)
            .motorTempError(90.0)
            .currentWarnFraction(0.8)
            .alignmentErrorWarn(20.0)
            .odometryMinHz(100.0)
            .diagnosticCooldown(5.0)
            .brownout(11.0, 8.0, 0.3)
            .build();
    assertEquals(70.0, config.motorTempWarnC);
    assertEquals(90.0, config.motorTempErrorC);
    assertEquals(0.8, config.currentWarnFraction);
    assertEquals(20.0, config.alignmentErrorWarnDeg);
    assertEquals(100.0, config.odometryMinHz);
    assertEquals(5.0, config.diagnosticCooldownSec);
    assertEquals(11.0, config.brownoutStartV);
    assertEquals(8.0, config.brownoutMinV);
    assertEquals(0.3, config.brownoutMinScale);
  }

  @Test
  void rejectsNonPositiveMotorTempWarn() {
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().motorTempWarn(0));
  }

  @Test
  void rejectsNonPositiveMotorTempError() {
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().motorTempError(0));
  }

  @Test
  void rejectsInvalidCurrentWarnFraction() {
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().currentWarnFraction(0));
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().currentWarnFraction(1.1));
  }

  @Test
  void acceptsCurrentWarnFractionAtOne() {
    assertDoesNotThrow(() -> DiagnosticsConfig.builder().currentWarnFraction(1.0));
  }

  @Test
  void rejectsNonPositiveAlignmentWarn() {
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().alignmentErrorWarn(0));
  }

  @Test
  void rejectsNonPositiveOdometryMinHz() {
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().odometryMinHz(0));
  }

  @Test
  void rejectsNonPositiveDiagnosticCooldown() {
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().diagnosticCooldown(0));
  }

  @Test
  void rejectsBrownoutMinVGreaterThanStartV() {
    assertThrows(
        IllegalArgumentException.class,
        () -> DiagnosticsConfig.builder().brownout(10.0, 10.0, 0.25));
    assertThrows(
        IllegalArgumentException.class,
        () -> DiagnosticsConfig.builder().brownout(10.0, 11.0, 0.25));
  }

  @Test
  void rejectsBrownoutMinScaleOutOfRange() {
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().brownout(10.5, 7.0, 0));
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().brownout(10.5, 7.0, 1.0));
  }

  @Test
  void rejectsNonPositiveBrownoutVoltages() {
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().brownout(0, 7.0, 0.25));
    assertThrows(
        IllegalArgumentException.class, () -> DiagnosticsConfig.builder().brownout(10.5, 0, 0.25));
  }
}
