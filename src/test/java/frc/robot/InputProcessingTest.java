package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class InputProcessingTest {

  @Test
  void deadbandZerosSmallInputs() {
    assertEquals(0.0, InputProcessing.applyInputCurve(0.03, 0.05, 0.5));
    assertEquals(0.0, InputProcessing.applyInputCurve(-0.04, 0.05, 0.5));
  }

  @Test
  void deadbandPassesLargeInputs() {
    double result = InputProcessing.applyInputCurve(0.5, 0.05, 0.0);
    assertTrue(result > 0.0, "Should be positive for positive input above deadband");
  }

  @Test
  void fullInputReturnsOne() {
    assertEquals(1.0, InputProcessing.applyInputCurve(1.0, 0.05, 0.0), 1e-9);
    assertEquals(1.0, InputProcessing.applyInputCurve(1.0, 0.05, 1.0), 1e-9);
  }

  @Test
  void negativeInputReturnsNegative() {
    double result = InputProcessing.applyInputCurve(-1.0, 0.05, 0.5);
    assertEquals(-1.0, result, 1e-9);
  }

  @Test
  void linearExpoIsLinear() {
    // expo=0 should give linear response after deadband
    double result = InputProcessing.applyInputCurve(0.525, 0.05, 0.0);
    // (0.525 - 0.05) / (1 - 0.05) = 0.475 / 0.95 = 0.5
    assertEquals(0.5, result, 1e-9);
  }

  @Test
  void cubicExpoIsCubic() {
    // expo=1 should give cubic response
    double result = InputProcessing.applyInputCurve(0.525, 0.05, 1.0);
    // rescaled = 0.5, cubic = 0.5^3 = 0.125
    assertEquals(0.125, result, 1e-9);
  }

  @Test
  void clampStickMagnitudeWithinCircle() {
    double[] result = InputProcessing.clampStickMagnitude(0.3, 0.4);
    assertEquals(0.3, result[0], 1e-9);
    assertEquals(0.4, result[1], 1e-9);
  }

  @Test
  void clampStickMagnitudeNormalizesOverOne() {
    double[] result = InputProcessing.clampStickMagnitude(1.0, 1.0);
    double mag = Math.hypot(result[0], result[1]);
    assertEquals(1.0, mag, 1e-9);
  }

  @Test
  void clampStickMagnitudeZero() {
    double[] result = InputProcessing.clampStickMagnitude(0, 0);
    assertEquals(0.0, result[0]);
    assertEquals(0.0, result[1]);
  }
}
