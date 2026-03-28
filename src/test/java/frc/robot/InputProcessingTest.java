package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class InputProcessingTest {

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
