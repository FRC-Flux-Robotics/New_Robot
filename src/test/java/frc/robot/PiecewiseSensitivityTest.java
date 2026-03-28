package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PiecewiseSensitivityTest {

  private PiecewiseSensitivity sensitivity;

  // Standard test configuration:
  // xStart=0.1, xMiddle=0.5, yStart=0.1, yMiddle=0.5, yMax=1.0
  @BeforeEach
  void setUp() {
    sensitivity = new PiecewiseSensitivity(0.1, 0.5, 0.1, 0.5, 1.0);
  }

  @Test
  void transfer_belowStart_returnsZero() {
    assertEquals(0.0, sensitivity.transfer(0.05), 1e-9);
    assertEquals(0.0, sensitivity.transfer(0.0), 1e-9);
    assertEquals(0.0, sensitivity.transfer(0.09), 1e-9);
  }

  @Test
  void transfer_atStart_returnsStartY() {
    assertEquals(0.1, sensitivity.transfer(0.1), 1e-9);
  }

  @Test
  void transfer_atMiddle_returnsMiddleY() {
    assertEquals(0.5, sensitivity.transfer(0.5), 1e-9);
  }

  @Test
  void transfer_atOne_returnsMax() {
    assertEquals(1.0, sensitivity.transfer(1.0), 1e-9);
  }

  @Test
  void transfer_aboveOne_returnsMax() {
    assertEquals(1.0, sensitivity.transfer(1.5), 1e-9);
    assertEquals(1.0, sensitivity.transfer(2.0), 1e-9);
  }

  @Test
  void transfer_negativeInput_returnsNegativeOutput() {
    double positiveResult = sensitivity.transfer(0.3);
    double negativeResult = sensitivity.transfer(-0.3);
    assertEquals(-positiveResult, negativeResult, 1e-9);
  }

  @Test
  void transfer_negativeBelowStart_returnsZero() {
    assertEquals(0.0, sensitivity.transfer(-0.05), 1e-9);
  }

  @Test
  void transfer_inFirstSegment_interpolatesCorrectly() {
    // At x=0.3 (midpoint between 0.1 and 0.5), should be at y=0.3 (midpoint between 0.1 and 0.5)
    assertEquals(0.3, sensitivity.transfer(0.3), 1e-9);
  }

  @Test
  void transfer_inSecondSegment_interpolatesCorrectly() {
    // At x=0.75 (midpoint between 0.5 and 1.0), should be at y=0.75 (midpoint between 0.5 and 1.0)
    assertEquals(0.75, sensitivity.transfer(0.75), 1e-9);
  }

  @Test
  void set_updatesAllParameters() {
    sensitivity.set(0.2, 0.6, 0.2, 0.4, 0.8);
    // Below new threshold
    assertEquals(0.0, sensitivity.transfer(0.1), 1e-9);
    // At new max
    assertEquals(0.8, sensitivity.transfer(1.0), 1e-9);
  }

  @Test
  void transfer_withZeroDeadzone_noDeadband() {
    PiecewiseSensitivity noDeadzone = new PiecewiseSensitivity(0.0, 0.5, 0.0, 0.5, 1.0);
    assertEquals(0.01, noDeadzone.transfer(0.01), 1e-9);
    assertEquals(0.0, noDeadzone.transfer(0.0), 1e-9);
  }

  @Test
  void transfer_withFullMiddle_onlyFirstSegment() {
    PiecewiseSensitivity firstSegmentOnly = new PiecewiseSensitivity(0.1, 1.0, 0.1, 1.0, 1.0);
    double midpoint = 0.55;
    double expected = 0.1 + (1.0 - 0.1) / (1.0 - 0.1) * (midpoint - 0.1);
    assertEquals(expected, firstSegmentOnly.transfer(midpoint), 1e-9);
  }

  @Test
  void transfer_largeNegativeInput_clampsCorrectly() {
    assertEquals(-1.0, sensitivity.transfer(-1.5), 1e-9);
    assertEquals(-1.0, sensitivity.transfer(-2.0), 1e-9);
    assertEquals(-1.0, sensitivity.transfer(-10.0), 1e-9);
  }

  @Test
  void transfer_exactlyAtBoundaries_handlesCorrectly() {
    assertEquals(0.1, sensitivity.transfer(0.1), 1e-9);
    assertEquals(-0.1, sensitivity.transfer(-0.1), 1e-9);
    assertEquals(0.5, sensitivity.transfer(0.5), 1e-9);
    assertEquals(-0.5, sensitivity.transfer(-0.5), 1e-9);
  }
}
