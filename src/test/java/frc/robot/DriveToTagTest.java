package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class DriveToTagTest {

  // --- clampSpeed tests ---

  @Test
  void clampSpeedZerosBelowMin() {
    assertEquals(0.0, DriveToTag.clampSpeed(0.01, 0.05, 0.8));
    assertEquals(0.0, DriveToTag.clampSpeed(-0.01, 0.05, 0.8));
    assertEquals(0.0, DriveToTag.clampSpeed(0.049, 0.05, 0.8));
  }

  @Test
  void clampSpeedPassesValueInRange() {
    assertEquals(0.3, DriveToTag.clampSpeed(0.3, 0.05, 0.8), 1e-9);
    assertEquals(-0.3, DriveToTag.clampSpeed(-0.3, 0.05, 0.8), 1e-9);
    assertEquals(0.05, DriveToTag.clampSpeed(0.05, 0.05, 0.8), 1e-9);
    assertEquals(0.8, DriveToTag.clampSpeed(0.8, 0.05, 0.8), 1e-9);
  }

  @Test
  void clampSpeedCapsAtMax() {
    assertEquals(0.8, DriveToTag.clampSpeed(1.5, 0.05, 0.8), 1e-9);
    assertEquals(-0.8, DriveToTag.clampSpeed(-1.5, 0.05, 0.8), 1e-9);
    assertEquals(0.5, DriveToTag.clampSpeed(0.9, 0.02, 0.5), 1e-9);
  }

  @Test
  void clampSpeedPreservesSign() {
    assertTrue(DriveToTag.clampSpeed(0.1, 0.05, 0.8) > 0);
    assertTrue(DriveToTag.clampSpeed(-0.1, 0.05, 0.8) < 0);
    assertTrue(DriveToTag.clampSpeed(2.0, 0.05, 0.8) > 0);
    assertTrue(DriveToTag.clampSpeed(-2.0, 0.05, 0.8) < 0);
  }

  @Test
  void clampSpeedZeroReturnsZero() {
    assertEquals(0.0, DriveToTag.clampSpeed(0.0, 0.05, 0.8));
  }

  @Test
  void clampSpeedExactlyAtMinIsKept() {
    // Boundary: exactly at min should pass through
    assertEquals(0.05, DriveToTag.clampSpeed(0.05, 0.05, 0.8), 1e-9);
    assertEquals(-0.05, DriveToTag.clampSpeed(-0.05, 0.05, 0.8), 1e-9);
  }

  @Test
  void clampSpeedExactlyAtMaxIsKept() {
    assertEquals(0.8, DriveToTag.clampSpeed(0.8, 0.05, 0.8), 1e-9);
    assertEquals(-0.8, DriveToTag.clampSpeed(-0.8, 0.05, 0.8), 1e-9);
  }
}
