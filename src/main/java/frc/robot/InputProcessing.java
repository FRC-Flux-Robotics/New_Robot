package frc.robot;

/** Pure math utilities for joystick input processing. */
final class InputProcessing {

  private InputProcessing() {}

  /**
   * Clamp 2D stick magnitude to unit circle. If the magnitude exceeds 1.0,
   * normalize to unit length. Returns {x, y}.
   */
  static double[] clampStickMagnitude(double x, double y) {
    double mag = Math.hypot(x, y);
    if (mag > 1.0) {
      return new double[] {x / mag, y / mag};
    }
    return new double[] {x, y};
  }
}
