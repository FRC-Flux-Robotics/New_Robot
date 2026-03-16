package frc.robot;

/** Pure math utilities for joystick input processing. */
final class InputProcessing {

  private InputProcessing() {}

  /**
   * Apply expo response curve with deadband.
   *
   * @param value raw input [-1, 1]
   * @param deadband deadband threshold [0, 1)
   * @param expo expo factor [0, 1] where 0=linear, 1=cubic
   * @return processed value [-1, 1] with deadband and expo applied
   */
  static double applyInputCurve(double value, double deadband, double expo) {
    double abs = Math.abs(value);
    if (abs < deadband) {
      return 0.0;
    }
    // Rescale so the range [deadband, 1] maps to [0, 1]
    double rescaled = (abs - deadband) / (1.0 - deadband);
    // Blend between linear and cubic based on expo factor
    double output = (1.0 - expo) * rescaled + expo * rescaled * rescaled * rescaled;
    return Math.copySign(output, value);
  }

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
