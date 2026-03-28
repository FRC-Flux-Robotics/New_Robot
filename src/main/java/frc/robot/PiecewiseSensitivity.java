package frc.robot;

/**
 * Piecewise linear sensitivity curve for joystick input processing.
 *
 * <p>Applies a two-segment linear transfer function to joystick inputs, providing:
 *
 * <ul>
 *   <li>A deadzone where small inputs below {@code xStart} return zero
 *   <li>A first linear segment from ({@code xStart}, {@code yStart}) to ({@code xMiddle}, {@code
 *       yMiddle})
 *   <li>A second linear segment from ({@code xMiddle}, {@code yMiddle}) to (1.0, {@code yMax})
 * </ul>
 *
 * <p>This allows fine-grained control at low speeds while still reaching maximum output. Negative
 * inputs produce negative outputs with the same magnitude transformation.
 */
public class PiecewiseSensitivity {
  private double xStart;
  private double xMiddle;
  private double yStart;
  private double yMiddle;
  private double yMax;
  private double slope1;
  private double slope2;

  /**
   * Creates a new piecewise sensitivity curve with the specified parameters.
   *
   * @param xStart input threshold below which output is zero (deadzone), typically 0.02-0.1
   * @param xMiddle input value where the curve transitions between segments, typically 0.5-0.7
   * @param yStart output value at {@code xStart}, typically small (e.g., 0.1)
   * @param yMiddle output value at {@code xMiddle}, controls sensitivity in first segment
   * @param yMax maximum output value when input reaches 1.0
   */
  public PiecewiseSensitivity(
      double xStart, double xMiddle, double yStart, double yMiddle, double yMax) {
    set(xStart, xMiddle, yStart, yMiddle, yMax);
  }

  /**
   * Updates the sensitivity curve parameters and recalculates internal slopes.
   *
   * <p>Can be used to dynamically adjust sensitivity during runtime, for example via SmartDashboard
   * tuning.
   *
   * @param xStart input threshold below which output is zero (deadzone)
   * @param xMiddle input value where the curve transitions between segments
   * @param yStart output value at {@code xStart}
   * @param yMiddle output value at {@code xMiddle}
   * @param yMax maximum output value when input reaches 1.0
   */
  public void set(double xStart, double xMiddle, double yStart, double yMiddle, double yMax) {
    this.xStart = xStart;
    this.xMiddle = xMiddle;
    this.yStart = yStart;
    this.yMiddle = yMiddle;
    this.yMax = yMax;
    slope1 = xStart < xMiddle ? (yMiddle - yStart) / (xMiddle - xStart) : 0.0;
    slope2 = xMiddle < 1.0 ? (yMax - yMiddle) / (1.0 - xMiddle) : 0.0;
  }

  /**
   * Applies the sensitivity curve to transform a joystick input value.
   *
   * <p>The sign of the input is preserved in the output.
   *
   * @param x the input value, typically from a joystick axis in range [-1.0, 1.0]
   * @return the transformed output value, with sign preserved
   */
  public double transfer(double x) {
    double xabs = Math.abs(x);
    if (xabs < xStart) {
      return 0.0;
    } else if (xabs < xMiddle) {
      xabs = yStart + slope1 * (xabs - xStart);
    } else if (xabs < 1.0) {
      xabs = yMiddle + slope2 * (xabs - xMiddle);
    } else {
      xabs = yMax;
    }
    return x >= 0.0 ? xabs : -xabs;
  }
}
