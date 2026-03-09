package frc.robot;

/** Pure math utilities for joystick input processing. Package-private. */
final class InputProcessing {

    private InputProcessing() {}

    /**
     * Applies an expo input curve with sign preservation.
     * Values below the deadband threshold return zero to prevent drift.
     *
     * @param value raw joystick value (-1.0 to 1.0)
     * @param deadband values below this return zero
     * @param expo response curve exponent (1.0 = linear, 2.0 = squared, 3.0 = cubic)
     */
    static double applyInputCurve(double value, double deadband, double expo) {
        double abs = Math.abs(value);
        if (abs <= deadband) {
            return 0.0;
        }
        return Math.copySign(Math.pow(abs, expo), value);
    }

    /**
     * Clamps a 2D stick vector to unit magnitude.
     * If the magnitude exceeds 1.0, the vector is normalized.
     * Otherwise it is returned unchanged.
     *
     * @return a new array {x, y} with magnitude <= 1.0
     */
    static double[] clampStickMagnitude(double x, double y) {
        double mag = Math.hypot(x, y);
        if (mag > 1.0) {
            return new double[] {x / mag, y / mag};
        }
        return new double[] {x, y};
    }
}
