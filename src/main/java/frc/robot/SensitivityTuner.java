package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages sensitivity curve tuning via SmartDashboard.
 *
 * <p>Parameters are published to SmartDashboard on construction and read
 * back on each transfer() call, allowing real-time adjustment during testing.
 */
public class SensitivityTuner {
    private double xStart;
    private double xMiddle;
    private double yStart;
    private double yMiddle;
    private double yMax;

    private final PiecewiseSensitivity sensitivity;
    private final String prefix;

    /**
     * Creates a new SensitivityTuner with the specified parameters.
     *
     * @param prefix SmartDashboard key prefix (e.g., "Drive_" for translation, "Rot_" for rotation)
     * @param xStart input threshold below which output is zero (deadzone)
     * @param xMiddle input value where the curve transitions between segments
     * @param yStart output value at xStart
     * @param yMiddle output value at xMiddle
     * @param yMax maximum output value when input reaches 1.0
     */
    public SensitivityTuner(String prefix, double xStart, double xMiddle, double yStart, double yMiddle, double yMax) {
        this.prefix = prefix;
        this.xStart = xStart;
        this.xMiddle = xMiddle;
        this.yStart = yStart;
        this.yMiddle = yMiddle;
        this.yMax = yMax;
        this.sensitivity = new PiecewiseSensitivity(xStart, xMiddle, yStart, yMiddle, yMax);

        SmartDashboard.putNumber(prefix + "Start_X", xStart);
        SmartDashboard.putNumber(prefix + "Middle_X", xMiddle);
        SmartDashboard.putNumber(prefix + "Start_Y", yStart);
        SmartDashboard.putNumber(prefix + "Middle_Y", yMiddle);
        SmartDashboard.putNumber(prefix + "Max_Y", yMax);
    }

    /**
     * Updates parameters from SmartDashboard and applies the sensitivity curve.
     *
     * @param input the input value, typically from a joystick axis in range [-1.0, 1.0]
     * @return the transformed output value
     */
    public double transfer(double input) {
        updateFromDashboard();
        return sensitivity.transfer(input);
    }

    private void updateFromDashboard() {
        double newXStart = SmartDashboard.getNumber(prefix + "Start_X", xStart);
        double newXMiddle = SmartDashboard.getNumber(prefix + "Middle_X", xMiddle);
        double newYStart = SmartDashboard.getNumber(prefix + "Start_Y", yStart);
        double newYMiddle = SmartDashboard.getNumber(prefix + "Middle_Y", yMiddle);
        double newYMax = SmartDashboard.getNumber(prefix + "Max_Y", yMax);

        boolean changed = xStart != newXStart
                || xMiddle != newXMiddle
                || yStart != newYStart
                || yMiddle != newYMiddle
                || yMax != newYMax;

        if (changed) {
            xStart = newXStart;
            xMiddle = newXMiddle;
            yStart = newYStart;
            yMiddle = newYMiddle;
            yMax = newYMax;
            sensitivity.set(xStart, xMiddle, yStart, yMiddle, yMax);
        }
    }
}
