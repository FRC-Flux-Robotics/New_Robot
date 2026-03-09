package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Driver-tunable parameters stored via {@link Preferences} (persists across reboots).
 * Values appear in SmartDashboard under the Preferences widget and can be edited live.
 */
public final class DriverPreferences {

    // Keys
    private static final String MAX_SPEED_SCALE = "Drive/MaxSpeed%";
    private static final String MAX_ROTATION_SCALE = "Drive/MaxRotation%";
    private static final String DRIVE_EXPO = "Drive/Expo";
    private static final String ROTATION_EXPO = "Drive/RotExpo";
    private static final String DEADBAND = "Drive/Deadband";
    private static final String ACCEL_LIMIT = "Drive/AccelLimit";
    private static final String ROT_ACCEL_LIMIT = "Drive/RotAccelLimit";
    private static final String SLOW_MODE_SCALE = "Drive/SlowMode%";

    // Defaults
    private static final double DEFAULT_MAX_SPEED_SCALE = 1.0;
    private static final double DEFAULT_MAX_ROTATION_SCALE = 1.0;
    private static final double DEFAULT_DRIVE_EXPO = 2.0;
    private static final double DEFAULT_ROTATION_EXPO = 2.0;
    private static final double DEFAULT_DEADBAND = 0.05;
    private static final double DEFAULT_ACCEL_LIMIT = 4.0;
    private static final double DEFAULT_ROT_ACCEL_LIMIT = 5.0;
    private static final double DEFAULT_SLOW_MODE_SCALE = 0.3;

    private DriverPreferences() {}

    /** Initialize all keys with defaults (only sets if key doesn't already exist). */
    public static void init() {
        Preferences.initDouble(MAX_SPEED_SCALE, DEFAULT_MAX_SPEED_SCALE);
        Preferences.initDouble(MAX_ROTATION_SCALE, DEFAULT_MAX_ROTATION_SCALE);
        Preferences.initDouble(DRIVE_EXPO, DEFAULT_DRIVE_EXPO);
        Preferences.initDouble(ROTATION_EXPO, DEFAULT_ROTATION_EXPO);
        Preferences.initDouble(DEADBAND, DEFAULT_DEADBAND);
        Preferences.initDouble(ACCEL_LIMIT, DEFAULT_ACCEL_LIMIT);
        Preferences.initDouble(ROT_ACCEL_LIMIT, DEFAULT_ROT_ACCEL_LIMIT);
        Preferences.initDouble(SLOW_MODE_SCALE, DEFAULT_SLOW_MODE_SCALE);
    }

    /** Max translational speed scale factor (0.0–1.0). */
    public static double maxSpeedScale() {
        return clamp(Preferences.getDouble(MAX_SPEED_SCALE, DEFAULT_MAX_SPEED_SCALE), 0.1, 1.0);
    }

    /** Max rotational speed scale factor (0.0–1.0). */
    public static double maxRotationScale() {
        return clamp(Preferences.getDouble(MAX_ROTATION_SCALE, DEFAULT_MAX_ROTATION_SCALE), 0.1, 1.0);
    }

    /** Drive joystick response curve exponent (1.0 = linear, 2.0 = squared, 3.0 = cubic). */
    public static double driveExpo() {
        return clamp(Preferences.getDouble(DRIVE_EXPO, DEFAULT_DRIVE_EXPO), 1.0, 4.0);
    }

    /** Rotation joystick response curve exponent. */
    public static double rotationExpo() {
        return clamp(Preferences.getDouble(ROTATION_EXPO, DEFAULT_ROTATION_EXPO), 1.0, 4.0);
    }

    /** Joystick deadband threshold. */
    public static double deadband() {
        return clamp(Preferences.getDouble(DEADBAND, DEFAULT_DEADBAND), 0.01, 0.25);
    }

    /** Translational acceleration limit in m/s per second (slew rate). */
    public static double accelLimit() {
        return clamp(Preferences.getDouble(ACCEL_LIMIT, DEFAULT_ACCEL_LIMIT), 1.0, 20.0);
    }

    /** Rotational acceleration limit in rad/s per second (slew rate). */
    public static double rotAccelLimit() {
        return clamp(Preferences.getDouble(ROT_ACCEL_LIMIT, DEFAULT_ROT_ACCEL_LIMIT), 1.0, 20.0);
    }

    /** Slow mode speed scale factor (0.0–1.0). */
    public static double slowModeScale() {
        return clamp(Preferences.getDouble(SLOW_MODE_SCALE, DEFAULT_SLOW_MODE_SCALE), 0.1, 0.8);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
