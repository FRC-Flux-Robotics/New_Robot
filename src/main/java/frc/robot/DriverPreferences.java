package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/** Persistent driver-tunable parameters stored on the RoboRIO via WPILib Preferences. */
public final class DriverPreferences {

  private static final String KEY_MAX_SPEED_SCALE = "driver/maxSpeedScale";
  private static final String KEY_MAX_ROTATION_SCALE = "driver/maxRotationScale";
  private static final String KEY_ACCEL_LIMIT = "driver/accelLimit";
  private static final String KEY_ROT_ACCEL_LIMIT = "driver/rotAccelLimit";
  private static final String KEY_SLOW_MODE_SCALE = "driver/slowModeScale";
  private static final String KEY_DECEL_MULTIPLIER = "driver/decelMultiplier";

  private DriverPreferences() {}

  /** Initialize default values if not already set. */
  public static void init() {
    initIfMissing(KEY_MAX_SPEED_SCALE, 1.0);
    initIfMissing(KEY_MAX_ROTATION_SCALE, 1.0);
    initIfMissing(KEY_ACCEL_LIMIT, 3.0);
    initIfMissing(KEY_ROT_ACCEL_LIMIT, 3.0);
    initIfMissing(KEY_SLOW_MODE_SCALE, 0.3);
    initIfMissing(KEY_DECEL_MULTIPLIER, 2.0);
  }

  private static void initIfMissing(String key, double defaultValue) {
    if (!Preferences.containsKey(key)) {
      Preferences.initDouble(key, defaultValue);
    }
  }

  public static double maxSpeedScale() {
    return clamp(Preferences.getDouble(KEY_MAX_SPEED_SCALE, 1.0), 0.0, 1.0);
  }

  public static double maxRotationScale() {
    return clamp(Preferences.getDouble(KEY_MAX_ROTATION_SCALE, 1.0), 0.0, 1.0);
  }

  public static double accelLimit() {
    return clamp(Preferences.getDouble(KEY_ACCEL_LIMIT, 3.0), 0.5, 20.0);
  }

  public static double rotAccelLimit() {
    return clamp(Preferences.getDouble(KEY_ROT_ACCEL_LIMIT, 3.0), 0.5, 20.0);
  }

  public static double slowModeScale() {
    return clamp(Preferences.getDouble(KEY_SLOW_MODE_SCALE, 0.3), 0.05, 0.8);
  }

  public static double decelMultiplier() {
    return clamp(Preferences.getDouble(KEY_DECEL_MULTIPLIER, 2.0), 1.0, 5.0);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
