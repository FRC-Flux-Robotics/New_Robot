package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Dashboard-tunable mechanism parameters with persistent save/load via WPILib Preferences. On boot,
 * loads last-saved values (or code defaults). Dashboard edits are live. Press "Save" button to
 * persist current values across reboots.
 */
public final class MechanismTuning {

  // --- Keys (Preferences storage + SmartDashboard display) ---
  private static final String PREFIX = "Tuning/";

  // Intake
  private static final String KEY_INTAKE_IN_SPEED = PREFIX + "IntakeInSpeed";
  private static final String KEY_INTAKE_OUT_SPEED = PREFIX + "IntakeOutSpeed";

  // Indexer / Feeder
  private static final String KEY_INDEXER_SPEED = PREFIX + "IndexerSpeed";
  private static final String KEY_FEEDER_SPEED = PREFIX + "FeederSpeed";

  // Shooter
  private static final String KEY_SHOOTER_SPEED = PREFIX + "ShooterSpeed";
  private static final String KEY_SHOOTER_SPEED_STEP = PREFIX + "ShooterSpeedStep";

  // Hood presets (rotations)
  private static final String KEY_HOOD_SHORT = PREFIX + "HoodShort";
  private static final String KEY_HOOD_MID = PREFIX + "HoodMid";
  private static final String KEY_HOOD_LONG = PREFIX + "HoodLong";

  // Shooter speed presets (RPS)
  private static final String KEY_SPEED_SHORT = PREFIX + "SpeedShort";
  private static final String KEY_SPEED_MID = PREFIX + "SpeedMid";
  private static final String KEY_SPEED_LONG = PREFIX + "SpeedLong";

  // Tilter positions (rotations)
  private static final String KEY_TILT_DEPLOY = PREFIX + "TiltDeploy";
  private static final String KEY_TILT_STOW = PREFIX + "TiltStow";

  // Save trigger
  private static final String KEY_SAVE = PREFIX + "Save";

  // --- Code defaults (from MechanismConfigs + RangeTable) ---
  private static final double DEF_INTAKE_IN_SPEED = 60.0;
  private static final double DEF_INTAKE_OUT_SPEED = 60.0;
  private static final double DEF_INDEXER_SPEED = 45.0;
  private static final double DEF_FEEDER_SPEED = 40.0;
  private static final double DEF_SHOOTER_SPEED = 2600.0 / 60.0;
  private static final double DEF_SHOOTER_SPEED_STEP = 100.0 / 60.0;
  private static final double DEF_HOOD_SHORT = 0.0;
  private static final double DEF_HOOD_MID = 2.8;
  private static final double DEF_HOOD_LONG = 6.7;
  private static final double DEF_SPEED_SHORT = 2000.0 / 60.0;
  private static final double DEF_SPEED_MID = 2000.0 / 60.0;
  private static final double DEF_SPEED_LONG = 2300.0 / 60.0;
  private static final double DEF_TILT_DEPLOY = -17.0;
  private static final double DEF_TILT_STOW = 0.0;

  private MechanismTuning() {}

  /** Load saved values from Preferences and publish to SmartDashboard. */
  public static void init() {
    publish(KEY_INTAKE_IN_SPEED, DEF_INTAKE_IN_SPEED);
    publish(KEY_INTAKE_OUT_SPEED, DEF_INTAKE_OUT_SPEED);
    publish(KEY_INDEXER_SPEED, DEF_INDEXER_SPEED);
    publish(KEY_FEEDER_SPEED, DEF_FEEDER_SPEED);
    publish(KEY_SHOOTER_SPEED, DEF_SHOOTER_SPEED);
    publish(KEY_SHOOTER_SPEED_STEP, DEF_SHOOTER_SPEED_STEP);
    publish(KEY_HOOD_SHORT, DEF_HOOD_SHORT);
    publish(KEY_HOOD_MID, DEF_HOOD_MID);
    publish(KEY_HOOD_LONG, DEF_HOOD_LONG);
    publish(KEY_SPEED_SHORT, DEF_SPEED_SHORT);
    publish(KEY_SPEED_MID, DEF_SPEED_MID);
    publish(KEY_SPEED_LONG, DEF_SPEED_LONG);
    publish(KEY_TILT_DEPLOY, DEF_TILT_DEPLOY);
    publish(KEY_TILT_STOW, DEF_TILT_STOW);
    SmartDashboard.putBoolean(KEY_SAVE, false);
  }

  /** Call from periodic — checks save button. */
  public static void periodic() {
    if (SmartDashboard.getBoolean(KEY_SAVE, false)) {
      save();
      SmartDashboard.putBoolean(KEY_SAVE, false);
    }
  }

  // --- Getters (read live from SmartDashboard) ---

  public static double intakeInSpeed() {
    return SmartDashboard.getNumber(KEY_INTAKE_IN_SPEED, DEF_INTAKE_IN_SPEED);
  }

  public static double intakeOutSpeed() {
    return SmartDashboard.getNumber(KEY_INTAKE_OUT_SPEED, DEF_INTAKE_OUT_SPEED);
  }

  public static double indexerSpeed() {
    return SmartDashboard.getNumber(KEY_INDEXER_SPEED, DEF_INDEXER_SPEED);
  }

  public static double feederSpeed() {
    return SmartDashboard.getNumber(KEY_FEEDER_SPEED, DEF_FEEDER_SPEED);
  }

  public static double shooterSpeed() {
    return SmartDashboard.getNumber(KEY_SHOOTER_SPEED, DEF_SHOOTER_SPEED);
  }

  public static double shooterSpeedStep() {
    return SmartDashboard.getNumber(KEY_SHOOTER_SPEED_STEP, DEF_SHOOTER_SPEED_STEP);
  }

  public static double hoodShort() {
    return SmartDashboard.getNumber(KEY_HOOD_SHORT, DEF_HOOD_SHORT);
  }

  public static double hoodMid() {
    return SmartDashboard.getNumber(KEY_HOOD_MID, DEF_HOOD_MID);
  }

  public static double hoodLong() {
    return SmartDashboard.getNumber(KEY_HOOD_LONG, DEF_HOOD_LONG);
  }

  public static double speedShort() {
    return SmartDashboard.getNumber(KEY_SPEED_SHORT, DEF_SPEED_SHORT);
  }

  public static double speedMid() {
    return SmartDashboard.getNumber(KEY_SPEED_MID, DEF_SPEED_MID);
  }

  public static double speedLong() {
    return SmartDashboard.getNumber(KEY_SPEED_LONG, DEF_SPEED_LONG);
  }

  public static double tiltDeploy() {
    return SmartDashboard.getNumber(KEY_TILT_DEPLOY, DEF_TILT_DEPLOY);
  }

  public static double tiltStow() {
    return SmartDashboard.getNumber(KEY_TILT_STOW, DEF_TILT_STOW);
  }

  // --- Persistence ---

  /** Read from Preferences, put on SmartDashboard. */
  private static void publish(String key, double defaultValue) {
    double value = Preferences.getDouble(key, defaultValue);
    SmartDashboard.putNumber(key, value);
  }

  /** Save all current SmartDashboard values to Preferences. */
  private static void save() {
    persist(KEY_INTAKE_IN_SPEED, DEF_INTAKE_IN_SPEED);
    persist(KEY_INTAKE_OUT_SPEED, DEF_INTAKE_OUT_SPEED);
    persist(KEY_INDEXER_SPEED, DEF_INDEXER_SPEED);
    persist(KEY_FEEDER_SPEED, DEF_FEEDER_SPEED);
    persist(KEY_SHOOTER_SPEED, DEF_SHOOTER_SPEED);
    persist(KEY_SHOOTER_SPEED_STEP, DEF_SHOOTER_SPEED_STEP);
    persist(KEY_HOOD_SHORT, DEF_HOOD_SHORT);
    persist(KEY_HOOD_MID, DEF_HOOD_MID);
    persist(KEY_HOOD_LONG, DEF_HOOD_LONG);
    persist(KEY_SPEED_SHORT, DEF_SPEED_SHORT);
    persist(KEY_SPEED_MID, DEF_SPEED_MID);
    persist(KEY_SPEED_LONG, DEF_SPEED_LONG);
    persist(KEY_TILT_DEPLOY, DEF_TILT_DEPLOY);
    persist(KEY_TILT_STOW, DEF_TILT_STOW);
  }

  /** Read current value from SmartDashboard, write to Preferences. */
  private static void persist(String key, double defaultValue) {
    Preferences.setDouble(key, SmartDashboard.getNumber(key, defaultValue));
  }
}
