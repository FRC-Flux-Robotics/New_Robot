package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;

/**
 * Alliance-neutral field positions. All positions defined from blue alliance perspective and
 * automatically mirrored for red alliance at runtime.
 *
 * <p>Alliance source priority: FMS game data > dashboard override > blue default.
 */
public final class FieldPositions {

  public static final double FIELD_LENGTH_METERS = 16.54;

  // Blue alliance canonical positions
  private static final Pose2d ORIGIN = new Pose2d();
  private static final Pose2d LEFT = new Pose2d(1.0, 7.0, new Rotation2d());
  private static final Pose2d RIGHT = new Pose2d(1.0, 1.0, new Rotation2d());
  private static final Pose2d HUB = new Pose2d(3.5, 4.1, new Rotation2d());

  private static final Map<String, Pose2d> POSITIONS =
      Map.of(
          "Origin", ORIGIN,
          "Left", LEFT,
          "Right", RIGHT,
          "HUB", HUB);

  private static final SendableChooser<Alliance> s_allianceChooser = new SendableChooser<>();

  private FieldPositions() {}

  /** Publishes the alliance chooser to SmartDashboard. Call once at startup. */
  public static void init() {
    s_allianceChooser.setDefaultOption("Blue", Alliance.Blue);
    s_allianceChooser.addOption("Red", Alliance.Red);
    SmartDashboard.putData("Alliance", s_allianceChooser);
  }

  /**
   * Returns true if currently on red alliance. Uses FMS data when connected, otherwise falls back
   * to dashboard chooser.
   */
  public static boolean isRedAlliance() {
    // FMS data takes priority when available
    if (DriverStation.isFMSAttached()) {
      return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }
    // Dashboard override for practice / testing
    Alliance selected = s_allianceChooser.getSelected();
    return selected != null && selected == Alliance.Red;
  }

  /** Mirrors a blue-alliance pose to red alliance (flips X, rotates 180 deg). */
  public static Pose2d mirror(Pose2d bluePose) {
    return new Pose2d(
        FIELD_LENGTH_METERS - bluePose.getX(),
        bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.fromDegrees(180)));
  }

  /** Returns pose resolved for current alliance. */
  public static Pose2d forAlliance(Pose2d bluePose) {
    return isRedAlliance() ? mirror(bluePose) : bluePose;
  }

  /** Returns translation resolved for current alliance (mirrors X only, no rotation). */
  public static Translation2d forAlliance(Translation2d blueTranslation) {
    if (!isRedAlliance()) return blueTranslation;
    return new Translation2d(FIELD_LENGTH_METERS - blueTranslation.getX(), blueTranslation.getY());
  }

  /** Resolves a named position for the current alliance. Returns null if unknown. */
  public static Pose2d resolve(String name) {
    Pose2d bluePose = POSITIONS.get(name);
    if (bluePose == null) return null;
    // Origin is always (0,0) regardless of alliance
    if (name.equals("Origin")) return bluePose;
    return forAlliance(bluePose);
  }

  /** Returns all position names. */
  public static String[] names() {
    return POSITIONS.keySet().toArray(new String[0]);
  }

  /** Returns the operator forward direction for the current alliance. */
  public static Rotation2d operatorForward() {
    return isRedAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.kZero;
  }
}
