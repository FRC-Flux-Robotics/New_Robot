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
 * <p>Alliance source priority: FMS (at competition) → dashboard chooser (practice/testing). The
 * dashboard chooser defaults to "Auto" which uses DriverStation, or can be forced to Blue/Red.
 */
public final class FieldPositions {

  public static final double FIELD_LENGTH_METERS = 16.54;

  private static final String AUTO = "auto";
  private static final String BLUE = "blue";
  private static final String RED = "red";

  private static final SendableChooser<String> allianceChooser = new SendableChooser<>();

  // Blue alliance canonical positions
  private static final Pose2d ORIGIN = new Pose2d();
  private static final Pose2d LEFT = new Pose2d(3.227, 6.55, new Rotation2d());
  private static final Pose2d RIGHT = new Pose2d(3.227, 1.55, new Rotation2d());
  private static final Pose2d HUB = new Pose2d(3.227, 4.05, new Rotation2d());

  private static final Map<String, Pose2d> POSITIONS =
      Map.of(
          "Origin", ORIGIN,
          "Left", LEFT,
          "Right", RIGHT,
          "HUB", HUB);

  private FieldPositions() {}

  /** Publish alliance chooser to SmartDashboard. Call once from RobotContainer. */
  public static void init() {
    allianceChooser.setDefaultOption("Auto (from DS)", AUTO);
    allianceChooser.addOption("Blue", BLUE);
    allianceChooser.addOption("Red", RED);
    SmartDashboard.putData("Alliance", allianceChooser);
  }

  /**
   * Returns true if currently on red alliance. Checks dashboard override first, then falls back to
   * DriverStation (FMS or DS software). Publish resolved value so operators can verify.
   */
  public static boolean isRedAlliance() {
    String selected = allianceChooser.getSelected();
    boolean red;
    if (RED.equals(selected)) {
      red = true;
    } else if (BLUE.equals(selected)) {
      red = false;
    } else {
      red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }
    SmartDashboard.putString("Alliance/Active", red ? "Red" : "Blue");
    return red;
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
