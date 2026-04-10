package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DriveInterface;
import frc.robot.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Self-referencing camera transform calibration. Does NOT depend on accurate absolute odometry.
 *
 * <p>Procedure at each station:
 *
 * <ol>
 *   <li>Stand still, sample vision poses at 4 robot headings (0/90/180/270 relative to start)
 *   <li>Compare each heading's vision pose against the mean of all 4 — a camera translation error
 *       rotates with the robot in field coords, producing a sinusoidal pattern that reveals the
 *       offset direction and magnitude
 *   <li>Drive forward ~1m in the original heading direction
 *   <li>Repeat at the next station
 * </ol>
 *
 * <p>Between stations, compare vision displacement vs odometry displacement. Odometry is accurate
 * for short straight-line moves. A camera yaw error produces position error proportional to tag
 * distance, so it grows across stations and shows up as a displacement mismatch.
 *
 * <p>Sequence: 3 stations, 4 rotations each, ~45 seconds total.
 */
public class CameraValidationCmd extends Command {
  static final double SAMPLE_DURATION_SECONDS = 2.0;
  static final double ROTATE_DURATION_SECONDS = 1.5;
  static final double DRIVE_DURATION_SECONDS = 2.0;
  static final double ROTATE_SPEED_RAD_PER_SEC = Math.PI / 3.0; // 60 deg/s -> 90 in 1.5s
  static final double DRIVE_SPEED_MPS = 0.5; // 0.5 m/s -> 1m in 2s
  static final int NUM_ROTATIONS = 4;
  static final int NUM_STATIONS = 3;
  private static final String PREFIX = "CamVal/";
  private static final String CAMTUNE_PREFIX = "CamTune/";

  private final Vision m_vision;
  private final DriveInterface m_drive;
  private final CameraConfig[] m_cameras;

  // Per-camera, per-station, per-rotation vision pose samples (averaged)
  // [camera][station][rotation] -> accumulated pose X/Y and sample count
  private double[][][] m_rotPoseX;
  private double[][][] m_rotPoseY;
  private int[][][] m_rotSampleCount;

  // Per-station odometry pose (recorded at start of each station)
  private Pose2d[] m_stationOdomPose;

  // Per-station mean vision pose (averaged across all rotations)
  private double[][] m_stationVisionX; // [camera][station]
  private double[][] m_stationVisionY;

  // Computed corrections
  private double[] m_correctionXcm;
  private double[] m_correctionYcm;
  private double[] m_correctionYawDeg;

  private final Timer m_timer = new Timer();
  private Rotation2d m_startHeading;
  private int m_station;
  private int m_rotation;
  private Phase m_phase;
  private Result m_result = Result.INCOMPLETE;

  /** Overall result of a calibration pass. */
  enum Result {
    /** Still running or interrupted. */
    INCOMPLETE,
    /** All cameras passed — no corrections needed. */
    ALL_PASSED,
    /** Corrections computed — apply and rerun for best results. */
    NEEDS_RERUN,
    /** Corrections computed — single apply should fix. */
    FAILED
  }

  enum Phase {
    SAMPLING,
    ROTATING,
    DRIVING
  }

  public CameraValidationCmd(Vision vision, DriveInterface drive, CameraConfig[] cameras) {
    m_vision = vision;
    m_drive = drive;
    m_cameras = cameras;
    addRequirements(vision, drive);
  }

  @Override
  public void initialize() {
    int camCount = m_vision.getCameraCount();
    m_rotPoseX = new double[camCount][NUM_STATIONS][NUM_ROTATIONS];
    m_rotPoseY = new double[camCount][NUM_STATIONS][NUM_ROTATIONS];
    m_rotSampleCount = new int[camCount][NUM_STATIONS][NUM_ROTATIONS];
    m_stationOdomPose = new Pose2d[NUM_STATIONS];
    m_stationVisionX = new double[camCount][NUM_STATIONS];
    m_stationVisionY = new double[camCount][NUM_STATIONS];
    m_correctionXcm = new double[camCount];
    m_correctionYcm = new double[camCount];
    m_correctionYawDeg = new double[camCount];

    m_startHeading = m_drive.getHeading();
    m_station = 0;
    m_rotation = 0;
    m_phase = Phase.SAMPLING;
    m_stationOdomPose[0] = m_drive.getPose();

    m_timer.restart();
    m_drive.setBrake();
    SmartDashboard.putString(PREFIX + "Status", "Running - Station 0, Rotation 0 (sampling)");
  }

  @Override
  public void execute() {
    switch (m_phase) {
      case SAMPLING:
        executeSampling();
        break;
      case ROTATING:
        executeRotating();
        break;
      case DRIVING:
        executeDriving();
        break;
    }
  }

  private void executeSampling() {
    // Hold position
    m_drive.drive(0, 0, 0, false, 0.02);

    // Collect vision samples from all cameras
    int camCount = m_vision.getCameraCount();
    for (int c = 0; c < camCount; c++) {
      Pose2d pose = m_vision.getCameraPose(c);
      if (pose == null) continue;
      m_rotPoseX[c][m_station][m_rotation] += pose.getX();
      m_rotPoseY[c][m_station][m_rotation] += pose.getY();
      m_rotSampleCount[c][m_station][m_rotation]++;
    }

    if (m_timer.hasElapsed(SAMPLE_DURATION_SECONDS)) {
      m_rotation++;
      if (m_rotation < NUM_ROTATIONS) {
        // Rotate to next heading
        m_phase = Phase.ROTATING;
        m_timer.restart();
        SmartDashboard.putString(
            PREFIX + "Status",
            "Running - Station " + m_station + ", rotating to heading " + m_rotation);
      } else {
        // Done with all rotations at this station
        if (m_station + 1 < NUM_STATIONS) {
          // Drive to next station
          m_phase = Phase.DRIVING;
          m_timer.restart();
          SmartDashboard.putString(
              PREFIX + "Status", "Running - driving to Station " + (m_station + 1));
        }
        // else: isFinished() will end the command
      }
    }
  }

  private void executeRotating() {
    // Rotate toward the target heading for this rotation index
    double targetRad =
        m_startHeading.getRadians() + (m_rotation * Math.PI / 2.0); // 0, 90, 180, 270
    double currentRad = m_drive.getHeading().getRadians();
    double error = normalizeAngle(targetRad - currentRad);

    if (Math.abs(error) < Math.toRadians(3.0) || m_timer.hasElapsed(ROTATE_DURATION_SECONDS)) {
      // Close enough or timeout — start sampling
      m_phase = Phase.SAMPLING;
      m_timer.restart();
      m_drive.setBrake();
      SmartDashboard.putString(
          PREFIX + "Status",
          "Running - Station " + m_station + ", Rotation " + m_rotation + " (sampling)");
    } else {
      // P-control rotation toward target
      double rotSpeed = Math.signum(error) * ROTATE_SPEED_RAD_PER_SEC;
      m_drive.drive(0, 0, rotSpeed, false, 0.02);
    }
  }

  private void executeDriving() {
    // Drive forward in the original heading direction (field-relative)
    double cos = m_startHeading.getCos();
    double sin = m_startHeading.getSin();
    m_drive.drive(DRIVE_SPEED_MPS * cos, DRIVE_SPEED_MPS * sin, 0, true, 0.02);

    if (m_timer.hasElapsed(DRIVE_DURATION_SECONDS)) {
      // Arrived at next station — rotate back to starting heading first
      m_station++;
      m_rotation = 0;
      m_stationOdomPose[m_station] = m_drive.getPose();
      m_phase = Phase.ROTATING;
      m_timer.restart();
      m_drive.setBrake();
      SmartDashboard.putString(
          PREFIX + "Status", "Running - Station " + m_station + ", aligning to heading 0");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false, 0.02);
    m_timer.stop();

    if (interrupted) {
      SmartDashboard.putString(PREFIX + "Status", "Interrupted");
      m_result = Result.INCOMPLETE;
      return;
    }

    int camCount = m_vision.getCameraCount();
    boolean allPassed = true;
    StringBuilder summary = new StringBuilder();

    // Compute per-rotation averages and per-station means
    computeStationMeans();

    // Sanity check: detect grossly wrong transforms before fine calibration
    boolean[] grossError = new boolean[camCount];
    for (int c = 0; c < camCount; c++) {
      int totalSamples = 0;
      for (int s = 0; s < NUM_STATIONS; s++) {
        for (int r = 0; r < NUM_ROTATIONS; r++) {
          totalSamples += m_rotSampleCount[c][s][r];
        }
      }

      if (totalSamples == 0) {
        String msg = m_vision.getCameraName(c) + ": NO DATA - camera not seeing any tags!";
        SmartDashboard.putString(PREFIX + "Cam" + c, msg);
        SmartDashboard.putString(PREFIX + "Cam" + c + "/Fix", "Check connection, focus, pipeline");
        Logger.recordOutput(PREFIX + "Cam" + c, msg);
        DriverStation.reportWarning("[CamVal] " + msg, false);
        allPassed = false;
        grossError[c] = true;
        summary.append("NODATA ");
        continue;
      }

      // Check if camera disagrees wildly with odometry displacement
      if (NUM_STATIONS >= 2
          && !Double.isNaN(m_stationVisionX[c][0])
          && !Double.isNaN(m_stationVisionX[c][1])
          && m_stationOdomPose[0] != null
          && m_stationOdomPose[1] != null) {

        double vdx = m_stationVisionX[c][1] - m_stationVisionX[c][0];
        double vdy = m_stationVisionY[c][1] - m_stationVisionY[c][0];
        double odx = m_stationOdomPose[1].getX() - m_stationOdomPose[0].getX();
        double ody = m_stationOdomPose[1].getY() - m_stationOdomPose[0].getY();

        double odomDist = Math.hypot(odx, ody);
        double visionDist = Math.hypot(vdx, vdy);

        if (odomDist > 0.3) {
          // Direction check using angle between vision and odometry displacement
          double dot = vdx * odx + vdy * ody;
          double angleBetween =
              Math.toDegrees(
                  Math.acos(Math.max(-1, Math.min(1, dot / (visionDist * odomDist + 1e-9)))));

          if (angleBetween > 135) {
            // ~180° off — camera pointing backward
            String msg =
                m_vision.getCameraName(c) + ": WRONG DIRECTION - camera may be flipped/backwards!";
            SmartDashboard.putString(PREFIX + "Cam" + c, msg);
            SmartDashboard.putString(
                PREFIX + "Cam" + c + "/Fix",
                String.format("Yaw ~%.0f deg off! May need +/-180 deg correction.", angleBetween));
            Logger.recordOutput(PREFIX + "Cam" + c, msg);
            DriverStation.reportWarning("[CamVal] " + msg, false);
            allPassed = false;
            grossError[c] = true;
            summary.append("FLIPPED ");
            continue;
          } else if (angleBetween > 45) {
            // ~90° off — likely left/right confused or axes swapped
            String msg =
                String.format(
                    "%s: LARGE YAW ERROR - vision ~%.0f deg off from odometry!",
                    m_vision.getCameraName(c), angleBetween);
            SmartDashboard.putString(PREFIX + "Cam" + c, msg);
            SmartDashboard.putString(
                PREFIX + "Cam" + c + "/Fix",
                String.format(
                    "Yaw ~%.0f deg off. Check camera direction - may be rotated/swapped.",
                    angleBetween));
            Logger.recordOutput(PREFIX + "Cam" + c, msg);
            DriverStation.reportWarning("[CamVal] " + msg, false);
            allPassed = false;
            grossError[c] = true;
            summary.append("BIGYAW ");
            continue;
          }

          // Distance check: vision displacement wildly different from odometry
          double distRatio = visionDist / odomDist;
          if (distRatio < 0.3 || distRatio > 3.0) {
            String msg =
                String.format(
                    "%s: BAD SCALE - vision moved %.2fm but odometry moved %.2fm!",
                    m_vision.getCameraName(c), visionDist, odomDist);
            SmartDashboard.putString(PREFIX + "Cam" + c, msg);
            SmartDashboard.putString(
                PREFIX + "Cam" + c + "/Fix",
                "Transform X/Y may be wildly wrong or camera seeing wrong tags");
            Logger.recordOutput(PREFIX + "Cam" + c, msg);
            DriverStation.reportWarning("[CamVal] " + msg, false);
            allPassed = false;
            grossError[c] = true;
            summary.append("BADSCALE ");
            continue;
          }
        }
      }
    }

    boolean needsRerun = false;

    for (int c = 0; c < camCount; c++) {
      if (grossError[c]) continue;

      String camName = m_vision.getCameraName(c);

      // Report per-rotation sample coverage (helps diagnose partial-data cameras)
      int rotationsWithData = 0;
      int totalSamplesThisCam = 0;
      for (int s = 0; s < NUM_STATIONS; s++) {
        for (int r = 0; r < NUM_ROTATIONS; r++) {
          int n = m_rotSampleCount[c][s][r];
          if (n > 0) rotationsWithData++;
          totalSamplesThisCam += n;
        }
      }
      int totalRotations = NUM_STATIONS * NUM_ROTATIONS;
      Logger.recordOutput(
          PREFIX + "Cam" + c + "/Coverage",
          rotationsWithData + "/" + totalRotations + " rotations");

      boolean sparseData = rotationsWithData < totalRotations * 0.6; // <60% coverage

      // --- Translation error from rotation self-consistency ---
      double translationErrX = 0;
      double translationErrY = 0;
      int translationStations = 0;
      double[][] transErr = computeTranslationError(c);

      if (transErr != null) {
        for (int s = 0; s < NUM_STATIONS; s++) {
          if (!Double.isNaN(transErr[0][s])) {
            translationErrX += transErr[0][s];
            translationErrY += transErr[1][s];
            translationStations++;
          }
        }
      }

      // --- Yaw error from displacement mismatch ---
      double yawErrDeg = computeYawError(c);

      // Build corrections
      boolean hasSamples = translationStations > 0;
      if (hasSamples) {
        translationErrX /= translationStations;
        translationErrY /= translationStations;
      }

      // Large yaw error (>15°) corrupts translation — apply yaw only, flag for rerun
      boolean largeYaw = Math.abs(yawErrDeg) > 15.0;
      if (largeYaw) {
        // Translation correction is unreliable when yaw is far off — zero it out
        m_correctionXcm[c] = 0;
        m_correctionYcm[c] = 0;
        m_correctionYawDeg[c] = -yawErrDeg;
        needsRerun = true;
      } else {
        m_correctionXcm[c] = hasSamples ? -translationErrX * 100.0 : 0;
        m_correctionYcm[c] = hasSamples ? -translationErrY * 100.0 : 0;
        m_correctionYawDeg[c] = -yawErrDeg;
      }

      double totalErrCm = Math.hypot(m_correctionXcm[c], m_correctionYcm[c]);
      double totalTransErrCm =
          hasSamples ? Math.hypot(translationErrX * 100.0, translationErrY * 100.0) : 0;
      boolean passed =
          totalTransErrCm < 3.0 && Math.abs(yawErrDeg) < 3.0 && !sparseData && !largeYaw;
      if (!passed) allPassed = false;

      // Large corrections should trigger a rerun
      if (totalErrCm > 10.0 || Math.abs(m_correctionYawDeg[c]) > 10.0) {
        needsRerun = true;
      }

      // Report
      StringBuilder fix = new StringBuilder();
      if (passed) {
        fix.append("Looks good, no changes needed");
        m_correctionXcm[c] = 0;
        m_correctionYcm[c] = 0;
        m_correctionYawDeg[c] = 0;
      } else {
        if (largeYaw) {
          fix.append(
              String.format(
                  "YAW WAY OFF (%+.1f deg) - fixing yaw only, RUN AGAIN for position. ",
                  -yawErrDeg));
        } else {
          if (totalTransErrCm >= 3.0) {
            fix.append(
                String.format(
                    "Adjust X by %+.1f cm, Y by %+.1f cm. ",
                    m_correctionXcm[c], m_correctionYcm[c]));
          }
          if (Math.abs(m_correctionYawDeg[c]) >= 3.0) {
            fix.append(String.format("Adjust yaw by %+.1f deg. ", m_correctionYawDeg[c]));
          }
        }
        if (sparseData) {
          fix.append(
              String.format(
                  "WARNING: only saw tags at %d/%d rotations - low confidence! ",
                  rotationsWithData, totalRotations));
        }
        Transform3d currentTransform = c < m_cameras.length ? m_cameras[c].robotToCamera() : null;
        if (currentTransform != null) {
          fix.append(
              String.format(
                  "| Current: X=%.1fcm Y=%.1fcm yaw=%.1fdeg",
                  currentTransform.getX() * 100,
                  currentTransform.getY() * 100,
                  Math.toDegrees(currentTransform.getRotation().getZ())));
        }
      }

      String status;
      if (largeYaw) {
        status = "BIG YAW";
      } else if (sparseData) {
        status = "LOW DATA";
      } else {
        status = passed ? "PASS" : "FAIL";
      }
      String result =
          String.format(
              "%s: %s transErr=%.1fcm yawErr=%.1fdeg (%d/%d rot)",
              camName,
              status,
              totalTransErrCm,
              Math.abs(yawErrDeg),
              rotationsWithData,
              totalRotations);
      SmartDashboard.putString(PREFIX + "Cam" + c, result);
      SmartDashboard.putString(PREFIX + "Cam" + c + "/Fix", fix.toString());
      Logger.recordOutput(PREFIX + "Cam" + c, result);
      Logger.recordOutput(PREFIX + "Cam" + c + "/Fix", fix.toString());
      Logger.recordOutput(PREFIX + "Cam" + c + "/TransErrCm", totalTransErrCm);
      Logger.recordOutput(PREFIX + "Cam" + c + "/YawErrDeg", yawErrDeg);
      summary.append(status).append(" ");

      // Store corrections on SmartDashboard for AutoApply
      SmartDashboard.putNumber(PREFIX + "Cam" + c + "/CorrXcm", m_correctionXcm[c]);
      SmartDashboard.putNumber(PREFIX + "Cam" + c + "/CorrYcm", m_correctionYcm[c]);
      SmartDashboard.putNumber(PREFIX + "Cam" + c + "/CorrYawDeg", m_correctionYawDeg[c]);
    }

    String overallResult;
    if (allPassed) {
      overallResult = "ALL PASSED - transforms look good";
      m_result = Result.ALL_PASSED;
    } else if (needsRerun) {
      overallResult = "NEEDS RERUN - AutoApply then run again";
      m_result = Result.NEEDS_RERUN;
    } else {
      overallResult = "FAILED - press CamVal/AutoApply to fix";
      m_result = Result.FAILED;
    }
    SmartDashboard.putString(
        PREFIX + "Result", overallResult + " [" + summary.toString().trim() + "]");
    SmartDashboard.putString(
        PREFIX + "Status",
        needsRerun ? "AutoApply then RUN AGAIN (large corrections)" : "Done - AutoApply available");
    SmartDashboard.putBoolean(PREFIX + "AutoApply", false);
    Logger.recordOutput(PREFIX + "Result", overallResult);
  }

  /** Returns the result of the last completed run. */
  public Result getResult() {
    return m_result;
  }

  /**
   * Average per-rotation samples into rotPoseX/Y, then compute per-station mean vision pose into
   * stationVisionX/Y. Modifies arrays in place.
   */
  static void computeStationMeans(
      double[][][] rotPoseX,
      double[][][] rotPoseY,
      int[][][] rotSampleCount,
      double[][] stationVisionX,
      double[][] stationVisionY,
      int camCount,
      int numStations,
      int numRotations) {
    for (int c = 0; c < camCount; c++) {
      for (int s = 0; s < numStations; s++) {
        for (int r = 0; r < numRotations; r++) {
          int n = rotSampleCount[c][s][r];
          if (n > 0) {
            rotPoseX[c][s][r] /= n;
            rotPoseY[c][s][r] /= n;
          }
        }

        double sumX = 0, sumY = 0;
        int count = 0;
        for (int r = 0; r < numRotations; r++) {
          if (rotSampleCount[c][s][r] > 0) {
            sumX += rotPoseX[c][s][r];
            sumY += rotPoseY[c][s][r];
            count++;
          }
        }
        if (count > 0) {
          stationVisionX[c][s] = sumX / count;
          stationVisionY[c][s] = sumY / count;
        } else {
          stationVisionX[c][s] = Double.NaN;
          stationVisionY[c][s] = Double.NaN;
        }
      }
    }
  }

  private void computeStationMeans() {
    computeStationMeans(
        m_rotPoseX,
        m_rotPoseY,
        m_rotSampleCount,
        m_stationVisionX,
        m_stationVisionY,
        m_vision.getCameraCount(),
        NUM_STATIONS,
        NUM_ROTATIONS);
  }

  /**
   * Detect camera translation error from rotation self-consistency at each station.
   *
   * <p>A camera X offset of dx in robot frame produces, in field frame:
   *
   * <ul>
   *   <li>rotation 0 (heading h): error = (+dx*cos(h), +dx*sin(h))
   *   <li>rotation 1 (heading h+90): error = (-dx*sin(h), +dx*cos(h))
   *   <li>rotation 2 (heading h+180): error = (-dx*cos(h), -dx*sin(h))
   *   <li>rotation 3 (heading h+270): error = (+dx*sin(h), -dx*cos(h))
   * </ul>
   *
   * Average across 4 rotations cancels out. But the pattern reveals dx.
   *
   * <p>Per-rotation deviation from station mean, projected back into robot frame:
   *
   * <pre>
   * robotX_err = fieldX_err * cos(heading) + fieldY_err * sin(heading)
   * robotY_err = -fieldX_err * sin(heading) + fieldY_err * cos(heading)
   * </pre>
   *
   * If this is consistent across rotations, it's a real camera offset.
   *
   * @return per-station [X, Y] error in robot frame (meters), or null if no data. NaN for stations
   *     without data.
   */
  static double[][] computeTranslationError(
      int camIdx,
      double[][][] rotPoseX,
      double[][][] rotPoseY,
      int[][][] rotSampleCount,
      double[][] stationVisionX,
      double[][] stationVisionY,
      double startHeadingRad,
      int numStations,
      int numRotations) {
    double[] resultX = new double[numStations];
    double[] resultY = new double[numStations];
    boolean anyData = false;

    for (int s = 0; s < numStations; s++) {
      if (Double.isNaN(stationVisionX[camIdx][s])) {
        resultX[s] = Double.NaN;
        resultY[s] = Double.NaN;
        continue;
      }

      double sumRobotX = 0, sumRobotY = 0;
      int count = 0;
      for (int r = 0; r < numRotations; r++) {
        if (rotSampleCount[camIdx][s][r] == 0) continue;

        double devX = rotPoseX[camIdx][s][r] - stationVisionX[camIdx][s];
        double devY = rotPoseY[camIdx][s][r] - stationVisionY[camIdx][s];

        double heading = startHeadingRad + r * Math.PI / 2.0;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        sumRobotX += devX * cos + devY * sin;
        sumRobotY += -devX * sin + devY * cos;
        count++;
      }

      resultX[s] = count > 0 ? sumRobotX / count : Double.NaN;
      resultY[s] = count > 0 ? sumRobotY / count : Double.NaN;
      if (count > 0) anyData = true;
    }

    return anyData ? new double[][] {resultX, resultY} : null;
  }

  double[][] computeTranslationError(int camIdx) {
    return computeTranslationError(
        camIdx,
        m_rotPoseX,
        m_rotPoseY,
        m_rotSampleCount,
        m_stationVisionX,
        m_stationVisionY,
        m_startHeading.getRadians(),
        NUM_STATIONS,
        NUM_ROTATIONS);
  }

  /**
   * Detect camera yaw error from displacement mismatch between stations.
   *
   * <p>Compares vision displacement (station-to-station) against odometry displacement. A yaw error
   * in the camera transform produces position error proportional to tag distance, so it grows as
   * the robot moves away from tags. The displacement mismatch reveals the yaw offset.
   *
   * @return estimated yaw error in degrees (positive = camera yaw should increase)
   */
  static double computeYawError(
      int camIdx,
      double[][] stationVisionX,
      double[][] stationVisionY,
      Pose2d[] stationOdomPose,
      int numStations) {
    if (numStations < 2) return 0;

    double totalAngularErr = 0;
    int count = 0;

    for (int s = 1; s < numStations; s++) {
      if (Double.isNaN(stationVisionX[camIdx][s]) || Double.isNaN(stationVisionX[camIdx][s - 1])) {
        continue;
      }
      if (stationOdomPose[s] == null || stationOdomPose[s - 1] == null) {
        continue;
      }

      double vdx = stationVisionX[camIdx][s] - stationVisionX[camIdx][s - 1];
      double vdy = stationVisionY[camIdx][s] - stationVisionY[camIdx][s - 1];

      double odx = stationOdomPose[s].getX() - stationOdomPose[s - 1].getX();
      double ody = stationOdomPose[s].getY() - stationOdomPose[s - 1].getY();

      double odomDist = Math.hypot(odx, ody);
      if (odomDist < 0.1) continue;

      double visionAngle = Math.atan2(vdy, vdx);
      double odomAngle = Math.atan2(ody, odx);
      double angleDiff = normalizeAngle(visionAngle - odomAngle);

      totalAngularErr += angleDiff;
      count++;
    }

    return count > 0 ? Math.toDegrees(totalAngularErr / count) : 0;
  }

  double computeYawError(int camIdx) {
    return computeYawError(
        camIdx, m_stationVisionX, m_stationVisionY, m_stationOdomPose, NUM_STATIONS);
  }

  @Override
  public boolean isFinished() {
    // Done after last rotation at last station finishes sampling
    return m_phase == Phase.SAMPLING
        && m_station >= NUM_STATIONS - 1
        && m_rotation >= NUM_ROTATIONS
        && m_timer.hasElapsed(SAMPLE_DURATION_SECONDS);
  }

  /** Normalize angle to [-pi, pi]. */
  static double normalizeAngle(double radians) {
    while (radians > Math.PI) radians -= 2.0 * Math.PI;
    while (radians < -Math.PI) radians += 2.0 * Math.PI;
    return radians;
  }

  /**
   * Checks SmartDashboard for the AutoApply button and applies stored corrections to the CamTune
   * values. Call this from Vision.periodic() so it works even after the command has finished.
   */
  public static void checkAutoApply(int cameraCount) {
    if (!SmartDashboard.getBoolean(PREFIX + "AutoApply", false)) return;
    SmartDashboard.putBoolean(PREFIX + "AutoApply", false);

    for (int i = 0; i < cameraCount; i++) {
      String tunePrefix = CAMTUNE_PREFIX + "Cam" + i + "/";
      String valPrefix = PREFIX + "Cam" + i + "/";

      double corrXcm = SmartDashboard.getNumber(valPrefix + "CorrXcm", 0);
      double corrYcm = SmartDashboard.getNumber(valPrefix + "CorrYcm", 0);
      double corrYawDeg = SmartDashboard.getNumber(valPrefix + "CorrYawDeg", 0);

      if (Math.abs(corrXcm) < 1.0 && Math.abs(corrYcm) < 1.0 && Math.abs(corrYawDeg) < 1.0) {
        continue;
      }

      double curX = SmartDashboard.getNumber(tunePrefix + "X_cm", 0);
      double curY = SmartDashboard.getNumber(tunePrefix + "Y_cm", 0);
      double curYaw = SmartDashboard.getNumber(tunePrefix + "Yaw_deg", 0);

      SmartDashboard.putNumber(tunePrefix + "X_cm", curX + corrXcm);
      SmartDashboard.putNumber(tunePrefix + "Y_cm", curY + corrYcm);
      SmartDashboard.putNumber(tunePrefix + "Yaw_deg", curYaw + corrYawDeg);
    }

    SmartDashboard.putBoolean(CAMTUNE_PREFIX + "Apply", true);
    DriverStation.reportWarning(
        "CamVal AutoApply: corrections applied to CamTune - verify and Save if happy", false);
    SmartDashboard.putString(PREFIX + "Status", "AutoApply done - check CamTune values, then Save");
  }
}
