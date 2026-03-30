package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DriveInterface;
import frc.robot.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Automated camera transform calibration. Rotates through 4 orientations, collects vision samples,
 * and recommends transform adjustments for each camera.
 *
 * <p>Reports three levels of information:
 *
 * <ul>
 *   <li><b>Per-camera accuracy</b>: average/max error vs odometry
 *   <li><b>Inter-camera consistency</b>: do cameras agree with each other?
 *   <li><b>Recommended adjustments</b>: signed X/Y/heading corrections for each camera's transform
 * </ul>
 *
 * <p>If signed errors are consistent across all 4 rotation stations, it's a translation offset in
 * the transform. If errors change with rotation, it's a rotation (yaw/pitch) offset. The command
 * detects which case applies and adjusts its recommendations.
 *
 * <p>Sequence: sample 3s → rotate 90° → sample → rotate → sample → rotate → sample → report. Total
 * ~20 seconds.
 */
public class CameraValidationCmd extends Command {
  private static final double CONSISTENCY_THRESHOLD_METERS = 0.10;
  private static final double ABSOLUTE_THRESHOLD_METERS = 0.15;
  private static final double SAMPLE_DURATION_SECONDS = 3.0;
  private static final double ROTATE_DURATION_SECONDS = 2.0;
  private static final double ROTATE_SPEED_RAD_PER_SEC = Math.PI / 4.0; // 45 deg/s
  private static final int NUM_STATIONS = 4; // 0°, 90°, 180°, 270°
  private static final String PREFIX = "CamVal/";

  private final Vision m_vision;
  private final DriveInterface m_drive;
  private final CameraConfig[] m_cameras;

  // Per-camera, per-station signed error tracking
  private double[][] m_stationErrorX; // [camera][station]
  private double[][] m_stationErrorY;
  private double[][] m_stationErrorDeg;
  private int[][] m_stationSampleCount;

  // Per-camera overall magnitude tracking
  private double[] m_errorSum;
  private double[] m_errorMax;
  private int[] m_sampleCount;

  // Inter-camera consistency
  private double m_pairDisagreementSum;
  private double m_pairDisagreementMax;
  private int m_pairCount;

  private final Timer m_timer = new Timer();
  private int m_station;
  private boolean m_rotating;

  public CameraValidationCmd(Vision vision, DriveInterface drive, CameraConfig[] cameras) {
    m_vision = vision;
    m_drive = drive;
    m_cameras = cameras;
    addRequirements(vision, drive);
  }

  @Override
  public void initialize() {
    int camCount = m_vision.getCameraCount();
    m_stationErrorX = new double[camCount][NUM_STATIONS];
    m_stationErrorY = new double[camCount][NUM_STATIONS];
    m_stationErrorDeg = new double[camCount][NUM_STATIONS];
    m_stationSampleCount = new int[camCount][NUM_STATIONS];
    m_errorSum = new double[camCount];
    m_errorMax = new double[camCount];
    m_sampleCount = new int[camCount];
    m_pairDisagreementSum = 0;
    m_pairDisagreementMax = 0;
    m_pairCount = 0;

    m_station = 0;
    m_rotating = false;
    m_timer.restart();
    m_drive.setBrake();

    SmartDashboard.putString(PREFIX + "Status", "Running - Station 0 (sampling)");
    for (int i = 0; i < camCount; i++) {
      SmartDashboard.putString(PREFIX + "Cam" + i, "...");
      SmartDashboard.putString(PREFIX + "Cam" + i + "/Fix", "...");
    }
    SmartDashboard.putString(PREFIX + "Consistency", "...");
    SmartDashboard.putString(PREFIX + "Result", "...");
  }

  @Override
  public void execute() {
    Pose2d odomPose = m_drive.getPose();

    if (m_rotating) {
      m_drive.drive(0, 0, ROTATE_SPEED_RAD_PER_SEC, false, 0.02);
      if (m_timer.hasElapsed(ROTATE_DURATION_SECONDS)) {
        m_rotating = false;
        m_timer.restart();
        m_drive.setBrake();
        SmartDashboard.putString(
            PREFIX + "Status", "Running - Station " + m_station + " (sampling)");
      }
    } else {
      m_drive.drive(0, 0, 0, false, 0.02);

      int camCount = m_vision.getCameraCount();
      Pose2d[] camPoses = new Pose2d[camCount];

      for (int i = 0; i < camCount; i++) {
        camPoses[i] = m_vision.getCameraPose(i);
        if (camPoses[i] == null) continue;

        // Signed errors (vision - odometry)
        double dx = camPoses[i].getX() - odomPose.getX();
        double dy = camPoses[i].getY() - odomPose.getY();
        double dHeading = camPoses[i].getRotation().minus(odomPose.getRotation()).getDegrees();

        // Accumulate per-station (for averaging)
        m_stationErrorX[i][m_station] += dx;
        m_stationErrorY[i][m_station] += dy;
        m_stationErrorDeg[i][m_station] += dHeading;
        m_stationSampleCount[i][m_station]++;

        // Overall magnitude
        double error = Math.hypot(dx, dy);
        m_errorSum[i] += error;
        m_errorMax[i] = Math.max(m_errorMax[i], error);
        m_sampleCount[i]++;
      }

      // Inter-camera disagreement
      for (int i = 0; i < camCount; i++) {
        if (camPoses[i] == null) continue;
        for (int j = i + 1; j < camCount; j++) {
          if (camPoses[j] == null) continue;
          double dist = camPoses[i].getTranslation().getDistance(camPoses[j].getTranslation());
          m_pairDisagreementSum += dist;
          m_pairDisagreementMax = Math.max(m_pairDisagreementMax, dist);
          m_pairCount++;
        }
      }

      if (m_timer.hasElapsed(SAMPLE_DURATION_SECONDS)) {
        m_station++;
        if (m_station < NUM_STATIONS) {
          m_rotating = true;
          m_timer.restart();
          SmartDashboard.putString(PREFIX + "Status", "Running - rotating to Station " + m_station);
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false, 0.02);
    m_timer.stop();

    if (interrupted) {
      SmartDashboard.putString(PREFIX + "Status", "Interrupted");
      return;
    }

    int camCount = m_vision.getCameraCount();
    boolean allAbsPassed = true;
    boolean consistencyPassed = true;
    StringBuilder summary = new StringBuilder();

    // --- Per-camera results ---
    for (int i = 0; i < camCount; i++) {
      String camName = m_vision.getCameraName(i);

      if (m_sampleCount[i] == 0) {
        SmartDashboard.putString(PREFIX + "Cam" + i, camName + ": NO DATA");
        SmartDashboard.putString(PREFIX + "Cam" + i + "/Fix", "No tags seen - move closer");
        Logger.recordOutput(PREFIX + "Cam" + i, camName + ": NO DATA");
        allAbsPassed = false;
        summary.append("NODATA ");
        continue;
      }

      double avgError = m_errorSum[i] / m_sampleCount[i];
      boolean passed = avgError < ABSOLUTE_THRESHOLD_METERS;
      if (!passed) allAbsPassed = false;

      // Per-camera accuracy line
      String result =
          String.format(
              "%s: %s avg=%.3fm max=%.3fm (%d samples)",
              camName, passed ? "PASS" : "FAIL", avgError, m_errorMax[i], m_sampleCount[i]);
      SmartDashboard.putString(PREFIX + "Cam" + i, result);
      Logger.recordOutput(PREFIX + "Cam" + i, result);
      Logger.recordOutput(PREFIX + "Cam" + i + "/AvgError", avgError);
      Logger.recordOutput(PREFIX + "Cam" + i + "/MaxError", m_errorMax[i]);
      Logger.recordOutput(PREFIX + "Cam" + i + "/Samples", m_sampleCount[i]);
      summary.append(passed ? "PASS " : "FAIL ");

      // --- Compute recommended adjustments ---
      computeRecommendation(i, camName);
    }

    // --- Inter-camera consistency ---
    if (m_pairCount > 0) {
      double avgDisagreement = m_pairDisagreementSum / m_pairCount;
      consistencyPassed = avgDisagreement < CONSISTENCY_THRESHOLD_METERS;
      String consistencyResult =
          String.format(
              "%s avg=%.3fm max=%.3fm (%d pairs)",
              consistencyPassed ? "PASS" : "FAIL",
              avgDisagreement,
              m_pairDisagreementMax,
              m_pairCount);
      SmartDashboard.putString(PREFIX + "Consistency", consistencyResult);
      Logger.recordOutput(PREFIX + "Consistency", consistencyResult);
    } else {
      SmartDashboard.putString(PREFIX + "Consistency", "NO DATA (need 2+ cameras with tags)");
      consistencyPassed = false;
    }

    // --- Overall result ---
    String overallResult;
    if (allAbsPassed && consistencyPassed) {
      overallResult = "ALL PASSED — transforms look good";
    } else if (consistencyPassed && !allAbsPassed) {
      overallResult = "CONSISTENCY OK — pose reset may be inaccurate, transforms are fine";
    } else {
      overallResult = "FAILED — check per-camera Fix recommendations";
    }
    SmartDashboard.putString(
        PREFIX + "Result", overallResult + " [" + summary.toString().trim() + "]");
    SmartDashboard.putString(PREFIX + "Status", "Done");
    Logger.recordOutput(PREFIX + "Result", overallResult);
  }

  private void computeRecommendation(int camIdx, String camName) {
    // Average signed error across all stations
    double totalX = 0, totalY = 0, totalDeg = 0;
    int totalSamples = 0;
    double[] stationAvgX = new double[NUM_STATIONS];
    double[] stationAvgY = new double[NUM_STATIONS];
    int stationsWithData = 0;

    for (int s = 0; s < NUM_STATIONS; s++) {
      int n = m_stationSampleCount[camIdx][s];
      if (n == 0) continue;
      stationAvgX[s] = m_stationErrorX[camIdx][s] / n;
      stationAvgY[s] = m_stationErrorY[camIdx][s] / n;
      totalX += m_stationErrorX[camIdx][s];
      totalY += m_stationErrorY[camIdx][s];
      totalDeg += m_stationErrorDeg[camIdx][s];
      totalSamples += n;
      stationsWithData++;
    }

    if (totalSamples == 0) return;

    double avgX = totalX / totalSamples;
    double avgY = totalY / totalSamples;
    double avgDeg = totalDeg / totalSamples;

    // Check consistency across stations: if error changes with rotation, it's a yaw issue
    // Compute standard deviation of per-station averages
    double varX = 0, varY = 0;
    for (int s = 0; s < NUM_STATIONS; s++) {
      if (m_stationSampleCount[camIdx][s] == 0) continue;
      varX += (stationAvgX[s] - avgX) * (stationAvgX[s] - avgX);
      varY += (stationAvgY[s] - avgY) * (stationAvgY[s] - avgY);
    }
    double stdX = stationsWithData > 1 ? Math.sqrt(varX / (stationsWithData - 1)) : 0;
    double stdY = stationsWithData > 1 ? Math.sqrt(varY / (stationsWithData - 1)) : 0;
    boolean errorVariesWithRotation = stdX > 0.05 || stdY > 0.05;

    // Current transform values
    Transform3d currentTransform =
        camIdx < m_cameras.length ? m_cameras[camIdx].robotToCamera() : null;

    // Build recommendation string
    StringBuilder fix = new StringBuilder();

    double absError = Math.hypot(avgX, avgY);
    if (absError < ABSOLUTE_THRESHOLD_METERS && Math.abs(avgDeg) < 3.0) {
      fix.append("Looks good, no changes needed");
    } else {
      if (errorVariesWithRotation) {
        // Error pattern changes with robot rotation → likely a yaw offset error
        fix.append("YAW offset suspected (error varies with rotation). ");
        if (Math.abs(avgDeg) > 1.0) {
          fix.append(String.format("Try adjusting yaw by %.1f deg. ", -avgDeg));
        }
        fix.append(String.format("Spread: stdX=%.3fm stdY=%.3fm across stations", stdX, stdY));
      } else {
        // Consistent error across rotations → translation offset
        // Vision reads too far in +X → camera X offset should increase (move camera forward)
        if (Math.abs(avgX) > 0.02) {
          fix.append(
              String.format(
                  "Adjust X by %+.1f cm (%s). ",
                  -avgX * 100,
                  avgX > 0 ? "camera is further back than measured" : "camera is further fwd"));
        }
        if (Math.abs(avgY) > 0.02) {
          fix.append(
              String.format(
                  "Adjust Y by %+.1f cm (%s). ",
                  -avgY * 100,
                  avgY > 0 ? "camera is further right than measured" : "camera is further left"));
        }
        if (Math.abs(avgDeg) > 1.0) {
          fix.append(String.format("Adjust yaw by %+.1f deg. ", -avgDeg));
        }
      }

      // Show current transform for reference
      if (currentTransform != null) {
        fix.append(
            String.format(
                "| Current: X=%.1fcm Y=%.1fcm yaw=%.1fdeg",
                currentTransform.getX() * 100,
                currentTransform.getY() * 100,
                Math.toDegrees(currentTransform.getRotation().getZ())));
      }
    }

    String fixStr = fix.toString();
    SmartDashboard.putString(PREFIX + "Cam" + camIdx + "/Fix", fixStr);
    Logger.recordOutput(PREFIX + "Cam" + camIdx + "/Fix", fixStr);

    // Log raw signed errors for detailed analysis
    Logger.recordOutput(PREFIX + "Cam" + camIdx + "/AvgErrorX", avgX);
    Logger.recordOutput(PREFIX + "Cam" + camIdx + "/AvgErrorY", avgY);
    Logger.recordOutput(PREFIX + "Cam" + camIdx + "/AvgErrorDeg", avgDeg);
    Logger.recordOutput(
        PREFIX + "Cam" + camIdx + "/ErrorVariesWithRotation", errorVariesWithRotation);
  }

  @Override
  public boolean isFinished() {
    return !m_rotating && m_station >= NUM_STATIONS;
  }
}
