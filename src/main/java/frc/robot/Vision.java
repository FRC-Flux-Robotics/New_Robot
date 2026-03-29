package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DriveInterface;
import frc.lib.vision.VisionIO;
import frc.lib.vision.VisionIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/** Vision subsystem: processes VisionIO inputs for pose estimation with dynamic std devs. */
public class Vision extends SubsystemBase {

  private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  // Vision rejection thresholds
  static final double MAX_AMBIGUITY = 0.3;
  static final double FIELD_LENGTH_METERS = 16.54;
  static final double FIELD_WIDTH_METERS = 8.21;
  static final double FIELD_MARGIN_METERS = 0.5;
  static final double MAX_Z_ERROR_METERS = 0.75;
  static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.toRadians(120);
  static final double MAX_LINEAR_SPEED_MPS = 3.0;

  private static final String kUseNewStdDevsKey = "Vision/UseNewStdDevs";

  private final VisionIO[] m_ios;
  private final CameraConfig[] m_cameras;
  private final VisionIOInputsAutoLogged[] m_inputs;
  private final AprilTagFieldLayout m_fieldLayout;
  private final DriveInterface m_drive;
  private final int[][] m_rejectionCounts;

  private volatile Matrix<N3, N1> m_curStdDevs = kSingleTagStdDevs;
  private volatile Pose2d m_lastVisionPose = new Pose2d();
  private boolean m_enabled = true;

  // Cached per-periodic best target (avoids re-scanning all cameras per getter)
  private boolean m_cachedConnected = false;
  private boolean m_cachedHasTargets = false;
  private double m_cachedBestYaw = 0;
  private double m_cachedBestArea = 0;
  private int m_cachedBestId = -1;

  public Vision(VisionIO[] ios, CameraConfig[] cameras, DriveInterface drive) {
    m_ios = ios;
    m_cameras = cameras;
    m_drive = drive;
    m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    m_inputs = new VisionIOInputsAutoLogged[ios.length];
    m_rejectionCounts = new int[ios.length][VisionRejectReason.values().length];
    for (int i = 0; i < ios.length; i++) {
      m_inputs[i] = new VisionIOInputsAutoLogged();
    }

    SmartDashboard.putBoolean(kUseNewStdDevsKey, true);
  }

  @Override
  public void periodic() {
    for (int i = 0; i < m_ios.length; i++) {
      m_ios[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Vision/Camera" + i, m_inputs[i]);

      var inputs = m_inputs[i];
      String prefix = "Vision/Camera" + i + "/";

      if (inputs.posePresent) {
        Pose3d estimatedPose3d =
            new Pose3d(
                inputs.poseX,
                inputs.poseY,
                inputs.poseZ,
                new Rotation3d(0, 0, inputs.poseRotRadians));
        Pose2d estimatedPose2d = estimatedPose3d.toPose2d();

        updateStdDevs(inputs, i);
        m_lastVisionPose = estimatedPose2d;

        var speeds = m_drive.getVelocity();
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);

        VisionRejectReason rejection =
            checkRejection(estimatedPose3d, inputs, linearSpeed, angularSpeed);
        Logger.recordOutput(prefix + "Rejected", rejection != null);
        Logger.recordOutput(prefix + "RejectionReason", rejection != null ? rejection.name() : "");

        if (rejection != null) {
          m_rejectionCounts[i][rejection.ordinal()]++;
        }

        if (m_enabled && rejection == null) {
          m_drive.addVisionMeasurement(estimatedPose2d, inputs.poseTimestamp, m_curStdDevs);
        }
      }

      Logger.recordOutput(prefix + "Connected", inputs.connected);
      Logger.recordOutput(prefix + "HasTargets", inputs.hasTargets);
      Logger.recordOutput(prefix + "TagCount", inputs.targetCount);
    }

    // Log rejection counts per reason per camera
    for (int i = 0; i < m_ios.length; i++) {
      String prefix = "Vision/Camera" + i + "/Rejections/";
      for (VisionRejectReason reason : VisionRejectReason.values()) {
        Logger.recordOutput(prefix + reason.name(), m_rejectionCounts[i][reason.ordinal()]);
      }
    }

    // Cache best target across all cameras (single scan)
    boolean connected = false;
    boolean hasTargets = false;
    double bestArea = 0;
    double bestYaw = 0;
    int bestId = -1;
    StringBuilder visibleIds = new StringBuilder();
    for (var inputs : m_inputs) {
      if (inputs.connected) connected = true;
      if (inputs.connected && inputs.hasTargets) {
        hasTargets = true;
        if (inputs.bestTargetArea > bestArea) {
          bestArea = inputs.bestTargetArea;
          bestYaw = inputs.bestTargetYaw;
          bestId = inputs.bestTargetId;
        }
        for (int t = 0; t < inputs.targetCount; t++) {
          int id = inputs.targetFiducialIds[t];
          // Avoid duplicates from multiple cameras seeing same tag
          String idStr = String.valueOf(id);
          if (visibleIds.indexOf(idStr) == -1) {
            if (visibleIds.length() > 0) visibleIds.append(", ");
            visibleIds.append(idStr);
          }
        }
      }
    }
    m_cachedConnected = connected;
    m_cachedHasTargets = hasTargets;
    m_cachedBestArea = bestArea;
    m_cachedBestYaw = bestYaw;
    m_cachedBestId = bestId;

    // Aggregate telemetry
    Logger.recordOutput("Vision/Connected", m_cachedConnected);
    Logger.recordOutput("Vision/HasTargets", m_cachedHasTargets);
    Logger.recordOutput("Vision/BestTagId", m_cachedBestId);
    Logger.recordOutput("Vision/Enabled", m_enabled);
    Logger.recordOutput("Vision/UseNewStdDevs", SmartDashboard.getBoolean(kUseNewStdDevsKey, true));
    Logger.recordOutput("Vision/AutoHeadingDisabled", DriverStation.isAutonomous());
    Logger.recordOutput("Vision/EstimatedPose", m_lastVisionPose);

    // Dashboard-readable vision pose and visible tags
    Pose2d visionPose = m_lastVisionPose;
    SmartDashboard.putNumber("Vision/PoseX", visionPose.getX());
    SmartDashboard.putNumber("Vision/PoseY", visionPose.getY());
    SmartDashboard.putNumber("Vision/PoseHeading", visionPose.getRotation().getDegrees());
    SmartDashboard.putString("Vision/VisibleTagIDs", visibleIds.toString());
  }

  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }

  public boolean isEnabled() {
    return m_enabled;
  }

  public boolean isConnected() {
    return m_cachedConnected;
  }

  public Pose2d getLastVisionPose() {
    return m_lastVisionPose;
  }

  public boolean hasTargets() {
    return m_cachedHasTargets;
  }

  public double getTargetYaw() {
    return m_cachedBestYaw;
  }

  public double getTargetArea() {
    return m_cachedBestArea;
  }

  public int getTargetID() {
    return m_cachedBestId;
  }

  /** Returns null if measurement is acceptable, or a {@link VisionRejectReason}. */
  static VisionRejectReason checkRejection(
      Pose3d estimatedPose,
      VisionIO.VisionIOInputs inputs,
      double linearSpeedMps,
      double angularSpeedRadPerSec) {
    // Single-tag ambiguity check
    if (inputs.targetCount == 1 && inputs.targetPoseAmbiguities[0] > MAX_AMBIGUITY) {
      return VisionRejectReason.TOO_AMBIGUOUS;
    }
    // Pose outside field bounds
    double x = estimatedPose.getX();
    double y = estimatedPose.getY();
    if (x < -FIELD_MARGIN_METERS
        || x > FIELD_LENGTH_METERS + FIELD_MARGIN_METERS
        || y < -FIELD_MARGIN_METERS
        || y > FIELD_WIDTH_METERS + FIELD_MARGIN_METERS) {
      return VisionRejectReason.OUT_OF_FIELD;
    }
    // Z-error too large
    if (Math.abs(estimatedPose.getZ()) > MAX_Z_ERROR_METERS) {
      return VisionRejectReason.Z_ERROR;
    }
    // Robot spinning too fast
    if (angularSpeedRadPerSec > MAX_ANGULAR_SPEED_RAD_PER_SEC) {
      return VisionRejectReason.ANGULAR_VEL_TOO_HIGH;
    }
    // Robot moving too fast
    if (linearSpeedMps > MAX_LINEAR_SPEED_MPS) {
      return VisionRejectReason.MOVING_TOO_FAST;
    }
    return null;
  }

  private void updateStdDevs(VisionIO.VisionIOInputs inputs, int cameraIndex) {
    if (!inputs.posePresent) {
      m_curStdDevs = kSingleTagStdDevs;
      return;
    }

    var estStdDevs = kSingleTagStdDevs;
    Pose2d estimatedPose2d =
        new Pose2d(inputs.poseX, inputs.poseY, new Rotation2d(inputs.poseRotRadians));
    int numTags = 0;
    double avgDist = 0;

    for (int i = 0; i < inputs.targetCount; i++) {
      var tagPose = m_fieldLayout.getTagPose(inputs.targetFiducialIds[i]);
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose2d.getTranslation());
    }

    if (numTags == 0) {
      m_curStdDevs = kSingleTagStdDevs;
    } else {
      avgDist /= numTags;
      if (numTags > 1) estStdDevs = kMultiTagStdDevs;
      if (numTags == 1 && avgDist > 4) {
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      } else if (SmartDashboard.getBoolean(kUseNewStdDevsKey, true)) {
        // New formula: distance² / tagCount with floor at 1.0, per-camera multiplier
        double cameraMultiplier =
            cameraIndex < m_cameras.length ? m_cameras[cameraIndex].stdDevMultiplier() : 1.0;
        double scaleFactor = Math.max(1.0, avgDist * avgDist / numTags) * cameraMultiplier;
        estStdDevs = estStdDevs.times(scaleFactor);
      } else {
        // Old formula: 1 + distance² / 30
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
      }
      m_curStdDevs = estStdDevs;
    }

    // During autonomous, disable heading correction so PathPlanner owns heading
    if (DriverStation.isAutonomous()) {
      m_curStdDevs = m_curStdDevs.copy();
      m_curStdDevs.set(2, 0, Double.MAX_VALUE);
    }
  }
}
