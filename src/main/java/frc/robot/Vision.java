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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  private final VisionIO[] m_ios;
  private final VisionIOInputsAutoLogged[] m_inputs;
  private final AprilTagFieldLayout m_fieldLayout;
  private final DriveInterface m_drive;

  private Matrix<N3, N1> m_curStdDevs = kSingleTagStdDevs;
  private Pose2d m_lastVisionPose = new Pose2d();
  private boolean m_enabled = true;

  public Vision(VisionIO[] ios, DriveInterface drive) {
    m_ios = ios;
    m_drive = drive;
    m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    m_inputs = new VisionIOInputsAutoLogged[ios.length];
    for (int i = 0; i < ios.length; i++) {
      m_inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < m_ios.length; i++) {
      m_ios[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Vision/Camera" + i, m_inputs[i]);

      var inputs = m_inputs[i];
      String prefix = "Vision/Camera" + i + "/";

      if (inputs.posePresent) {
        Pose3d estimatedPose3d = new Pose3d(
            inputs.poseX, inputs.poseY, inputs.poseZ,
            new Rotation3d(0, 0, inputs.poseRotRadians));
        Pose2d estimatedPose2d = estimatedPose3d.toPose2d();

        updateStdDevs(inputs);
        m_lastVisionPose = estimatedPose2d;

        var speeds = m_drive.getVelocity();
        double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);

        String rejection = checkRejection(estimatedPose3d, inputs, linearSpeed, angularSpeed);
        Logger.recordOutput(prefix + "Rejected", rejection != null);
        Logger.recordOutput(prefix + "RejectionReason", rejection != null ? rejection : "");

        if (m_enabled && rejection == null) {
          m_drive.addVisionMeasurement(estimatedPose2d, inputs.poseTimestamp, m_curStdDevs);
        }
      }

      Logger.recordOutput(prefix + "Connected", inputs.connected);
      Logger.recordOutput(prefix + "HasTargets", inputs.hasTargets);
      Logger.recordOutput(prefix + "TagCount", inputs.targetCount);
    }

    // Aggregate telemetry
    Logger.recordOutput("Vision/Connected", isConnected());
    Logger.recordOutput("Vision/HasTargets", hasTargets());
    Logger.recordOutput("Vision/BestTagId", getTargetID());
    Logger.recordOutput("Vision/Enabled", m_enabled);
    Logger.recordOutput("Vision/AutoHeadingDisabled", DriverStation.isAutonomous());
    Logger.recordOutput("Vision/EstimatedPose", m_lastVisionPose);
  }

  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }

  public boolean isEnabled() {
    return m_enabled;
  }

  public boolean isConnected() {
    for (var inputs : m_inputs) {
      if (inputs.connected) return true;
    }
    return false;
  }

  public Pose2d getLastVisionPose() {
    return m_lastVisionPose;
  }

  public boolean hasTargets() {
    for (var inputs : m_inputs) {
      if (inputs.connected && inputs.hasTargets) return true;
    }
    return false;
  }

  public double getTargetYaw() {
    double bestArea = 0;
    double bestYaw = 0;
    for (var inputs : m_inputs) {
      if (inputs.connected && inputs.hasTargets && inputs.bestTargetArea > bestArea) {
        bestArea = inputs.bestTargetArea;
        bestYaw = inputs.bestTargetYaw;
      }
    }
    return bestYaw;
  }

  public double getTargetArea() {
    double bestArea = 0;
    for (var inputs : m_inputs) {
      if (inputs.connected && inputs.hasTargets && inputs.bestTargetArea > bestArea) {
        bestArea = inputs.bestTargetArea;
      }
    }
    return bestArea;
  }

  public int getTargetID() {
    double bestArea = 0;
    int bestId = -1;
    for (var inputs : m_inputs) {
      if (inputs.connected && inputs.hasTargets && inputs.bestTargetArea > bestArea) {
        bestArea = inputs.bestTargetArea;
        bestId = inputs.bestTargetId;
      }
    }
    return bestId;
  }

  /** Returns null if measurement is acceptable, or a rejection reason string. */
  static String checkRejection(Pose3d estimatedPose, VisionIO.VisionIOInputs inputs,
                                double linearSpeedMps, double angularSpeedRadPerSec) {
    // Single-tag ambiguity check
    if (inputs.targetCount == 1 && inputs.targetPoseAmbiguities[0] > MAX_AMBIGUITY) {
      return "ambiguity";
    }
    // Pose outside field bounds
    double x = estimatedPose.getX();
    double y = estimatedPose.getY();
    if (x < -FIELD_MARGIN_METERS || x > FIELD_LENGTH_METERS + FIELD_MARGIN_METERS
        || y < -FIELD_MARGIN_METERS || y > FIELD_WIDTH_METERS + FIELD_MARGIN_METERS) {
      return "out_of_field";
    }
    // Z-error too large
    if (Math.abs(estimatedPose.getZ()) > MAX_Z_ERROR_METERS) {
      return "z_error";
    }
    // Robot spinning too fast
    if (angularSpeedRadPerSec > MAX_ANGULAR_SPEED_RAD_PER_SEC) {
      return "spinning";
    }
    // Robot moving too fast
    if (linearSpeedMps > MAX_LINEAR_SPEED_MPS) {
      return "too_fast";
    }
    return null;
  }

  private void updateStdDevs(VisionIO.VisionIOInputs inputs) {
    if (!inputs.posePresent) {
      m_curStdDevs = kSingleTagStdDevs;
      return;
    }

    var estStdDevs = kSingleTagStdDevs;
    Pose2d estimatedPose2d = new Pose2d(inputs.poseX, inputs.poseY,
        new Rotation2d(inputs.poseRotRadians));
    int numTags = 0;
    double avgDist = 0;

    for (int i = 0; i < inputs.targetCount; i++) {
      var tagPose = m_fieldLayout.getTagPose(inputs.targetFiducialIds[i]);
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation()
          .getDistance(estimatedPose2d.getTranslation());
    }

    if (numTags == 0) {
      m_curStdDevs = kSingleTagStdDevs;
    } else {
      avgDist /= numTags;
      if (numTags > 1) estStdDevs = kMultiTagStdDevs;
      if (numTags == 1 && avgDist > 4) {
        estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      } else {
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
