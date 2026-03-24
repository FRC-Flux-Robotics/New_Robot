package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DriveInterface;

import org.littletonrobotics.junction.Logger;

/** Vision subsystem: PhotonVision pose estimation with dynamic std devs. */
public class Vision extends SubsystemBase {

  private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  private static final int DISCONNECT_THRESHOLD = 50;

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_poseEstimator;
  private final DriveInterface m_drive;

  private PhotonPipelineResult m_latestResult = new PhotonPipelineResult();
  private Matrix<N3, N1> m_curStdDevs = kSingleTagStdDevs;
  private Pose2d m_lastVisionPose = new Pose2d();
  private boolean m_enabled = true;
  private boolean m_connected = true;
  private int m_disconnectCount = 0;

  public Vision(CameraConfig cameraConfig, DriveInterface drive) {
    m_drive = drive;
    m_camera = new PhotonCamera(cameraConfig.name());

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    m_poseEstimator = new PhotonPoseEstimator(
        fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraConfig.robotToCamera());
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void periodic() {
    var results = m_camera.getAllUnreadResults();

    if (!results.isEmpty()) {
      m_latestResult = results.get(results.size() - 1);
      m_connected = true;
      m_disconnectCount = 0;
    } else {
      m_disconnectCount++;
      if (m_disconnectCount > DISCONNECT_THRESHOLD) {
        if (m_connected) {
          DriverStation.reportWarning("Vision camera disconnected — no frames for 1s", false);
        }
        m_connected = false;
      }
    }

    for (var result : results) {
      Optional<EstimatedRobotPose> visionEst = m_poseEstimator.update(result);
      updateStdDevs(visionEst, result.getTargets());

      visionEst.ifPresent(est -> {
        m_lastVisionPose = est.estimatedPose.toPose2d();
        if (m_enabled) {
          m_drive.addVisionMeasurement(
              m_lastVisionPose, est.timestampSeconds, m_curStdDevs);
        }
      });
    }

    // Telemetry
    Logger.recordOutput("Vision/Connected", m_connected);
    Logger.recordOutput("Vision/HasTargets", hasTargets());
    Logger.recordOutput("Vision/TagCount", m_latestResult.hasTargets() ? m_latestResult.getTargets().size() : 0);
    Logger.recordOutput("Vision/BestTagId", getTargetID());
    Logger.recordOutput("Vision/Enabled", m_enabled);
    Logger.recordOutput("Vision/EstimatedPose", m_lastVisionPose);
  }

  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }

  public boolean isEnabled() {
    return m_enabled;
  }

  public boolean isConnected() {
    return m_connected;
  }

  public Pose2d getLastVisionPose() {
    return m_lastVisionPose;
  }

  public boolean hasTargets() {
    return m_latestResult.hasTargets();
  }

  public Optional<PhotonTrackedTarget> getBestTarget() {
    if (!hasTargets() || !m_connected) {
      return Optional.empty();
    }
    return Optional.ofNullable(m_latestResult.getBestTarget());
  }

  public double getTargetYaw() {
    return getBestTarget().map(PhotonTrackedTarget::getYaw).orElse(0.0);
  }

  public double getTargetArea() {
    return getBestTarget().map(PhotonTrackedTarget::getArea).orElse(0.0);
  }

  public int getTargetID() {
    return getBestTarget().map(PhotonTrackedTarget::getFiducialId).orElse(-1);
  }

  private void updateStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      m_curStdDevs = kSingleTagStdDevs;
      return;
    }

    var estStdDevs = kSingleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;

    for (var tgt : targets) {
      var tagPose = m_poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation()
          .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
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
  }
}
