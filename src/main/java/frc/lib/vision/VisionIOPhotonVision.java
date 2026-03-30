package frc.lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.drivetrain.CameraConfig;
import java.util.Comparator;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Real PhotonVision hardware IO. One instance per camera. */
public class VisionIOPhotonVision implements VisionIO {

  private static final int DISCONNECT_THRESHOLD = 50;

  private final PhotonCamera m_camera;
  private final PhotonPoseEstimator m_poseEstimator;
  private int m_disconnectCount = 0;
  private boolean m_connected = true;

  public VisionIOPhotonVision(CameraConfig config, AprilTagFieldLayout fieldLayout) {
    m_camera = new PhotonCamera(config.name());
    m_poseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.robotToCamera());
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  @Override
  public void setTransform(Transform3d robotToCamera) {
    m_poseEstimator.setRobotToCameraTransform(robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputsAutoLogged inputs) {
    var results = m_camera.getAllUnreadResults();

    // Connection tracking
    if (!results.isEmpty()) {
      m_connected = true;
      m_disconnectCount = 0;
    } else {
      m_disconnectCount++;
      if (m_disconnectCount > DISCONNECT_THRESHOLD) {
        if (m_connected) {
          DriverStation.reportWarning(
              "Vision camera " + m_camera.getName() + " disconnected — no frames for 1s", false);
        }
        m_connected = false;
      }
    }
    inputs.connected = m_connected;

    // Default to no pose / no targets
    inputs.posePresent = false;
    inputs.hasTargets = false;
    inputs.targetCount = 0;
    inputs.bestTargetId = -1;
    inputs.bestTargetYaw = 0.0;
    inputs.bestTargetArea = 0.0;

    if (results.isEmpty()) {
      return;
    }

    // Process the latest result
    PhotonPipelineResult latestResult = results.get(results.size() - 1);
    List<PhotonTrackedTarget> targets = latestResult.getTargets();

    // Fill target data
    inputs.hasTargets = latestResult.hasTargets();
    inputs.targetCount = targets.size();
    inputs.targetFiducialIds = new int[targets.size()];
    inputs.targetPoseAmbiguities = new double[targets.size()];
    for (int i = 0; i < targets.size(); i++) {
      inputs.targetFiducialIds[i] = targets.get(i).getFiducialId();
      inputs.targetPoseAmbiguities[i] = targets.get(i).getPoseAmbiguity();
    }

    // Best target (largest area)
    if (inputs.hasTargets) {
      var best = targets.stream().max(Comparator.comparingDouble(PhotonTrackedTarget::getArea));
      if (best.isPresent()) {
        inputs.bestTargetId = best.get().getFiducialId();
        inputs.bestTargetYaw = best.get().getYaw();
        inputs.bestTargetArea = best.get().getArea();
      }
    }

    // Pose estimation
    var visionEst = m_poseEstimator.update(latestResult);
    if (visionEst.isPresent()) {
      var pose = visionEst.get().estimatedPose;
      inputs.posePresent = true;
      inputs.poseX = pose.getX();
      inputs.poseY = pose.getY();
      inputs.poseZ = pose.getZ();
      inputs.poseRotRadians = pose.getRotation().getZ();
      inputs.poseTimestamp = visionEst.get().timestampSeconds;
    }
  }
}
