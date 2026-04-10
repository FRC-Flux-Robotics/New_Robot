package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivetrain.DriveInterface;
import frc.robot.Vision;
import java.util.ArrayList;
import java.util.List;

/**
 * Collects 3 pose measurements from each camera, averages them, and resets odometry. Triggered from
 * the Elastic dashboard. Robot should be stationary for best results.
 */
public class ResetPoseFromVision extends Command {
  private static final int SAMPLES_PER_CAMERA = 3;

  private final Vision m_vision;
  private final DriveInterface m_drive;
  private final int m_cameraCount;

  private List<Pose2d>[] m_perCameraSamples;
  private boolean m_allDone;

  public ResetPoseFromVision(Vision vision, DriveInterface drive) {
    m_vision = vision;
    m_drive = drive;
    m_cameraCount = vision.getCameraCount();
  }

  @Override
  @SuppressWarnings("unchecked")
  public void initialize() {
    m_perCameraSamples = new List[m_cameraCount];
    for (int i = 0; i < m_cameraCount; i++) {
      m_perCameraSamples[i] = new ArrayList<>();
    }
    m_allDone = false;
  }

  @Override
  public void execute() {
    m_allDone = true;
    for (int i = 0; i < m_cameraCount; i++) {
      if (m_perCameraSamples[i].size() >= SAMPLES_PER_CAMERA) continue;
      m_allDone = false;
      Pose2d pose = m_vision.getCameraPose(i);
      if (pose != null) {
        m_perCameraSamples[i].add(pose);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_allDone;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      SmartDashboard.putString("Pose/VisionStatus", "Cancelled");
      return;
    }

    // Gather all samples across all cameras
    List<Pose2d> all = new ArrayList<>();
    for (int i = 0; i < m_cameraCount; i++) {
      all.addAll(m_perCameraSamples[i]);
    }
    if (all.isEmpty()) {
      SmartDashboard.putString("Pose/VisionStatus", "No poses");
      return;
    }

    // Average using sin/cos for correct angle wrapping
    double x = 0, y = 0, sin = 0, cos = 0;
    for (Pose2d p : all) {
      x += p.getX();
      y += p.getY();
      sin += p.getRotation().getSin();
      cos += p.getRotation().getCos();
    }
    int n = all.size();
    Pose2d avg = new Pose2d(x / n, y / n, new Rotation2d(cos / n, sin / n));
    m_drive.resetPose(avg);

    SmartDashboard.putString(
        "Pose/VisionStatus",
        String.format(
            "Set (%.2f, %.2f) %.1f° [%d samples]",
            avg.getX(), avg.getY(), avg.getRotation().getDegrees(), n));
  }
}
