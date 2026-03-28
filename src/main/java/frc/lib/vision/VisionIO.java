package frc.lib.vision;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for vision camera data. One instance per camera. */
public interface VisionIO {

  @AutoLog
  class VisionIOInputs {
    public boolean connected = false;

    // Pose estimation result (invalid when posePresent == false)
    public boolean posePresent = false;
    public double poseX = 0.0;
    public double poseY = 0.0;
    public double poseZ = 0.0;
    public double poseRotRadians = 0.0;
    public double poseTimestamp = -1.0;

    // Target data (parallel arrays, one entry per visible target)
    public int targetCount = 0;
    public int[] targetFiducialIds = new int[0];
    public double[] targetPoseAmbiguities = new double[0];

    // Best target across current frame (for DriveToTag)
    public boolean hasTargets = false;
    public int bestTargetId = -1;
    public double bestTargetYaw = 0.0;
    public double bestTargetArea = 0.0;
  }

  /** Batch-read all camera data into the inputs struct. */
  void updateInputs(VisionIOInputsAutoLogged inputs);
}
