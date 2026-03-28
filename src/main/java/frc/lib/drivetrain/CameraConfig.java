package frc.lib.drivetrain;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraConfig(String name, Transform3d robotToCamera, double stdDevMultiplier) {
  /** Creates a CameraConfig with default stdDevMultiplier of 1.0. */
  public CameraConfig(String name, Transform3d robotToCamera) {
    this(name, robotToCamera, 1.0);
  }

  public CameraConfig {
    if (name == null || name.isBlank()) {
      throw new IllegalArgumentException("Camera name must not be null or blank");
    }
    if (robotToCamera == null) {
      throw new IllegalArgumentException("robotToCamera transform must not be null");
    }
    if (stdDevMultiplier <= 0) {
      throw new IllegalArgumentException(
          "stdDevMultiplier must be positive, got " + stdDevMultiplier);
    }
  }
}
