package frc.lib.drivetrain;

import edu.wpi.first.math.geometry.Transform3d;

public record CameraConfig(String name, Transform3d robotToCamera) {
    public CameraConfig {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("Camera name must not be null or blank");
        }
        if (robotToCamera == null) {
            throw new IllegalArgumentException("robotToCamera transform must not be null");
        }
    }
}
