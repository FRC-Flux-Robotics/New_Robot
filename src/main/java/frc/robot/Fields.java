package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;

/**
 * Field layout constants. COMPETITION loads eagerly (always available).
 * Training field loads lazily via {@link #training()} to avoid crashing
 * if the deploy file is missing.
 */
public final class Fields {
    public static final AprilTagFieldLayout COMPETITION =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private static AprilTagFieldLayout trainingLayout;

    private Fields() {}

    public static AprilTagFieldLayout training() {
        if (trainingLayout == null) {
            try {
                trainingLayout = new AprilTagFieldLayout(
                        Path.of(Filesystem.getDeployDirectory().getAbsolutePath(),
                                "fields", "training-field.json"));
            } catch (IOException e) {
                throw new UncheckedIOException("Failed to load training field layout", e);
            }
        }
        return trainingLayout;
    }
}
