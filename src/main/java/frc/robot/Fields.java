package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;

public final class Fields {
    public static final AprilTagFieldLayout COMPETITION =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final AprilTagFieldLayout TRAINING = loadTrainingField();

    private Fields() {}

    private static AprilTagFieldLayout loadTrainingField() {
        try {
            return new AprilTagFieldLayout(
                    Path.of(Filesystem.getDeployDirectory().getAbsolutePath(),
                            "fields", "training-field.json"));
        } catch (IOException e) {
            throw new UncheckedIOException("Failed to load training field layout", e);
        }
    }
}
