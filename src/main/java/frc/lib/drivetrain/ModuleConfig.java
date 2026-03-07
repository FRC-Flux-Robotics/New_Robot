package frc.lib.drivetrain;

/** Hardware configuration for a single swerve module. */
public final class ModuleConfig {
    public final int driveMotorId;
    public final int steerMotorId;
    public final int encoderId;
    public final double encoderOffsetRotations;
    public final double xPositionMeters;
    public final double yPositionMeters;
    public final boolean invertDrive;
    public final boolean invertSteer;
    public final boolean invertEncoder;

    public ModuleConfig(
            int driveMotorId,
            int steerMotorId,
            int encoderId,
            double encoderOffsetRotations,
            double xPositionMeters,
            double yPositionMeters,
            boolean invertDrive,
            boolean invertSteer,
            boolean invertEncoder) {
        validateCanId("driveMotorId", driveMotorId);
        validateCanId("steerMotorId", steerMotorId);
        validateCanId("encoderId", encoderId);
        if (encoderOffsetRotations < -1.0 || encoderOffsetRotations > 1.0) {
            throw new IllegalArgumentException(
                    "encoderOffsetRotations must be -1.0 to 1.0, got: " + encoderOffsetRotations);
        }

        this.driveMotorId = driveMotorId;
        this.steerMotorId = steerMotorId;
        this.encoderId = encoderId;
        this.encoderOffsetRotations = encoderOffsetRotations;
        this.xPositionMeters = xPositionMeters;
        this.yPositionMeters = yPositionMeters;
        this.invertDrive = invertDrive;
        this.invertSteer = invertSteer;
        this.invertEncoder = invertEncoder;
    }

    private static void validateCanId(String name, int id) {
        if (id < 0 || id > 62) {
            throw new IllegalArgumentException(name + " must be 0-62, got: " + id);
        }
    }
}
