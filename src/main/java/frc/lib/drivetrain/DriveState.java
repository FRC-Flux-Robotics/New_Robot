package frc.lib.drivetrain;

/** Immutable snapshot of drivetrain state for a single cycle. */
public record DriveState(
        double speedMps,
        double speedPercent,
        double totalCurrentA,
        double[] driveCurrentsA,
        double[] steerCurrentsA,
        double batteryVoltage,
        boolean allHealthy,
        String statusMessage,
        String activeCommand) {
    /** Compact constructor — defensively copies mutable array fields. */
    public DriveState {
        driveCurrentsA = driveCurrentsA != null ? driveCurrentsA.clone() : new double[0];
        steerCurrentsA = steerCurrentsA != null ? steerCurrentsA.clone() : new double[0];
    }
}
