package frc.lib.drivetrain;

/** Immutable snapshot of drivetrain state for a single cycle. */
public record DriveState(
        double speedMps,
        double speedPercent,
        double totalCurrentA,
        double[] driveCurrentsA,
        double[] steerCurrentsA,
        double batteryVoltage,
        boolean brownoutActive,
        double brownoutScale,
        boolean allHealthy,
        String statusMessage,
        String activeCommand) {}
