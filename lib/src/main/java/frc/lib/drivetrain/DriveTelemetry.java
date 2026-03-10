package frc.lib.drivetrain;

/** Receives drivetrain state snapshots each cycle for dashboard publishing. */
@FunctionalInterface
public interface DriveTelemetry {

  /** Called every cycle with the current drivetrain state. */
  void update(DriveState state);
}
