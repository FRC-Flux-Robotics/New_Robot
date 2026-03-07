package frc.lib.drivetrain;

/**
 * No-op IO implementation for log replay. During replay, AdvantageKit automatically
 * fills inputs from the log file via Logger.processInputs().
 */
public class DrivetrainIOSim implements DrivetrainIO {
    @Override
    public void updateInputs(DrivetrainIOInputsAutoLogged inputs) {
        // No-op: AdvantageKit fills inputs from log during replay
    }
}
