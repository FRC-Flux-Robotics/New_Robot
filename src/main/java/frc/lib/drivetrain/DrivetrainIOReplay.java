package frc.lib.drivetrain;

/** No-op IO implementation for log replay. AdvantageKit fills inputs from log data. */
public class DrivetrainIOReplay implements DrivetrainIO {

  @Override
  public void updateInputs(DrivetrainIOInputsAutoLogged inputs) {
    // No-op — replay data will be injected by AdvantageKit
  }
}
