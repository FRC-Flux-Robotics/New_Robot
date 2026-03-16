package frc.lib.drivetrain;

/** No-op IO implementation for log replay. AdvantageKit (S0-5) will fill inputs from log data. */
public class DrivetrainIOReplay implements DrivetrainIO {

  @Override
  public void updateInputs(Inputs inputs) {
    // No-op — replay data will be injected by AdvantageKit
  }
}
