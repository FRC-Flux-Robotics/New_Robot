package frc.lib.mechanism;

/** No-op IO implementation for log replay. AdvantageKit fills inputs from log data. */
public class MechanismIOReplay implements MechanismIO {

  @Override
  public void updateInputs(MechanismIOInputsAutoLogged inputs) {
    // No-op — replay data will be injected by AdvantageKit
  }
}
