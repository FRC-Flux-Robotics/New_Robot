package frc.lib.vision;

/** No-op vision IO for AdvantageKit log replay. */
public class VisionIOReplay implements VisionIO {
  @Override
  public void updateInputs(VisionIOInputsAutoLogged inputs) {
    // No-op — AdvantageKit injects replay data directly
  }
}
