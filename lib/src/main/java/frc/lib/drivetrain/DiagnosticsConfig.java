package frc.lib.drivetrain;

/**
 * Configuration for drivetrain diagnostic thresholds and brownout protection. Extracted from
 * SwerveDrive's hardcoded constants. Use {@link #builder()} to construct.
 */
public final class DiagnosticsConfig {

  public final double motorTempWarnC;
  public final double motorTempErrorC;
  public final double currentWarnFraction;
  public final double alignmentErrorWarnDeg;
  public final double odometryMinHz;
  public final double diagnosticCooldownSec;
  public final double brownoutStartV;
  public final double brownoutMinV;
  public final double brownoutMinScale;

  private DiagnosticsConfig(Builder b) {
    this.motorTempWarnC = b.motorTempWarnC;
    this.motorTempErrorC = b.motorTempErrorC;
    this.currentWarnFraction = b.currentWarnFraction;
    this.alignmentErrorWarnDeg = b.alignmentErrorWarnDeg;
    this.odometryMinHz = b.odometryMinHz;
    this.diagnosticCooldownSec = b.diagnosticCooldownSec;
    this.brownoutStartV = b.brownoutStartV;
    this.brownoutMinV = b.brownoutMinV;
    this.brownoutMinScale = b.brownoutMinScale;
  }

  public static Builder builder() {
    return new Builder();
  }

  /** Returns a config with all default values (matching original SwerveDrive constants). */
  public static DiagnosticsConfig defaults() {
    return builder().build();
  }

  public static final class Builder {
    private double motorTempWarnC = 80.0;
    private double motorTempErrorC = 100.0;
    private double currentWarnFraction = 0.9;
    private double alignmentErrorWarnDeg = 30.0;
    private double odometryMinHz = 50.0;
    private double diagnosticCooldownSec = 2.0;
    private double brownoutStartV = 10.5;
    private double brownoutMinV = 7.0;
    private double brownoutMinScale = 0.25;

    private Builder() {}

    public Builder motorTempWarn(double tempC) {
      requirePositive("motorTempWarnC", tempC);
      this.motorTempWarnC = tempC;
      return this;
    }

    public Builder motorTempError(double tempC) {
      requirePositive("motorTempErrorC", tempC);
      this.motorTempErrorC = tempC;
      return this;
    }

    public Builder currentWarnFraction(double fraction) {
      if (fraction <= 0 || fraction > 1.0) {
        throw new IllegalArgumentException(
            "currentWarnFraction must be > 0 and <= 1.0, got: " + fraction);
      }
      this.currentWarnFraction = fraction;
      return this;
    }

    public Builder alignmentErrorWarn(double deg) {
      requirePositive("alignmentErrorWarnDeg", deg);
      this.alignmentErrorWarnDeg = deg;
      return this;
    }

    public Builder odometryMinHz(double hz) {
      requirePositive("odometryMinHz", hz);
      this.odometryMinHz = hz;
      return this;
    }

    public Builder diagnosticCooldown(double sec) {
      requirePositive("diagnosticCooldownSec", sec);
      this.diagnosticCooldownSec = sec;
      return this;
    }

    public Builder brownout(double startV, double minV, double minScale) {
      requirePositive("brownoutStartV", startV);
      requirePositive("brownoutMinV", minV);
      if (minV >= startV) {
        throw new IllegalArgumentException(
            "brownoutMinV must be < brownoutStartV, got: " + minV + " >= " + startV);
      }
      if (minScale <= 0 || minScale >= 1.0) {
        throw new IllegalArgumentException(
            "brownoutMinScale must be > 0 and < 1.0, got: " + minScale);
      }
      this.brownoutStartV = startV;
      this.brownoutMinV = minV;
      this.brownoutMinScale = minScale;
      return this;
    }

    public DiagnosticsConfig build() {
      return new DiagnosticsConfig(this);
    }

    private static void requirePositive(String name, double value) {
      if (value <= 0) {
        throw new IllegalArgumentException(name + " must be > 0, got: " + value);
      }
    }
  }
}
