package frc.lib.drivetrain;

/**
 * Configuration for vision-based pose estimation filtering. Extracted from SwerveDrive's hardcoded
 * constants. Use {@link #builder()} to construct.
 */
public final class VisionConfig {

  public final double maxAmbiguity;
  public final double maxSingleTagDistM;
  public final double zHeightThresholdM;
  public final double linearStdDevCoeff;
  public final double angularStdDevCoeff;
  public final double fieldMaxX;
  public final double fieldMaxY;

  private VisionConfig(Builder b) {
    this.maxAmbiguity = b.maxAmbiguity;
    this.maxSingleTagDistM = b.maxSingleTagDistM;
    this.zHeightThresholdM = b.zHeightThresholdM;
    this.linearStdDevCoeff = b.linearStdDevCoeff;
    this.angularStdDevCoeff = b.angularStdDevCoeff;
    this.fieldMaxX = b.fieldMaxX;
    this.fieldMaxY = b.fieldMaxY;
  }

  public static Builder builder() {
    return new Builder();
  }

  /** Returns a config with all default values. */
  public static VisionConfig defaults() {
    return builder().build();
  }

  public static final class Builder {
    private double maxAmbiguity = 0.2;
    private double maxSingleTagDistM = 4.0;
    private double zHeightThresholdM = 0.75;
    private double linearStdDevCoeff = 0.02;
    private double angularStdDevCoeff = 0.06;
    private double fieldMaxX = 17.0;
    private double fieldMaxY = 8.7;

    private Builder() {}

    public Builder maxAmbiguity(double maxAmbiguity) {
      requirePositive("maxAmbiguity", maxAmbiguity);
      this.maxAmbiguity = maxAmbiguity;
      return this;
    }

    public Builder maxSingleTagDist(double maxSingleTagDistM) {
      requirePositive("maxSingleTagDistM", maxSingleTagDistM);
      this.maxSingleTagDistM = maxSingleTagDistM;
      return this;
    }

    public Builder zHeightThreshold(double zHeightThresholdM) {
      requirePositive("zHeightThresholdM", zHeightThresholdM);
      this.zHeightThresholdM = zHeightThresholdM;
      return this;
    }

    public Builder linearStdDevCoeff(double linearStdDevCoeff) {
      requirePositive("linearStdDevCoeff", linearStdDevCoeff);
      this.linearStdDevCoeff = linearStdDevCoeff;
      return this;
    }

    public Builder angularStdDevCoeff(double angularStdDevCoeff) {
      requirePositive("angularStdDevCoeff", angularStdDevCoeff);
      this.angularStdDevCoeff = angularStdDevCoeff;
      return this;
    }

    public Builder fieldBounds(double maxX, double maxY) {
      requirePositive("fieldMaxX", maxX);
      requirePositive("fieldMaxY", maxY);
      this.fieldMaxX = maxX;
      this.fieldMaxY = maxY;
      return this;
    }

    public VisionConfig build() {
      return new VisionConfig(this);
    }

    private static void requirePositive(String name, double value) {
      if (value <= 0) {
        throw new IllegalArgumentException(name + " must be > 0, got: " + value);
      }
    }
  }
}
