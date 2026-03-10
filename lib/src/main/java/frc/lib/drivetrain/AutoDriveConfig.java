package frc.lib.drivetrain;

/**
 * Configuration for autonomous and teleop auto-drive controllers. Extracted from SwerveDrive's
 * hardcoded constants. Use {@link #builder()} to construct.
 */
public final class AutoDriveConfig {

  public final double headingKP;
  public final double autoTranslationKP;
  public final double autoTranslationKI;
  public final double autoTranslationKD;
  public final double autoRotationKP;
  public final double autoRotationKI;
  public final double autoRotationKD;
  public final double driveToPoseMaxVelMps;
  public final double driveToPoseMaxAccelMps2;
  public final double driveToPoseRotToleranceDeg;
  public final double pathMaxAccelMps2;
  public final double pathMaxAngularAccelRadPerSec2;

  private AutoDriveConfig(Builder b) {
    this.headingKP = b.headingKP;
    this.autoTranslationKP = b.autoTranslationKP;
    this.autoTranslationKI = b.autoTranslationKI;
    this.autoTranslationKD = b.autoTranslationKD;
    this.autoRotationKP = b.autoRotationKP;
    this.autoRotationKI = b.autoRotationKI;
    this.autoRotationKD = b.autoRotationKD;
    this.driveToPoseMaxVelMps = b.driveToPoseMaxVelMps;
    this.driveToPoseMaxAccelMps2 = b.driveToPoseMaxAccelMps2;
    this.driveToPoseRotToleranceDeg = b.driveToPoseRotToleranceDeg;
    this.pathMaxAccelMps2 = b.pathMaxAccelMps2;
    this.pathMaxAngularAccelRadPerSec2 = b.pathMaxAngularAccelRadPerSec2;
  }

  public static Builder builder() {
    return new Builder();
  }

  /** Returns a config with all default values (matching original SwerveDrive constants). */
  public static AutoDriveConfig defaults() {
    return builder().build();
  }

  public static final class Builder {
    private double headingKP = 5.0;
    private double autoTranslationKP = 5.0;
    private double autoTranslationKI = 0.0;
    private double autoTranslationKD = 0.0;
    private double autoRotationKP = 5.0;
    private double autoRotationKI = 0.0;
    private double autoRotationKD = 0.0;
    private double driveToPoseMaxVelMps = 2.0;
    private double driveToPoseMaxAccelMps2 = 2.0;
    private double driveToPoseRotToleranceDeg = 2.0;
    private double pathMaxAccelMps2 = 2.5;
    private double pathMaxAngularAccelRadPerSec2 = Math.PI;

    private Builder() {}

    public Builder headingKP(double headingKP) {
      requirePositive("headingKP", headingKP);
      this.headingKP = headingKP;
      return this;
    }

    public Builder autoTranslationPID(double kP, double kI, double kD) {
      requireNonNegative("autoTranslationKP", kP);
      this.autoTranslationKP = kP;
      this.autoTranslationKI = kI;
      this.autoTranslationKD = kD;
      return this;
    }

    public Builder autoRotationPID(double kP, double kI, double kD) {
      requireNonNegative("autoRotationKP", kP);
      this.autoRotationKP = kP;
      this.autoRotationKI = kI;
      this.autoRotationKD = kD;
      return this;
    }

    public Builder driveToPoseMaxVel(double maxVelMps) {
      requirePositive("driveToPoseMaxVelMps", maxVelMps);
      this.driveToPoseMaxVelMps = maxVelMps;
      return this;
    }

    public Builder driveToPoseMaxAccel(double maxAccelMps2) {
      requirePositive("driveToPoseMaxAccelMps2", maxAccelMps2);
      this.driveToPoseMaxAccelMps2 = maxAccelMps2;
      return this;
    }

    public Builder driveToPoseRotTolerance(double toleranceDeg) {
      requirePositive("driveToPoseRotToleranceDeg", toleranceDeg);
      this.driveToPoseRotToleranceDeg = toleranceDeg;
      return this;
    }

    public Builder pathMaxAccel(double maxAccelMps2) {
      requirePositive("pathMaxAccelMps2", maxAccelMps2);
      this.pathMaxAccelMps2 = maxAccelMps2;
      return this;
    }

    public Builder pathMaxAngularAccel(double maxAngularAccelRadPerSec2) {
      requirePositive("pathMaxAngularAccelRadPerSec2", maxAngularAccelRadPerSec2);
      this.pathMaxAngularAccelRadPerSec2 = maxAngularAccelRadPerSec2;
      return this;
    }

    public AutoDriveConfig build() {
      return new AutoDriveConfig(this);
    }

    private static void requirePositive(String name, double value) {
      if (value <= 0) {
        throw new IllegalArgumentException(name + " must be > 0, got: " + value);
      }
    }

    private static void requireNonNegative(String name, double value) {
      if (value < 0) {
        throw new IllegalArgumentException(name + " must be >= 0, got: " + value);
      }
    }
  }
}
