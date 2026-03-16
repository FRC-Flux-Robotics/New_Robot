package frc.lib.drivetrain;

public class DrivetrainConfig {
  // CAN
  public final String canBusName;
  public final int pigeonId;

  // Modules (FL, FR, BL, BR)
  public final ModuleConfig frontLeft;
  public final ModuleConfig frontRight;
  public final ModuleConfig backLeft;
  public final ModuleConfig backRight;

  // Mechanical
  public final double driveGearRatio;
  public final double steerGearRatio;
  public final double couplingRatio;
  public final double wheelRadiusInches;

  // Speed
  public final double maxSpeedMps;
  public final double maxAngularRateRadPerSec;

  // PID
  public final PIDGains steerGains;
  public final PIDGains driveGains;

  // Current limits (0 = disabled)
  public final double driveStatorCurrentLimit;
  public final double driveSupplyCurrentLimit;
  public final double steerStatorCurrentLimit;

  // Deadband
  public final double translationDeadband;
  public final double rotationDeadband;

  // Track dimensions
  public final double trackWidthInches;
  public final double trackLengthInches;

  private DrivetrainConfig(Builder builder) {
    this.canBusName = builder.canBusName;
    this.pigeonId = builder.pigeonId;
    this.frontLeft = builder.frontLeft;
    this.frontRight = builder.frontRight;
    this.backLeft = builder.backLeft;
    this.backRight = builder.backRight;
    this.driveGearRatio = builder.driveGearRatio;
    this.steerGearRatio = builder.steerGearRatio;
    this.couplingRatio = builder.couplingRatio;
    this.wheelRadiusInches = builder.wheelRadiusInches;
    this.maxSpeedMps = builder.maxSpeedMps;
    this.maxAngularRateRadPerSec = builder.maxAngularRateRadPerSec;
    this.steerGains = builder.steerGains;
    this.driveGains = builder.driveGains;
    this.driveStatorCurrentLimit = builder.driveStatorCurrentLimit;
    this.driveSupplyCurrentLimit = builder.driveSupplyCurrentLimit;
    this.steerStatorCurrentLimit = builder.steerStatorCurrentLimit;
    this.translationDeadband = builder.translationDeadband;
    this.rotationDeadband = builder.rotationDeadband;
    this.trackWidthInches = builder.trackWidthInches;
    this.trackLengthInches = builder.trackLengthInches;
  }

  public static class Builder {
    private String canBusName;
    private int pigeonId = -1;
    private ModuleConfig frontLeft;
    private ModuleConfig frontRight;
    private ModuleConfig backLeft;
    private ModuleConfig backRight;
    private double driveGearRatio;
    private double steerGearRatio;
    private double couplingRatio;
    private double wheelRadiusInches;
    private double maxSpeedMps;
    private double maxAngularRateRadPerSec;
    private PIDGains steerGains;
    private PIDGains driveGains;
    private double driveStatorCurrentLimit;
    private double driveSupplyCurrentLimit;
    private double steerStatorCurrentLimit;
    private double translationDeadband;
    private double rotationDeadband;
    private double trackWidthInches;
    private double trackLengthInches;

    public Builder canBusName(String canBusName) {
      this.canBusName = canBusName;
      return this;
    }

    public Builder pigeonId(int pigeonId) {
      this.pigeonId = pigeonId;
      return this;
    }

    public Builder frontLeft(ModuleConfig config) {
      this.frontLeft = config;
      return this;
    }

    public Builder frontRight(ModuleConfig config) {
      this.frontRight = config;
      return this;
    }

    public Builder backLeft(ModuleConfig config) {
      this.backLeft = config;
      return this;
    }

    public Builder backRight(ModuleConfig config) {
      this.backRight = config;
      return this;
    }

    public Builder driveGearRatio(double ratio) {
      this.driveGearRatio = ratio;
      return this;
    }

    public Builder steerGearRatio(double ratio) {
      this.steerGearRatio = ratio;
      return this;
    }

    public Builder couplingRatio(double ratio) {
      this.couplingRatio = ratio;
      return this;
    }

    public Builder wheelRadiusInches(double radius) {
      this.wheelRadiusInches = radius;
      return this;
    }

    public Builder maxSpeedMps(double speed) {
      this.maxSpeedMps = speed;
      return this;
    }

    public Builder maxAngularRateRadPerSec(double rate) {
      this.maxAngularRateRadPerSec = rate;
      return this;
    }

    public Builder steerGains(PIDGains gains) {
      this.steerGains = gains;
      return this;
    }

    public Builder driveGains(PIDGains gains) {
      this.driveGains = gains;
      return this;
    }

    public Builder driveStatorCurrentLimit(double limit) {
      this.driveStatorCurrentLimit = limit;
      return this;
    }

    public Builder driveSupplyCurrentLimit(double limit) {
      this.driveSupplyCurrentLimit = limit;
      return this;
    }

    public Builder steerStatorCurrentLimit(double limit) {
      this.steerStatorCurrentLimit = limit;
      return this;
    }

    public Builder translationDeadband(double deadband) {
      this.translationDeadband = deadband;
      return this;
    }

    public Builder rotationDeadband(double deadband) {
      this.rotationDeadband = deadband;
      return this;
    }

    public Builder trackWidthInches(double width) {
      this.trackWidthInches = width;
      return this;
    }

    public Builder trackLengthInches(double length) {
      this.trackLengthInches = length;
      return this;
    }

    public DrivetrainConfig build() {
      if (canBusName == null) {
        throw new IllegalStateException("canBusName is required");
      }
      if (pigeonId < 0) {
        throw new IllegalStateException("pigeonId is required");
      }
      if (frontLeft == null || frontRight == null || backLeft == null || backRight == null) {
        throw new IllegalStateException("All four module configs are required");
      }
      if (driveGearRatio <= 0) {
        throw new IllegalStateException("driveGearRatio must be positive");
      }
      if (steerGearRatio <= 0) {
        throw new IllegalStateException("steerGearRatio must be positive");
      }
      if (wheelRadiusInches <= 0) {
        throw new IllegalStateException("wheelRadiusInches must be positive");
      }
      if (maxSpeedMps <= 0) {
        throw new IllegalStateException("maxSpeedMps must be positive");
      }
      if (maxAngularRateRadPerSec <= 0) {
        throw new IllegalStateException("maxAngularRateRadPerSec must be positive");
      }
      if (steerGains == null) {
        throw new IllegalStateException("steerGains is required");
      }
      if (driveGains == null) {
        throw new IllegalStateException("driveGains is required");
      }
      if (trackWidthInches <= 0) {
        throw new IllegalStateException("trackWidthInches must be positive");
      }
      if (trackLengthInches <= 0) {
        throw new IllegalStateException("trackLengthInches must be positive");
      }
      return new DrivetrainConfig(this);
    }
  }
}
