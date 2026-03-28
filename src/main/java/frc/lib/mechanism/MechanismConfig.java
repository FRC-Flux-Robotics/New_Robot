package frc.lib.mechanism;

import frc.lib.drivetrain.PIDGains;

/**
 * Immutable configuration for a mechanism subsystem. Use the {@link Builder} to construct.
 *
 * <p>Mirrors the DrivetrainConfig pattern: builder with fluent setters, validation in build(),
 * immutable public final fields.
 */
public class MechanismConfig {
  // Required
  public final String name;
  public final int motorId;
  public final String canBus;
  public final PIDGains pidGains;
  public final ControlMode controlMode;

  // Dual motor (optional)
  public final int secondMotorId;
  public final boolean counterRotating;

  // Current limits (0 = disabled)
  public final double statorCurrentLimit;
  public final double supplyCurrentLimit;

  // Voltage
  public final double peakVoltage;

  // Mechanical
  public final double gearRatio;
  public final boolean inverted;
  public final boolean neutralModeBrake;

  // Position-specific
  public final double jogStep;
  public final double softLimitForward;
  public final double softLimitReverse;
  public final double motionMagicCruiseVelocity;
  public final double motionMagicAcceleration;

  private MechanismConfig(Builder builder) {
    this.name = builder.name;
    this.motorId = builder.motorId;
    this.canBus = builder.canBus;
    this.pidGains = builder.pidGains;
    this.controlMode = builder.controlMode;
    this.secondMotorId = builder.secondMotorId;
    this.counterRotating = builder.counterRotating;
    this.statorCurrentLimit = builder.statorCurrentLimit;
    this.supplyCurrentLimit = builder.supplyCurrentLimit;
    this.peakVoltage = builder.peakVoltage;
    this.gearRatio = builder.gearRatio;
    this.inverted = builder.inverted;
    this.neutralModeBrake = builder.neutralModeBrake;
    this.jogStep = builder.jogStep;
    this.softLimitForward = builder.softLimitForward;
    this.softLimitReverse = builder.softLimitReverse;
    this.motionMagicCruiseVelocity = builder.motionMagicCruiseVelocity;
    this.motionMagicAcceleration = builder.motionMagicAcceleration;
  }

  public boolean isDualMotor() {
    return secondMotorId >= 0;
  }

  public boolean hasMotionMagic() {
    return motionMagicCruiseVelocity > 0;
  }

  public boolean hasSoftLimits() {
    return softLimitForward != Double.MAX_VALUE || softLimitReverse != -Double.MAX_VALUE;
  }

  public boolean hasStatorCurrentLimit() {
    return statorCurrentLimit > 0;
  }

  public boolean hasSupplyCurrentLimit() {
    return supplyCurrentLimit > 0;
  }

  public static class Builder {
    private String name;
    private int motorId = -1;
    private String canBus;
    private PIDGains pidGains;
    private ControlMode controlMode;
    private int secondMotorId = -1;
    private boolean counterRotating = false;
    private double statorCurrentLimit = 0;
    private double supplyCurrentLimit = 0;
    private double peakVoltage = 12.0;
    private double gearRatio = 1.0;
    private boolean inverted = false;
    private boolean neutralModeBrake = true;
    private double jogStep = 0;
    private double softLimitForward = Double.MAX_VALUE;
    private double softLimitReverse = -Double.MAX_VALUE;
    private double motionMagicCruiseVelocity = 0;
    private double motionMagicAcceleration = 0;

    public Builder name(String name) {
      this.name = name;
      return this;
    }

    public Builder motorId(int motorId) {
      this.motorId = motorId;
      return this;
    }

    public Builder canBus(String canBus) {
      this.canBus = canBus;
      return this;
    }

    public Builder pidGains(PIDGains pidGains) {
      this.pidGains = pidGains;
      return this;
    }

    public Builder controlMode(ControlMode controlMode) {
      this.controlMode = controlMode;
      return this;
    }

    public Builder secondMotorId(int secondMotorId) {
      this.secondMotorId = secondMotorId;
      return this;
    }

    public Builder counterRotating(boolean counterRotating) {
      this.counterRotating = counterRotating;
      return this;
    }

    public Builder statorCurrentLimit(double limit) {
      this.statorCurrentLimit = limit;
      return this;
    }

    public Builder supplyCurrentLimit(double limit) {
      this.supplyCurrentLimit = limit;
      return this;
    }

    public Builder peakVoltage(double voltage) {
      this.peakVoltage = voltage;
      return this;
    }

    public Builder gearRatio(double ratio) {
      this.gearRatio = ratio;
      return this;
    }

    public Builder inverted(boolean inverted) {
      this.inverted = inverted;
      return this;
    }

    public Builder neutralModeBrake(boolean brake) {
      this.neutralModeBrake = brake;
      return this;
    }

    public Builder jogStep(double step) {
      this.jogStep = step;
      return this;
    }

    public Builder softLimitForward(double limit) {
      this.softLimitForward = limit;
      return this;
    }

    public Builder softLimitReverse(double limit) {
      this.softLimitReverse = limit;
      return this;
    }

    public Builder motionMagicCruiseVelocity(double velocity) {
      this.motionMagicCruiseVelocity = velocity;
      return this;
    }

    public Builder motionMagicAcceleration(double acceleration) {
      this.motionMagicAcceleration = acceleration;
      return this;
    }

    public MechanismConfig build() {
      if (name == null || name.isEmpty()) {
        throw new IllegalStateException("name is required");
      }
      if (motorId < 0 || motorId > 62) {
        throw new IllegalStateException("motorId must be between 0 and 62");
      }
      if (canBus == null) {
        throw new IllegalStateException("canBus is required");
      }
      if (pidGains == null) {
        throw new IllegalStateException("pidGains is required");
      }
      if (controlMode == null) {
        throw new IllegalStateException("controlMode is required");
      }
      if (secondMotorId >= 0 && (secondMotorId > 62 || secondMotorId == motorId)) {
        throw new IllegalStateException(
            "secondMotorId must be between 0 and 62 and different from motorId");
      }
      if (gearRatio <= 0) {
        throw new IllegalStateException("gearRatio must be positive");
      }
      if (peakVoltage <= 0) {
        throw new IllegalStateException("peakVoltage must be positive");
      }
      if (statorCurrentLimit < 0) {
        throw new IllegalStateException("statorCurrentLimit must be >= 0");
      }
      if (supplyCurrentLimit < 0) {
        throw new IllegalStateException("supplyCurrentLimit must be >= 0");
      }
      if (hasSoftLimits() && softLimitForward <= softLimitReverse) {
        throw new IllegalStateException("softLimitForward must be greater than softLimitReverse");
      }
      return new MechanismConfig(this);
    }

    private boolean hasSoftLimits() {
      return softLimitForward != Double.MAX_VALUE || softLimitReverse != -Double.MAX_VALUE;
    }
  }
}
