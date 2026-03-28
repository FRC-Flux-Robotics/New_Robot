package frc.lib.mechanism;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for mechanism motor data. One instance per mechanism. */
public interface MechanismIO {

  @AutoLog
  class MechanismIOInputs {
    public double positionRotations;
    public double velocityRPS;
    public double statorCurrentA;
    public double supplyCurrentA;
    public double appliedVoltage;
    public double tempCelsius;
    public boolean motorConnected = false;
  }

  /** Batch-read all motor data into the inputs struct. */
  void updateInputs(MechanismIOInputsAutoLogged inputs);

  /** Set closed-loop position target in rotations. */
  default void setPosition(double positionRotations) {}

  /** Set closed-loop position target using MotionMagic in rotations. */
  default void setMotionMagicPosition(double positionRotations) {}

  /** Set closed-loop velocity target in rotations per second. */
  default void setVelocity(double velocityRPS) {}

  /** Command motor to neutral output. */
  default void stop() {}

  /** Reset encoder position to zero. */
  default void resetPosition() {}
}
