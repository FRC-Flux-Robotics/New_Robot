package frc.lib.drivetrain;

/** Hardware abstraction for drivetrain sensor data. */
public interface DrivetrainIO {

  class Inputs {
    public double gyroYawDeg;
    public double gyroRateDegPerSec;
    public double batteryVoltage;
    public double[] driveCurrentA = new double[4];
    public double[] steerCurrentA = new double[4];
    public double[] moduleAngleDeg = new double[4];
  }

  /** Batch-read all sensor values into the inputs struct. */
  void updateInputs(Inputs inputs);
}
