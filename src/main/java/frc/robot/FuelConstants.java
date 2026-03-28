package frc.robot;

import edu.wpi.first.math.util.Units;

/** Constants for FUEL robot mechanisms. Ported from FluxRobot-2026. */
public final class FuelConstants {
  public static final int Forward = 0;
  public static final int Backward = 1;

  public static final int TimePeriodMsec = 20;
  public static final double TimePeriod = 0.001 * TimePeriodMsec;

  public static final double MaxMotorRPM = 6000.0;
  public static final double MaxMotorRPS = 0.5 * MaxMotorRPM / 60.0;
  public static final double MaxMotorRPStoSet = MaxMotorRPM / 60.0;

  public static final double VelocityPeakVoltage = 11.0;
  public static final double PositionPeakVoltage = 11.0;
  public static final double VelocityCurrentLimit = 40.0;
  public static final double PositionCurrentLimit = 40.0;

  public static final double RobotLengthMeters = Units.inchesToMeters(27 + 2 * 3.5); // frame + bumpers

  /** CAN bus name for FUEL mechanisms (default RIO bus). */
  public static final String MECHANISM_CAN_BUS = "";

  public static final class IntakeConstants {
    public static final int MotorId = 1;
    public static final int FollowerId = 2;
    public static final int TiltMotorId = 20;

    public static final double InSpeed = 60;
    public static final double OutSpeed = 60;

    public static final double TiltStep = 1;
    public static final double InTiltPosition = 0;
    public static final double OutTiltPosition = -17;

    public static final double MaxMotorRPS = 0.5 * MaxMotorRPM / 60.0;
  }

  public static final class IndexerConstants {
    public static final int IndexerId = 4;
    public static final int FeederId = 3;

    public static final double Speed = 45;
    public static final double FeederSpeed = 40;
  }

  public static final class ShooterConstants {
    public static final int LeftMotorId = 10;
    public static final int RightMotorId = 11;
    public static final int HoodMotorId = 12;

    public static final double Speed = 2600.0 / 60.0;
    public static final double SpeedStep = 100.0 / 60.0;

    public static final int ShortRange = 0;
    public static final int MidRange = 1;
    public static final int LongRange = 2;

    public static final double HoodStep = 0.5;
    public static final double MinElevation = 0;
    public static final double MaxElevation = 20;

    public static final double RangePositionTolerance = Units.inchesToMeters(6);
    public static final double MaxMotorRPS = 1.0 * MaxMotorRPM / 60.0;
  }

  private FuelConstants() {}
}
