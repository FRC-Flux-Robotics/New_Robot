package frc.robot;

import frc.lib.drivetrain.PIDGains;
import frc.lib.mechanism.ControlMode;
import frc.lib.mechanism.MechanismConfig;

/**
 * Static MechanismConfig instances for FUEL robot mechanisms. Mirrors Robots.java pattern for
 * drivetrain configs.
 */
public final class MechanismConfigs {

  private static final String CAN_BUS = "Mech";

  // Shared current/voltage limits
  private static final double VELOCITY_PEAK_VOLTAGE = 11.0;
  private static final double POSITION_PEAK_VOLTAGE = 11.0;
  private static final double VELOCITY_CURRENT_LIMIT = 40.0;
  private static final double POSITION_CURRENT_LIMIT = 40.0;

  private static final PIDGains VELOCITY_PID = new PIDGains(0.11, 0.0, 0.0, 0.01, 0.12, 0.0);
  // Legacy VelocityMech2 working values: kP=0.5, kI=0, kD=0.0005, kS=0.01, kV=0.12
  private static final PIDGains SHOOTER_PID = new PIDGains(0.5, 0.0, 0.0005, 0.01, 0.12, 0.0);
  private static final PIDGains POSITION_PID = new PIDGains(2.4, 0.0, 0.1, 0.0, 0.0, 0.0);

  public static final MechanismConfig INTAKE =
      new MechanismConfig.Builder()
          .name("Intake")
          .motorId(1)
          // No second motor — legacy FollowerId=2 is marked "Absent" (no physical motor)
          .canBus(CAN_BUS)
          .pidGains(VELOCITY_PID)
          .controlMode(ControlMode.VELOCITY)
          .peakVoltage(VELOCITY_PEAK_VOLTAGE)
          .supplyCurrentLimit(VELOCITY_CURRENT_LIMIT)
          .build();

  public static final MechanismConfig TILT =
      new MechanismConfig.Builder()
          .name("Tilter")
          .motorId(20)
          .canBus(CAN_BUS)
          .pidGains(POSITION_PID)
          .controlMode(ControlMode.POSITION)
          .peakVoltage(POSITION_PEAK_VOLTAGE)
          .supplyCurrentLimit(POSITION_CURRENT_LIMIT)
          .jogStep(1.0)
          .softLimitForward(0.0)
          .softLimitReverse(-17.0)
          .build();

  // Rollers
  public static final MechanismConfig INDEXER =
      new MechanismConfig.Builder()
          .name("Indexer")
          .motorId(4)
          .canBus(CAN_BUS)
          .pidGains(VELOCITY_PID)
          .controlMode(ControlMode.VELOCITY)
          .peakVoltage(VELOCITY_PEAK_VOLTAGE)
          .supplyCurrentLimit(VELOCITY_CURRENT_LIMIT)
          .build();

  // Mechanum
  public static final MechanismConfig FEEDER =
      new MechanismConfig.Builder()
          .name("Feeder")
          .motorId(3)
          .canBus(CAN_BUS)
          .pidGains(VELOCITY_PID)
          .controlMode(ControlMode.VELOCITY)
          .peakVoltage(VELOCITY_PEAK_VOLTAGE)
          .supplyCurrentLimit(VELOCITY_CURRENT_LIMIT)
          .build();

  public static final MechanismConfig SHOOTER =
      new MechanismConfig.Builder()
          .name("Shooter")
          .motorId(11)
          .secondMotorId(10)
          .inverted(true)
          .counterRotating(true)
          .canBus(CAN_BUS)
          .pidGains(SHOOTER_PID)
          .controlMode(ControlMode.VELOCITY)
          .peakVoltage(VELOCITY_PEAK_VOLTAGE)
          .supplyCurrentLimit(VELOCITY_CURRENT_LIMIT)
          .build();

  public static final MechanismConfig HOOD =
      new MechanismConfig.Builder()
          .name("Hood")
          .motorId(12)
          .canBus(CAN_BUS)
          .pidGains(POSITION_PID)
          .controlMode(ControlMode.POSITION)
          .peakVoltage(POSITION_PEAK_VOLTAGE)
          .supplyCurrentLimit(POSITION_CURRENT_LIMIT)
          .jogStep(0.5)
          .softLimitForward(20.0)
          .softLimitReverse(0.0)
          .build();

  /** Intake operational constants. */
  public static final class IntakeConstants {
    public static final double InSpeed = 60;
    public static final double OutSpeed = 60;
  }

  /** Indexer/feeder operational constants. */
  public static final class IndexerConstants {
    public static final double Speed = 45;
    public static final double FeederSpeed = 40;
  }

  /** Shooter operational constants. */
  public static final class ShooterConstants {
    public static final double Speed = 2600.0 / 60.0;

    public static final int ShortRange = 0;
    public static final int MidRange = 1;
    public static final int LongRange = 2;
  }

  private MechanismConfigs() {}
}
