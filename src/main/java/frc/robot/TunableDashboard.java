package frc.robot;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.DrivetrainIOInputsAutoLogged;
import frc.lib.drivetrain.PIDGains;
import frc.lib.drivetrain.SwerveDrive;

public class TunableDashboard {
  private final SwerveDrive m_drivetrain;

  // Tunable entries
  private final DoubleEntry m_driveStatorLimit;
  private final DoubleEntry m_driveSupplyLimit;
  private final DoubleEntry m_steerStatorLimit;
  private final DoubleEntry m_steerKP;
  private final DoubleEntry m_steerKD;
  private final DoubleEntry m_steerKS;
  private final DoubleEntry m_steerKV;

  // Telemetry entries
  private final DoubleEntry m_batteryVoltage;
  private final DoubleEntry[] m_driveCurrent = new DoubleEntry[4];
  private final DoubleEntry[] m_steerCurrent = new DoubleEntry[4];
  private final DoubleEntry[] m_moduleAngle = new DoubleEntry[4];

  // Last-applied values for change detection
  private double m_lastDriveStator;
  private double m_lastDriveSupply;
  private double m_lastSteerStator;
  private double m_lastSteerKP;
  private double m_lastSteerKD;
  private double m_lastSteerKS;
  private double m_lastSteerKV;

  public TunableDashboard(SwerveDrive drivetrain, DrivetrainConfig config) {
    m_drivetrain = drivetrain;
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    // Tunables
    NetworkTable tunables = nt.getTable("Tunables");
    m_driveStatorLimit = tunables.getDoubleTopic("driveStatorLimit").getEntry(config.driveStatorCurrentLimit);
    m_driveSupplyLimit = tunables.getDoubleTopic("driveSupplyLimit").getEntry(config.driveSupplyCurrentLimit);
    m_steerStatorLimit = tunables.getDoubleTopic("steerStatorLimit").getEntry(config.steerStatorCurrentLimit);
    m_steerKP = tunables.getDoubleTopic("steerKP").getEntry(config.steerGains.kP);
    m_steerKD = tunables.getDoubleTopic("steerKD").getEntry(config.steerGains.kD);
    m_steerKS = tunables.getDoubleTopic("steerKS").getEntry(config.steerGains.kS);
    m_steerKV = tunables.getDoubleTopic("steerKV").getEntry(config.steerGains.kV);

    // Set initial values
    m_driveStatorLimit.set(config.driveStatorCurrentLimit);
    m_driveSupplyLimit.set(config.driveSupplyCurrentLimit);
    m_steerStatorLimit.set(config.steerStatorCurrentLimit);
    m_steerKP.set(config.steerGains.kP);
    m_steerKD.set(config.steerGains.kD);
    m_steerKS.set(config.steerGains.kS);
    m_steerKV.set(config.steerGains.kV);

    // Cache initial values
    m_lastDriveStator = config.driveStatorCurrentLimit;
    m_lastDriveSupply = config.driveSupplyCurrentLimit;
    m_lastSteerStator = config.steerStatorCurrentLimit;
    m_lastSteerKP = config.steerGains.kP;
    m_lastSteerKD = config.steerGains.kD;
    m_lastSteerKS = config.steerGains.kS;
    m_lastSteerKV = config.steerGains.kV;

    // Telemetry
    NetworkTable telemetry = nt.getTable("Telemetry");
    m_batteryVoltage = telemetry.getDoubleTopic("batteryVoltage").getEntry(0);

    String[] moduleNames = {"FL", "FR", "BL", "BR"};
    for (int i = 0; i < 4; i++) {
      NetworkTable moduleTable = telemetry.getSubTable(moduleNames[i]);
      m_driveCurrent[i] = moduleTable.getDoubleTopic("driveCurrent").getEntry(0);
      m_steerCurrent[i] = moduleTable.getDoubleTopic("steerCurrent").getEntry(0);
      m_moduleAngle[i] = moduleTable.getDoubleTopic("angle").getEntry(0);
    }
  }

  public void periodic() {
    // Read current tunable values
    double driveStator = m_driveStatorLimit.get(m_lastDriveStator);
    double driveSupply = m_driveSupplyLimit.get(m_lastDriveSupply);
    double steerStator = m_steerStatorLimit.get(m_lastSteerStator);
    double steerKP = m_steerKP.get(m_lastSteerKP);
    double steerKD = m_steerKD.get(m_lastSteerKD);
    double steerKS = m_steerKS.get(m_lastSteerKS);
    double steerKV = m_steerKV.get(m_lastSteerKV);

    // Check for current limit changes
    if (driveStator != m_lastDriveStator
        || driveSupply != m_lastDriveSupply
        || steerStator != m_lastSteerStator) {
      m_drivetrain.applyCurrentLimits(driveStator, driveSupply, steerStator);
      m_lastDriveStator = driveStator;
      m_lastDriveSupply = driveSupply;
      m_lastSteerStator = steerStator;
    }

    // Check for steer PID changes
    if (steerKP != m_lastSteerKP
        || steerKD != m_lastSteerKD
        || steerKS != m_lastSteerKS
        || steerKV != m_lastSteerKV) {
      m_drivetrain.applySteerPID(new PIDGains(steerKP, 0, steerKD, steerKS, steerKV, 0));
      m_lastSteerKP = steerKP;
      m_lastSteerKD = steerKD;
      m_lastSteerKS = steerKS;
      m_lastSteerKV = steerKV;
    }

    // Update telemetry from batch-refreshed IO inputs
    DrivetrainIOInputsAutoLogged inputs = m_drivetrain.getIOInputs();
    m_batteryVoltage.set(inputs.batteryVoltage);

    for (int i = 0; i < 4; i++) {
      m_driveCurrent[i].set(inputs.driveCurrentA[i]);
      m_steerCurrent[i].set(inputs.steerCurrentA[i]);
      m_moduleAngle[i].set(inputs.moduleAngleDeg[i]);
    }
  }
}
