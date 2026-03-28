package frc.lib.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotController;

/** Real hardware IO — batch-refreshes CAN signals from TalonFX motors, CANcoders, and Pigeon2. */
public class DrivetrainIOTalonFX implements DrivetrainIO {

  private final StatusSignal<Angle> m_gyroYaw;
  private final StatusSignal<AngularVelocity> m_gyroRate;
  private final StatusSignal<Current>[] m_driveCurrent;
  private final StatusSignal<Current>[] m_steerCurrent;
  private final StatusSignal<Angle>[] m_encoderPosition;
  private final BaseStatusSignal[] m_allSignals;

  @SuppressWarnings("unchecked")
  public DrivetrainIOTalonFX(SwerveDrive drivetrain) {
    m_gyroYaw = drivetrain.getPigeon2().getYaw();
    m_gyroRate = drivetrain.getPigeon2().getAngularVelocityZWorld();

    m_driveCurrent = new StatusSignal[4];
    m_steerCurrent = new StatusSignal[4];
    m_encoderPosition = new StatusSignal[4];

    for (int i = 0; i < 4; i++) {
      SwerveModule<TalonFX, TalonFX, CANcoder> module = drivetrain.getModule(i);
      m_driveCurrent[i] = module.getDriveMotor().getStatorCurrent();
      m_steerCurrent[i] = module.getSteerMotor().getStatorCurrent();
      m_encoderPosition[i] = module.getEncoder().getPosition();
    }

    // 2 gyro + 4×3 module = 14 signals
    m_allSignals = new BaseStatusSignal[14];
    m_allSignals[0] = m_gyroYaw;
    m_allSignals[1] = m_gyroRate;
    for (int i = 0; i < 4; i++) {
      m_allSignals[2 + i * 3] = m_driveCurrent[i];
      m_allSignals[3 + i * 3] = m_steerCurrent[i];
      m_allSignals[4 + i * 3] = m_encoderPosition[i];
    }
  }

  @Override
  public void updateInputs(DrivetrainIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(m_allSignals);

    inputs.gyroYawDeg = m_gyroYaw.getValueAsDouble();
    inputs.gyroRateDegPerSec = m_gyroRate.getValueAsDouble();
    inputs.batteryVoltage = RobotController.getBatteryVoltage();

    for (int i = 0; i < 4; i++) {
      inputs.driveCurrentA[i] = m_driveCurrent[i].getValueAsDouble();
      inputs.steerCurrentA[i] = m_steerCurrent[i].getValueAsDouble();
      inputs.moduleAngleDeg[i] = m_encoderPosition[i].getValueAsDouble() * 360.0;
    }
  }
}
