package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivetrain.DriveInterface;

/** Simple P-controller command: rotate to align yaw, drive forward by area. */
public class DriveToTag extends Command {

  private static final double MIN_DRIVE_SPEED = 0.05;
  private static final double MAX_DRIVE_SPEED = 0.8;
  private static final double MIN_ROT_SPEED = 0.02;
  private static final double MAX_ROT_SPEED = 0.5;
  private static final double YAW_DRIVE_THRESHOLD = 15.0;

  private final Vision m_vision;
  private final DriveInterface m_drive;

  private double m_rotationP;
  private double m_driveP;
  private double m_angleTolerance;
  private double m_targetArea;
  private double m_areaTolerance;

  public DriveToTag(
      Vision vision,
      DriveInterface drive,
      double rotationP,
      double driveP,
      double angleTolerance,
      double targetArea,
      double areaTolerance) {
    m_vision = vision;
    m_drive = drive;
    m_rotationP = rotationP;
    m_driveP = driveP;
    m_angleTolerance = angleTolerance;
    m_targetArea = targetArea;
    m_areaTolerance = areaTolerance;
    addRequirements(vision, drive);
  }

  public DriveToTag(Vision vision, DriveInterface drive) {
    this(vision, drive, 0.07, 0.15, 2.0, 5.0, 0.5);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("DriveToTag/RotationP", m_rotationP);
    SmartDashboard.putNumber("DriveToTag/DriveP", m_driveP);
    SmartDashboard.putNumber("DriveToTag/AngleTolerance", m_angleTolerance);
    SmartDashboard.putNumber("DriveToTag/TargetArea", m_targetArea);
  }

  @Override
  public void execute() {
    // Read tunable gains from SmartDashboard
    m_rotationP = SmartDashboard.getNumber("DriveToTag/RotationP", m_rotationP);
    m_driveP = SmartDashboard.getNumber("DriveToTag/DriveP", m_driveP);
    m_angleTolerance = SmartDashboard.getNumber("DriveToTag/AngleTolerance", m_angleTolerance);
    m_targetArea = SmartDashboard.getNumber("DriveToTag/TargetArea", m_targetArea);

    if (!m_vision.hasTargets()) {
      m_drive.drive(0, 0, 0, false, 0.02);
      return;
    }

    double yaw = m_vision.getTargetYaw();
    double area = m_vision.getTargetArea();

    double rotSpeed = clampSpeed(-yaw * m_rotationP, MIN_ROT_SPEED, MAX_ROT_SPEED);
    double driveSpeed = 0;

    // Only drive forward when reasonably aligned
    if (Math.abs(yaw) < YAW_DRIVE_THRESHOLD) {
      driveSpeed = clampSpeed((m_targetArea - area) * m_driveP, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
    }

    m_drive.drive(driveSpeed, 0, rotSpeed, false, 0.02);
  }

  @Override
  public boolean isFinished() {
    if (!m_vision.hasTargets()) return false;
    return Math.abs(m_vision.getTargetYaw()) < m_angleTolerance
        && m_vision.getTargetArea() > m_targetArea - m_areaTolerance;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false, 0.02);
  }

  /**
   * Clamp speed with dead zone: if |value| < min, return 0; otherwise clamp magnitude to [min, max]
   * preserving sign.
   */
  static double clampSpeed(double value, double min, double max) {
    double abs = Math.abs(value);
    if (abs < min) return 0.0;
    if (abs > max) return Math.copySign(max, value);
    return value;
  }
}
