package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.drivetrain.DriveInterface;
import frc.lib.drivetrain.SwerveDrive;

/** Simple P-controller command: rotate to align yaw, drive forward by area. */
public class DriveToTag extends Command {

  private final Vision m_vision;
  private final DriveInterface m_drive;

  private final double m_rotationP;
  private final double m_driveP;
  private final double m_angleTolerance;
  private final double m_targetArea;

  public DriveToTag(
      Vision vision, SwerveDrive swerveDrive, DriveInterface drive,
      double rotationP, double driveP, double angleTolerance, double targetArea) {
    m_vision = vision;
    m_drive = drive;
    m_rotationP = rotationP;
    m_driveP = driveP;
    m_angleTolerance = angleTolerance;
    m_targetArea = targetArea;
    addRequirements(vision, swerveDrive);
  }

  public DriveToTag(Vision vision, SwerveDrive swerveDrive, DriveInterface drive) {
    this(vision, swerveDrive, drive, 0.07, 0.15, 2.0, 5.0);
  }

  @Override
  public void execute() {
    if (!m_vision.hasTargets()) {
      m_drive.drive(0, 0, 0, false, 0.02);
      return;
    }

    double yaw = m_vision.getTargetYaw();
    double area = m_vision.getTargetArea();

    double rotSpeed = -yaw * m_rotationP;
    double driveSpeed = 0;

    // Only drive forward when reasonably aligned
    if (Math.abs(yaw) < m_angleTolerance * 3) {
      driveSpeed = (m_targetArea - area) * m_driveP;
      driveSpeed = Math.max(-1.0, Math.min(1.0, driveSpeed));
    }

    m_drive.drive(driveSpeed, 0, rotSpeed, false, 0.02);
  }

  @Override
  public boolean isFinished() {
    if (!m_vision.hasTargets()) return false;
    return Math.abs(m_vision.getTargetYaw()) < m_angleTolerance
        && m_vision.getTargetArea() > m_targetArea;
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false, 0.02);
  }
}
