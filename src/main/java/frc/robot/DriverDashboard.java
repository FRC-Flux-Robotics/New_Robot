package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivetrain.DriveState;
import frc.lib.drivetrain.DriveTelemetry;

/** Publishes driver-relevant drivetrain data to SmartDashboard. */
public class DriverDashboard implements DriveTelemetry {

  @Override
  public void update(DriveState state) {
    SmartDashboard.putNumber("Speed %", state.speedPercent());
    SmartDashboard.putNumber("Total Current A", state.totalCurrentA());
    SmartDashboard.putBoolean("Low Battery", state.brownoutActive());
    SmartDashboard.putBoolean("All Healthy", state.allHealthy());
    SmartDashboard.putString("Status", state.statusMessage());
  }
}
