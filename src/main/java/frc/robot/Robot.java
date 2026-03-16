// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.lib.drivetrain.DriveInterface;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.SwerveDrive;

public class Robot extends LoggedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final DrivetrainConfig m_config = Robots.CORAL;
  private final SwerveDrive m_swerveDrive;
  private final DriveInterface m_drive;
  private final TunableDashboard m_dashboard;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public Robot() {
    Logger.recordMetadata("ProjectName", "FRC10413-2026");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    Logger.start();

    m_swerveDrive = new SwerveDrive(m_config);
    m_drive = m_swerveDrive;
    m_dashboard = new TunableDashboard(m_swerveDrive, m_config);
  }

  @Override
  public void robotPeriodic() {
    double loopStart = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    m_dashboard.periodic();
    Logger.recordOutput("LoopTimeMs", (Timer.getFPGATimestamp() - loopStart) * 1000.0);
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    double speedScale = m_dashboard.getMaxSpeedScale();

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02))
            * m_drive.getMaxSpeed()
            * speedScale;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02))
            * m_drive.getMaxSpeed()
            * speedScale;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * m_drive.getMaxAngularSpeed();

    m_drive.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
