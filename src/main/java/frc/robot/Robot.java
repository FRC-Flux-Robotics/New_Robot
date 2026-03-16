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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.SwerveDrive;

public class Robot extends LoggedRobot {
  private final RobotContainer m_robotContainer;
  private Command m_autoCommand;

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

    DrivetrainConfig config = Robots.CORAL;
    SwerveDrive swerveDrive = new SwerveDrive(config);
    m_robotContainer = new RobotContainer(swerveDrive, config);
  }

  @Override
  public void robotPeriodic() {
    double loopStart = Timer.getFPGATimestamp();
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
    Logger.recordOutput("LoopTimeMs", (Timer.getFPGATimestamp() - loopStart) * 1000.0);
  }

  @Override
  public void autonomousInit() {
    m_autoCommand = m_robotContainer.getAutonomousCommand();
    if (m_autoCommand != null) {
      CommandScheduler.getInstance().schedule(m_autoCommand);
    }
  }

  @Override
  public void autonomousExit() {
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
      m_autoCommand = null;
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}
}
