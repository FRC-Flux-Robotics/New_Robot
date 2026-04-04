// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.DrivetrainIOReplay;
import frc.lib.drivetrain.SwerveDrive;
import frc.lib.util.LogFileManager;
import frc.lib.util.LoggedTracer;
import frc.lib.util.PhoenixSignals;
import frc.lib.vision.VisionIO;
import frc.lib.vision.VisionIOPhotonVision;
import frc.lib.vision.VisionIOReplay;
import frc.robot.util.Elastic;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  /** Robot operating mode. Change to REPLAY to replay log files. */
  enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  static final Mode mode = Robot.isReal() ? Mode.REAL : Mode.SIM;

  private final RobotContainer m_robotContainer;
  private Command m_autoCommand;

  /** Select robot config from RoboRIO comments string. Default: FUEL. */
  private static DrivetrainConfig selectRobot() {
    String comments = RobotController.getComments();
    if (comments.contains("CORAL")) {
      return Robots.CORAL;
    }
    return Robots.FUEL;
  }

  public Robot() {
    DrivetrainConfig config = selectRobot();

    Logger.recordMetadata("ProjectName", "FRC10413-2026");
    Logger.recordMetadata("RobotName", config == Robots.CORAL ? "CORAL" : "FUEL");

    switch (mode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter(LogFileManager.getLogPath()));
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    SwerveDrive swerveDrive =
        (mode != Mode.REPLAY)
            ? new SwerveDrive(config)
            : new SwerveDrive(config, new DrivetrainIOReplay());

    VisionIO[] visionIOs;
    if (mode != Mode.REPLAY) {
      AprilTagFieldLayout fieldLayout =
          AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
      visionIOs =
          config.cameras.stream()
              .map(cfg -> (VisionIO) new VisionIOPhotonVision(cfg, fieldLayout))
              .toArray(VisionIO[]::new);
    } else {
      visionIOs =
          config.cameras.stream()
              .map(cfg -> (VisionIO) new VisionIOReplay())
              .toArray(VisionIO[]::new);
    }

    CameraConfig[] cameras = config.cameras.toArray(CameraConfig[]::new);

    if (config == Robots.FUEL) {
      m_robotContainer = new FuelRobotContainer(swerveDrive, visionIOs, cameras);
    } else {
      m_robotContainer = new RobotContainer(swerveDrive, visionIOs, cameras);
    }

    String robotName = config == Robots.CORAL ? "CORAL" : "FUEL";
    SmartDashboard.putString("RobotName", robotName);
    Elastic.sendNotification(
        new Elastic.Notification(
            Elastic.NotificationLevel.INFO,
            "Robot Ready",
            robotName + " initialized with " + config.cameras.size() + " camera(s)"));
  }

  @Override
  public void robotPeriodic() {
    PhoenixSignals.refreshAll();
    LoggedTracer.reset();
    CommandScheduler.getInstance().run();
    LoggedTracer.trace("Commands");
    m_robotContainer.periodic();
    LoggedTracer.trace("Periodic");
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
