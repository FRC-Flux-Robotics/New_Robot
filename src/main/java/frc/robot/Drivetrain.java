// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;

import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.ModuleConfig;
import frc.lib.drivetrain.PIDGains;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  private final double m_maxSpeed;
  private final double m_maxAngularSpeed;

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final Pigeon2 m_gyro;

  private final SwerveDriveKinematics m_kinematics;
  private final SwerveDriveOdometry m_odometry;

  public Drivetrain(DrivetrainConfig config) {
    m_maxSpeed = config.maxSpeedMps;
    m_maxAngularSpeed = config.maxAngularRateRadPerSec;

    m_frontLeft = createModule(config, config.frontLeft);
    m_frontRight = createModule(config, config.frontRight);
    m_backLeft = createModule(config, config.backLeft);
    m_backRight = createModule(config, config.backRight);

    m_gyro = new Pigeon2(config.pigeonId, config.canBusName);

    Translation2d flLocation = moduleLocation(config.frontLeft);
    Translation2d frLocation = moduleLocation(config.frontRight);
    Translation2d blLocation = moduleLocation(config.backLeft);
    Translation2d brLocation = moduleLocation(config.backRight);

    m_kinematics =
        new SwerveDriveKinematics(flLocation, frLocation, blLocation, brLocation);

    m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
            });

    m_gyro.reset();
  }

  private SwerveModule createModule(DrivetrainConfig config, ModuleConfig module) {
    return new SwerveModule(
        module,
        config.steerGains,
        config.driveGains,
        config.driveStatorCurrentLimit,
        config.driveSupplyCurrentLimit,
        config.steerStatorCurrentLimit,
        config.canBusName,
        config.driveGearRatio,
        config.steerGearRatio,
        config.wheelRadiusInches);
  }

  private Translation2d moduleLocation(ModuleConfig module) {
    return new Translation2d(
        Units.inchesToMeters(module.xPositionInches),
        Units.inchesToMeters(module.yPositionInches));
  }

  public double getMaxSpeed() {
    return m_maxSpeed;
  }

  public double getMaxAngularSpeed() {
    return m_maxAngularSpeed;
  }

  public SwerveModule[] getModules() {
    return new SwerveModule[] {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
  }

  public void applyCurrentLimits(double driveStator, double driveSupply, double steerStator) {
    for (SwerveModule module : getModules()) {
      module.applyCurrentLimits(driveStator, driveSupply, steerStator);
    }
  }

  public void applySteerPID(PIDGains gains) {
    for (SwerveModule module : getModules()) {
      module.applySteerPID(gains);
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
