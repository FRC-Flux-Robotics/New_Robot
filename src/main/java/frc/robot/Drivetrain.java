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

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  // CAN bus name
  private static final String kCANbus = "CANdace";

  // Kraken x60 free speed: 96.7 rps, drive ratio: 6.3947, wheel radius: 2.0 in
  private static final double kWheelRadiusMeters = Units.inchesToMeters(2.0);
  private static final double kDriveGearRatio = 6.394736842105262;
  private static final double kFreeSpeedRps = 96.7;
  public static final double kMaxSpeed =
      (kFreeSpeedRps / kDriveGearRatio) * (2.0 * Math.PI * kWheelRadiusMeters);
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // Track size: 23.5 x 23.5 inches
  private static final double kHalfTrackWidthMeters = Units.inchesToMeters(23.5 / 2.0);
  private static final double kHalfTrackLengthMeters = Units.inchesToMeters(23.5 / 2.0);

  private final Translation2d m_frontLeftLocation =
      new Translation2d(kHalfTrackLengthMeters, kHalfTrackWidthMeters);
  private final Translation2d m_frontRightLocation =
      new Translation2d(kHalfTrackLengthMeters, -kHalfTrackWidthMeters);
  private final Translation2d m_backLeftLocation =
      new Translation2d(-kHalfTrackLengthMeters, kHalfTrackWidthMeters);
  private final Translation2d m_backRightLocation =
      new Translation2d(-kHalfTrackLengthMeters, -kHalfTrackWidthMeters);

  // Swerve modules: driveId, steerId, encoderId, canbus, encoderOffset, driveInverted, steerInverted
  private final SwerveModule m_frontLeft =
      new SwerveModule(7, 8, 23, kCANbus, 0.121337890625, false, false);
  private final SwerveModule m_frontRight =
      new SwerveModule(1, 2, 20, kCANbus, -0.294921875, true, false);
  private final SwerveModule m_backLeft =
      new SwerveModule(5, 6, 22, kCANbus, 0.040771484375, false, false);
  private final SwerveModule m_backRight =
      new SwerveModule(3, 4, 21, kCANbus, -0.376953125, true, false);

  // Pigeon 2 gyro (ID 24)
  private final Pigeon2 m_gyro = new Pigeon2(24, kCANbus);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
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
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
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
