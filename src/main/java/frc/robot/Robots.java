package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.ModuleConfig;
import frc.lib.drivetrain.PIDGains;

public final class Robots {

  // Kraken x60 free speed: 96.7 rps, drive ratio: 6.3947, wheel radius: 2.0 in
  private static final double kFreeSpeedRps = 96.7;
  private static final double kDriveGearRatio = 6.394736842105262;
  private static final double kWheelRadiusInches = 2.0;
  private static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);
  private static final double kMaxSpeedMps =
      (kFreeSpeedRps / kDriveGearRatio) * (2.0 * Math.PI * kWheelRadiusMeters);

  // CORAL track: 23.5 x 23.5 inches, half = 11.75
  private static final double kCoralHalfTrack = 23.5 / 2.0;

  // FUEL track: 22.25 x 22.25 inches, half = 11.125
  private static final double kFuelHalfTrack = 22.25 / 2.0;

  public static final DrivetrainConfig CORAL =
      new DrivetrainConfig.Builder()
          .canBusName("CANdace")
          .pigeonId(24)
          // FL: drive=7, steer=8, encoder=23
          .frontLeft(
              new ModuleConfig(
                  7, 8, 23, 0.121337890625, kCoralHalfTrack, kCoralHalfTrack, false, false))
          // FR: drive=1, steer=2, encoder=20
          .frontRight(
              new ModuleConfig(
                  1, 2, 20, -0.294921875, kCoralHalfTrack, -kCoralHalfTrack, true, false))
          // BL: drive=5, steer=6, encoder=22
          .backLeft(
              new ModuleConfig(
                  5, 6, 22, 0.040771484375, -kCoralHalfTrack, kCoralHalfTrack, false, false))
          // BR: drive=3, steer=4, encoder=21
          .backRight(
              new ModuleConfig(
                  3, 4, 21, -0.376953125, -kCoralHalfTrack, -kCoralHalfTrack, true, false))
          // Mechanical
          .driveGearRatio(kDriveGearRatio)
          .steerGearRatio(12.1)
          .couplingRatio(4.5)
          .wheelRadiusInches(kWheelRadiusInches)
          // Speed
          .maxSpeedMps(kMaxSpeedMps)
          .maxAngularRateRadPerSec(Math.PI)
          .speedCoefficient(0.5)
          // PID - tuned values
          .driveGains(new PIDGains(0.1, 0, 0, 0, 0.12, 0))
          .steerGains(new PIDGains(100, 0, 0.5, 0, 0, 0))
          // Current limits - tuned values
          .driveStatorCurrentLimit(60)
          .driveSupplyCurrentLimit(120)
          .steerStatorCurrentLimit(60)
          // Deadband
          .translationDeadband(0.02)
          .rotationDeadband(0.02)
          // Track
          .trackWidthInches(23.5)
          .trackLengthInches(23.5)
          // Vision — OV9281 front camera
          // TODO: Measure actual camera offset on robot (X=forward, Y=left, Z=up from center)
          .camera(
              new CameraConfig(
                  "OV9281",
                  new Transform3d(new Translation3d(0.3, 0, 0.25), new Rotation3d(0, 0, 0))))
          .build();

  public static final DrivetrainConfig FUEL =
      new DrivetrainConfig.Builder()
          .canBusName("Drivetrain")
          .pigeonId(20)
          // FL: drive=2, steer=1, encoder=21
          .frontLeft(
              new ModuleConfig(
                  2, 1, 21, 0.105224609375, kFuelHalfTrack, kFuelHalfTrack, false, false))
          // FR: drive=4, steer=3, encoder=22
          .frontRight(
              new ModuleConfig(
                  4, 3, 22, -0.12060546875, kFuelHalfTrack, -kFuelHalfTrack, true, false))
          // BL: drive=6, steer=5, encoder=23
          .backLeft(
              new ModuleConfig(
                  6, 5, 23, -0.466796875, -kFuelHalfTrack, kFuelHalfTrack, false, false))
          // BR: drive=8, steer=7, encoder=24
          .backRight(
              new ModuleConfig(
                  8, 7, 24, -0.037109375, -kFuelHalfTrack, -kFuelHalfTrack, true, false))
          // Mechanical
          .driveGearRatio(kDriveGearRatio)
          .steerGearRatio(12.1)
          .couplingRatio(4.5)
          .wheelRadiusInches(kWheelRadiusInches)
          // Speed
          .maxSpeedMps(kMaxSpeedMps)
          .maxAngularRateRadPerSec(Math.PI)
          .speedCoefficient(0.5)
          // PID - same tuned values as CORAL
          .driveGains(new PIDGains(0.1, 0, 0, 0, 0.12, 0))
          .steerGains(new PIDGains(100, 0, 0.5, 0, 0, 0))
          // Current limits - safety baseline
          .driveStatorCurrentLimit(60)
          .driveSupplyCurrentLimit(120)
          .steerStatorCurrentLimit(60)
          // Deadband
          .translationDeadband(0.02)
          .rotationDeadband(0.02)
          // Track
          .trackWidthInches(22.25)
          .trackLengthInches(22.25)
          // Vision — center camera
          .camera(
              new CameraConfig(
                  "OV9281-5",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-10.875), 0, Units.inchesToMeters(21.875)),
                      new Rotation3d(0, Units.degreesToRadians(-10), 0))))
          // Vision — left camera (TODO: measure actual transform on robot)
          .camera(
              new CameraConfig(
                  "OV9281-5-left",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-10.875),
                          Units.inchesToMeters(5.0),
                          Units.inchesToMeters(21.875)),
                      new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(30)))))
          // Vision — right camera (TODO: measure actual transform on robot)
          .camera(
              new CameraConfig(
                  "OV9281-5-right",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-10.875),
                          Units.inchesToMeters(-5.0),
                          Units.inchesToMeters(21.875)),
                      new Rotation3d(0, Units.degreesToRadians(-10), Units.degreesToRadians(-30)))))
          .build();

  private Robots() {}
}
