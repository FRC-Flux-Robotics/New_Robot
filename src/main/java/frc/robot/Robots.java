package frc.robot;

import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.ModuleConfig;
import frc.lib.drivetrain.PIDGains;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Robots {

  // Kraken x60 free speed: 96.7 rps, drive ratio: 6.3947, wheel radius: 2.0 in
  private static final double kFreeSpeedRps = 96.7;
  private static final double kDriveGearRatio = 6.394736842105262;
  private static final double kWheelRadiusInches = 2.0;
  private static final double kWheelRadiusMeters = Units.inchesToMeters(kWheelRadiusInches);
  private static final double kMaxSpeedMps =
      (kFreeSpeedRps / kDriveGearRatio) * (2.0 * Math.PI * kWheelRadiusMeters);

  // Track: 23.5 x 23.5 inches, half = 11.75
  private static final double kHalfTrack = 23.5 / 2.0;

  public static final DrivetrainConfig CORAL =
      new DrivetrainConfig.Builder()
          .canBusName("CANdace")
          .pigeonId(24)
          // FL: drive=7, steer=8, encoder=23
          .frontLeft(
              new ModuleConfig(7, 8, 23, 0.121337890625, kHalfTrack, kHalfTrack, false, false))
          // FR: drive=1, steer=2, encoder=20
          .frontRight(
              new ModuleConfig(1, 2, 20, -0.294921875, kHalfTrack, -kHalfTrack, true, false))
          // BL: drive=5, steer=6, encoder=22
          .backLeft(
              new ModuleConfig(5, 6, 22, 0.040771484375, -kHalfTrack, kHalfTrack, false, false))
          // BR: drive=3, steer=4, encoder=21
          .backRight(
              new ModuleConfig(3, 4, 21, -0.376953125, -kHalfTrack, -kHalfTrack, true, false))
          // Mechanical
          .driveGearRatio(kDriveGearRatio)
          .steerGearRatio(12.1)
          .couplingRatio(4.5)
          .wheelRadiusInches(kWheelRadiusInches)
          // Speed
          .maxSpeedMps(kMaxSpeedMps)
          .maxAngularRateRadPerSec(Math.PI)
          // PID - working values from tag 0.1
          .driveGains(new PIDGains(0.1, 0, 0, 0, 0.12, 0))
          .steerGains(new PIDGains(100, 0, 0.5, 0, 0, 0))
          // Current limits - working values from tag 0.1
          .driveStatorCurrentLimit(60)
          .driveSupplyCurrentLimit(120)
          .steerStatorCurrentLimit(0) // disabled
          // Deadband
          .translationDeadband(0.02)
          .rotationDeadband(0.02)
          // Track
          .trackWidthInches(23.5)
          .trackLengthInches(23.5)
          // Vision — OV9281 front camera
          .camera(new CameraConfig("OV9281", new Transform3d(
              new Translation3d(0.3, 0, 0.25), new Rotation3d(0, 0, 0))))
          .build();

  private Robots() {}
}
