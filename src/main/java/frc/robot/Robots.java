package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.drivetrain.CameraConfig;
import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.ModuleConfig;
import frc.lib.drivetrain.PIDGains;

/**
 * Robot-specific drivetrain configurations for CORAL and FUEL.
 *
 * <h2>Shared Constants</h2>
 *
 * <table>
 *   <tr><th>Parameter</th><th>Value</th></tr>
 *   <tr><td>Motor</td><td>Kraken x60 (free speed 96.7 rps)</td></tr>
 *   <tr><td>Drive Gear Ratio</td><td>6.3947</td></tr>
 *   <tr><td>Steer Gear Ratio</td><td>12.1</td></tr>
 *   <tr><td>Coupling Ratio</td><td>4.5</td></tr>
 *   <tr><td>Wheel Radius</td><td>2.0 in</td></tr>
 *   <tr><td>Max Speed (12V)</td><td>~4.83 m/s</td></tr>
 *   <tr><td>Speed Coefficient</td><td>0.5 (teleop = ~2.42 m/s)</td></tr>
 *   <tr><td>Max Angular Rate</td><td>π rad/s (0.5 rot/s)</td></tr>
 *   <tr><td>Translation Deadband</td><td>0.02</td></tr>
 *   <tr><td>Rotation Deadband</td><td>0.02</td></tr>
 * </table>
 *
 * <h3>PID Gains</h3>
 *
 * <table>
 *   <tr><th>Controller</th><th>kP</th><th>kI</th><th>kD</th><th>kS</th><th>kV</th><th>kA</th></tr>
 *   <tr><td>Steer</td><td>15</td><td>0</td><td>0.9</td><td>0.1</td><td>1.5</td><td>0</td></tr>
 *   <tr><td>Drive</td><td>0.1</td><td>0</td><td>0</td><td>0</td><td>0.124</td><td>0</td></tr>
 * </table>
 *
 * <h3>Current Limits</h3>
 *
 * <table>
 *   <tr><th>Parameter</th><th>Value</th></tr>
 *   <tr><td>Drive Stator</td><td>60 A</td></tr>
 *   <tr><td>Drive Supply</td><td>120 A</td></tr>
 *   <tr><td>Steer Stator</td><td>60 A</td></tr>
 * </table>
 *
 * <h2>CORAL</h2>
 *
 * <table>
 *   <tr><th>Parameter</th><th>Value</th></tr>
 *   <tr><td>CAN Bus</td><td>"CANdace"</td></tr>
 *   <tr><td>Pigeon ID</td><td>24</td></tr>
 *   <tr><td>Track</td><td>23.5 × 23.5 in</td></tr>
 *   <tr><td>Camera</td><td>OV9281 at (0.3m, 0, 0.25m)</td></tr>
 * </table>
 *
 * <table>
 *   <tr><th>Module</th><th>Drive</th><th>Steer</th><th>Encoder</th><th>Offset</th><th>Inv Drive</th></tr>
 *   <tr><td>FL</td><td>7</td><td>8</td><td>23</td><td>+0.1213</td><td>no</td></tr>
 *   <tr><td>FR</td><td>1</td><td>2</td><td>20</td><td>-0.2949</td><td>yes</td></tr>
 *   <tr><td>BL</td><td>5</td><td>6</td><td>22</td><td>+0.0408</td><td>no</td></tr>
 *   <tr><td>BR</td><td>3</td><td>4</td><td>21</td><td>-0.3770</td><td>yes</td></tr>
 * </table>
 *
 * <h2>FUEL</h2>
 *
 * <table>
 *   <tr><th>Parameter</th><th>Value</th></tr>
 *   <tr><td>CAN Bus</td><td>"Drivetrain"</td></tr>
 *   <tr><td>Pigeon ID</td><td>20</td></tr>
 *   <tr><td>Track</td><td>22.25 × 22.25 in</td></tr>
 * </table>
 *
 * <table>
 *   <tr><th>Module</th><th>Drive</th><th>Steer</th><th>Encoder</th><th>Offset</th><th>Inv Drive</th></tr>
 *   <tr><td>FL</td><td>2</td><td>1</td><td>21</td><td>+0.1052</td><td>no</td></tr>
 *   <tr><td>FR</td><td>4</td><td>3</td><td>22</td><td>-0.1206</td><td>yes</td></tr>
 *   <tr><td>BL</td><td>6</td><td>5</td><td>23</td><td>-0.4668</td><td>no</td></tr>
 *   <tr><td>BR</td><td>8</td><td>7</td><td>24</td><td>-0.0371</td><td>yes</td></tr>
 * </table>
 *
 * <p>Vision cameras (FUEL):
 *
 * <table>
 *   <tr><th>Camera</th><th>Position (in)</th><th>Rotation (deg)</th></tr>
 *   <tr><td>OV9281-2 (center)</td><td>(-11.75, 0, 21.875)</td><td>(0, -10, 0)</td></tr>
 *   <tr><td>OV9281-1 (left)</td><td>(-1.875, 16.375, 20.875)</td><td>(0, 0, 15)</td></tr>
 *   <tr><td>OV9281-3 (right)</td><td>(-1.875, -16.375, 20.875)</td><td>(0, 0, -15)</td></tr>
 * </table>
 */
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
          // PID - matching working FluxRobot-2026 values
          .driveGains(new PIDGains(0.1, 0, 0, 0, 0.124, 0))
          .steerGains(new PIDGains(15, 0, 0.9, 0.1, 1.5, 0))
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
          // PID - matching working FluxRobot-2026 values
          .driveGains(new PIDGains(0.1, 0, 0, 0, 0.124, 0))
          .steerGains(new PIDGains(15, 0, 0.9, 0.1, 1.5, 0))
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
          // Vision — center camera (measured from FluxRobot-2026: X = -27/2 + 1.75 = -11.75")
          .camera(
              new CameraConfig(
                  "OV9281-2",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-11.75), 0, Units.inchesToMeters(21.875)),
                      new Rotation3d(0, Units.degreesToRadians(-10), 0))))
          // Vision — left camera (measured from FluxRobot-2026)
          .camera(
              new CameraConfig(
                  "OV9281-1",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-1.875),
                          Units.inchesToMeters(16.375),
                          Units.inchesToMeters(20.875)),
                      new Rotation3d(0, 0, Units.degreesToRadians(15)))))
          // Vision — right camera (measured from FluxRobot-2026)
          .camera(
              new CameraConfig(
                  "OV9281-3",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-1.875),
                          Units.inchesToMeters(-16.375),
                          Units.inchesToMeters(20.875)),
                      new Rotation3d(0, 0, Units.degreesToRadians(-15)))))
          .build();

  private Robots() {}
}
