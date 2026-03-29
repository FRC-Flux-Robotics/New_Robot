package frc.robot;

import edu.wpi.first.math.util.Units;

public class RangeTable {
  public class Range {
    public double distance;
    public double speed;
    public double elevation;

    public Range(double distance, double speed, double elevation) {
      this.distance = distance;
      this.speed = speed;
      this.elevation = elevation;
    }
  }
  ;

  // Elevation
  // 4.25 in = 20 => k = 4.7
  private final double hoodCoef = (9.0 - 0.0) / 2.0; // Position/inch

  // BUG FIX (S5-4): Robot length was in meters but added to inch values before
  // the inchesToMeters() conversion — mixing units in the sum. Fixed by using inches throughout.
  private static final double kRobotLengthInches = 27 + 2 * 3.5; // frame + bumpers

  private Range[] ranges = {
    new Range(
        Units.inchesToMeters(47.75 / 2 + kRobotLengthInches / 2 - 1 + 8), 2000.0 / 60.0, 0), // 0
    new Range(
        Units.inchesToMeters(47.75 / 2 + kRobotLengthInches / 2 - 1 + 36),
        2000.0 / 60.0,
        2.8), // 7/16
    new Range(
        Units.inchesToMeters(47.75 / 2 + kRobotLengthInches / 2 - 1 + 72),
        2200.0 / 60.0,
        3.9), // 1.25
    new Range(
        Units.inchesToMeters(47.75 / 2 + kRobotLengthInches / 2 - 1 + 108),
        2300.0 / 60.0,
        6.7), // 1.25
    new Range(
        Units.inchesToMeters(1.4142 * 47.75 / 2 + kRobotLengthInches / 2 - 1 + 134),
        2400.0 / 60.0,
        8.85) // 1.5
  };

  public RangeTable() {}

  public double getSpeedPreset(int preset) {
    if (preset > ranges.length) preset = ranges.length - 1;

    return preset >= 0 ? ranges[preset].speed : 0;
  }

  public double getElevationPreset(int preset) {
    if (preset > ranges.length) preset = ranges.length - 1;

    return preset >= 0 ? ranges[preset].elevation : 0;
  }

  public double lerp(double a, double b, double x) {
    return a * (1.0 - x) + x * b;
  }

  public Range getRange(double distance) {
    if (ranges.length == 0) return null;

    int i = 0;
    while (i < ranges.length && distance <= ranges[i].distance) ++i;

    if (i == 0) {
      return ranges[0];
    } else if (i >= ranges.length - 1) {
      return ranges[ranges.length - 1];
    }

    double d = distance - ranges[i].distance;
    double speed = lerp(ranges[i].speed, ranges[i + 1].speed, d);
    double elevation = lerp(ranges[i].elevation, ranges[i + 1].elevation, d);
    return new Range(distance, speed, elevation);
  }

  public Range getRangeStep(double distance) {
    Range range = null;
    for (int i = 0; i < ranges.length; ++i) {
      if (distance <= ranges[i].distance) {
        range = ranges[i];
        break;
      }
    }
    if (range == null && ranges.length > 0) {
      range = ranges[ranges.length - 1];
    }

    return range;
  }
}
