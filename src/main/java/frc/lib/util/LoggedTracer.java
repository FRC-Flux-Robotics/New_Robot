package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/** Lightweight per-section loop profiler that logs elapsed ms via AdvantageKit. */
public final class LoggedTracer {

  private static double lastTimestamp;

  private LoggedTracer() {}

  /** Call at the start of each periodic loop to begin timing. */
  public static void reset() {
    lastTimestamp = Timer.getFPGATimestamp();
  }

  /**
   * Record elapsed ms since last reset/trace under "Performance/{section}Ms".
   *
   * @param section name of the code section (e.g. "Commands", "Vision")
   */
  public static void trace(String section) {
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput("Performance/" + section + "Ms", (now - lastTimestamp) * 1000.0);
    lastTimestamp = now;
  }
}
