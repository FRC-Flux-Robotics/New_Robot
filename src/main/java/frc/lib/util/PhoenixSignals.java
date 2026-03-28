package frc.lib.util;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.*;

/**
 * Centralized CAN signal registry — collects all StatusSignals across subsystems and refreshes them
 * in a single {@code BaseStatusSignal.refreshAll()} call per bus, reducing CAN round-trips.
 *
 * <p>Usage: IO classes call {@link #register} in their constructor, then {@link #refreshAll} is
 * called once per robot loop (in Robot.robotPeriodic). IO classes check connectivity via {@link
 * #isOK}.
 */
public final class PhoenixSignals {
  private PhoenixSignals() {}

  private static int s_nextGroupId = 0;
  private static final Map<String, List<BaseStatusSignal>> s_busBuckets = new LinkedHashMap<>();
  private static final Map<Integer, GroupInfo> s_groups = new HashMap<>();
  private static final Map<String, BaseStatusSignal[]> s_cachedArrays = new HashMap<>();
  private static boolean s_dirty = true;

  private static final class GroupInfo {
    final BaseStatusSignal[] signals;
    boolean lastOk = false;

    GroupInfo(BaseStatusSignal[] signals) {
      this.signals = signals;
    }
  }

  /**
   * Registers signals for batch refresh. Call from IO constructors.
   *
   * @param busName CAN bus name ("" for RoboRIO bus, named string for CANivore)
   * @param signals the status signals to register
   * @return group ID for use with {@link #isOK}
   */
  public static int register(String busName, BaseStatusSignal... signals) {
    if (busName == null) {
      throw new IllegalArgumentException("busName must not be null");
    }
    if (signals == null || signals.length == 0) {
      throw new IllegalArgumentException("signals must not be null or empty");
    }

    int groupId = s_nextGroupId++;
    s_busBuckets.computeIfAbsent(busName, k -> new ArrayList<>()).addAll(Arrays.asList(signals));
    s_groups.put(groupId, new GroupInfo(signals.clone()));
    s_dirty = true;
    return groupId;
  }

  /**
   * Refreshes all registered signals — one CAN round-trip per bus. Call once per robot loop, before
   * subsystem periodics.
   */
  public static void refreshAll() {
    if (s_groups.isEmpty()) {
      return;
    }

    // Rebuild cached per-bus arrays if registrations changed
    if (s_dirty) {
      s_cachedArrays.clear();
      for (var entry : s_busBuckets.entrySet()) {
        List<BaseStatusSignal> signals = entry.getValue();
        s_cachedArrays.put(entry.getKey(), signals.toArray(new BaseStatusSignal[0]));
      }
      s_dirty = false;
    }

    // One refreshAll() per bus
    for (BaseStatusSignal[] busSignals : s_cachedArrays.values()) {
      BaseStatusSignal.refreshAll(busSignals);
    }

    // Update per-group connectivity status
    for (GroupInfo group : s_groups.values()) {
      boolean ok = true;
      for (BaseStatusSignal signal : group.signals) {
        if (!signal.getStatus().isOK()) {
          ok = false;
          break;
        }
      }
      group.lastOk = ok;
    }
  }

  /**
   * Returns whether all signals in the given group had OK status on the last refresh.
   *
   * @param groupId the group ID returned by {@link #register}
   * @return true if all signals in the group are healthy
   */
  public static boolean isOK(int groupId) {
    GroupInfo info = s_groups.get(groupId);
    return info != null && info.lastOk;
  }

  /** Returns the total number of registered signals across all buses. */
  public static int signalCount() {
    int count = 0;
    for (List<BaseStatusSignal> signals : s_busBuckets.values()) {
      count += signals.size();
    }
    return count;
  }

  /** Clears all registered signals and groups. For testing only. */
  public static void reset() {
    s_nextGroupId = 0;
    s_busBuckets.clear();
    s_groups.clear();
    s_cachedArrays.clear();
    s_dirty = true;
  }
}
