package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.drivetrain.DriveInterface;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Wraps PathPlanner commands with pre-flight validation and runtime monitoring.
 *
 * <p>Pre-flight (on initialize): checks starting position, path velocity limits, deadband. Runtime
 * (every cycle): monitors speed and path deviation, cancels if thresholds exceeded. All status
 * published to SmartDashboard under "Auto/" prefix with color-coded boolean indicators.
 */
public final class SafeAutoBuilder {

  /** Time to drive at zero velocity before path starts, letting modules align. */
  private static final double SETTLE_SECONDS = 0.25;

  private SafeAutoBuilder() {}

  /** Configurable safety thresholds. */
  public record Limits(
      double maxVelocityMps,
      double maxPositionErrorM,
      double maxStartErrorM,
      double speedOvershootFraction,
      double minEffectiveSpeedMps,
      double collisionDecelMps2,
      double collisionMinSpeedMps,
      int collisionCyclesRequired) {

    /** Sensible defaults for test-speed autos. */
    public static final Limits DEFAULT = new Limits(2.0, 1.0, 0.5, 0.20, 0.5, 8.0, 0.3, 10);

    /** Relaxed limits for competition (higher speeds allowed). */
    public static final Limits COMPETITION = new Limits(4.0, 2.0, 1.0, 0.30, 0.5, 10.0, 0.5, 5);
  }

  /** Wrap a PathPlannerPath with safety checks. Builds the follow-path command internally. */
  public static Command wrap(PathPlannerPath path, DriveInterface drive) {
    return wrap(AutoBuilder.followPath(path), path, drive, Limits.DEFAULT);
  }

  /** Wrap with custom limits. */
  public static Command wrap(PathPlannerPath path, DriveInterface drive, Limits limits) {
    return wrap(AutoBuilder.followPath(path), path, drive, limits);
  }

  /** Wrap an existing command (e.g. pathfindToPose) with a path's metadata. Path may be null. */
  public static Command wrap(Command innerCommand, PathPlannerPath path, DriveInterface drive) {
    return wrap(innerCommand, path, drive, Limits.DEFAULT);
  }

  /** Full-control overload. */
  public static Command wrap(
      Command innerCommand, PathPlannerPath path, DriveInterface drive, Limits limits) {
    // Pre-align swerve modules at zero speed before starting the path.
    // Prevents wiggle caused by modules snapping to new angles under velocity.
    Command settle =
        Commands.run(() -> drive.drive(0, 0, 0, true, 0.02), drive).withTimeout(SETTLE_SECONDS);
    return settle.andThen(new SafePathCommand(innerCommand, path, drive, limits));
  }

  /**
   * Check if a velocity is above the drivetrain's effective deadband. Returns true if the speed
   * will actually move the robot. Logs a warning if not.
   */
  public static boolean checkSpeedAboveDeadband(double speedMps, DriveInterface drive) {
    double deadband = drive.getMaxSpeed() * drive.getConfig().translationDeadband;
    if (speedMps < deadband) {
      warn(
          "Speed %.3f m/s is below deadband %.3f m/s (max=%.1f, db=%.0f%%). Robot will NOT move!",
          speedMps, deadband, drive.getMaxSpeed(), drive.getConfig().translationDeadband * 100);
      return false;
    }
    return true;
  }

  /** Initialize all dashboard keys to safe defaults. Call once from RobotContainer. */
  public static void initDashboard() {
    SmartDashboard.putString("Auto/Status", "Idle");
    SmartDashboard.putBoolean("Auto/PreflightPassed", false);
    SmartDashboard.putBoolean("Auto/Running", false);
    SmartDashboard.putBoolean("Auto/SpeedOK", true);
    SmartDashboard.putBoolean("Auto/OnPath", true);
    SmartDashboard.putString("Auto/CancelReason", "");
    SmartDashboard.putNumber("Auto/PathMaxVelMps", 0);
    SmartDashboard.putNumber("Auto/StartErrorM", 0);
    SmartDashboard.putNumber("Auto/CurrentSpeedMps", 0);
    SmartDashboard.putNumber("Auto/PathErrorM", 0);
    SmartDashboard.putBoolean("Auto/NoCollision", true);
    SmartDashboard.putNumber("Auto/DecelMps2", 0);
    SmartDashboard.putString("Auto/Waypoint", "");
  }

  private static void warn(String fmt, Object... args) {
    String msg = String.format(fmt, args);
    DriverStation.reportWarning("[SafeAuto] " + msg, false);
    Logger.recordOutput("Auto/Warning", msg);
  }

  /**
   * Command decorator that runs pre-flight checks in initialize() and monitors runtime safety in
   * execute(). Delegates lifecycle to the inner command.
   */
  private static final class SafePathCommand extends Command {
    private final Command m_inner;
    private final PathPlannerPath m_path;
    private final DriveInterface m_drive;
    private final Limits m_limits;

    private boolean m_cancelled;
    private String m_cancelReason;
    private double m_allowedSpeed;
    private boolean m_shouldFlip;
    private List<Pose2d> m_pathPoses;
    private List<Translation2d> m_waypointAnchors;
    private int m_nextWaypoint;
    private double m_prevSpeed;
    private long m_prevTimeNanos;
    private int m_collisionCycles;

    SafePathCommand(Command inner, PathPlannerPath path, DriveInterface drive, Limits limits) {
      m_inner = inner;
      m_path = path;
      m_drive = drive;
      m_limits = limits;
      addRequirements(inner.getRequirements().toArray(new Subsystem[0]));
    }

    @Override
    public void initialize() {
      m_cancelled = false;
      m_cancelReason = "";
      m_pathPoses = null;
      m_waypointAnchors = null;
      m_nextWaypoint = 1;

      // Use FieldPositions for consistent alliance detection (includes dashboard fallback)
      m_shouldFlip = FieldPositions.isRedAlliance();

      SmartDashboard.putString("Auto/Status", "Pre-flight...");
      SmartDashboard.putString("Auto/Waypoint", "");
      SmartDashboard.putBoolean("Auto/Running", true);
      SmartDashboard.putBoolean("Auto/SpeedOK", true);
      SmartDashboard.putBoolean("Auto/OnPath", true);
      SmartDashboard.putBoolean("Auto/NoCollision", true);
      SmartDashboard.putString("Auto/CancelReason", "");
      SmartDashboard.putNumber("Auto/DecelMps2", 0);
      m_prevSpeed = 0;
      m_prevTimeNanos = System.nanoTime();
      m_collisionCycles = 0;

      boolean allPassed = true;

      if (m_path != null) {
        PathConstraints constraints = m_path.getGlobalConstraints();
        double pathMaxVel = constraints.maxVelocityMPS();

        SmartDashboard.putNumber("Auto/PathMaxVelMps", pathMaxVel);
        Logger.recordOutput("Auto/PathMaxVelMps", pathMaxVel);

        // CHECK 1: Path velocity exceeds safe limit
        if (pathMaxVel > m_limits.maxVelocityMps) {
          warn(
              "Path max velocity %.1f m/s exceeds safe limit %.1f m/s!",
              pathMaxVel, m_limits.maxVelocityMps);
          allPassed = false;
        }

        // CHECK 2: Path velocity below effective minimum (deadband risk)
        if (pathMaxVel < m_limits.minEffectiveSpeedMps) {
          warn(
              "Path max velocity %.2f m/s below minimum effective speed %.2f m/s. May not move!",
              pathMaxVel, m_limits.minEffectiveSpeedMps);
          allPassed = false;
        }

        // CHECK 3: Robot position vs path starting position (alliance-aware)
        Optional<Pose2d> pathStart = m_path.getStartingHolonomicPose();
        if (pathStart.isPresent()) {
          Pose2d expectedStart =
              m_shouldFlip ? FlippingUtil.flipFieldPose(pathStart.get()) : pathStart.get();
          double startError =
              m_drive.getPose().getTranslation().getDistance(expectedStart.getTranslation());
          SmartDashboard.putNumber("Auto/StartErrorM", startError);
          Logger.recordOutput("Auto/StartErrorM", startError);

          if (startError > m_limits.maxStartErrorM) {
            warn(
                "Robot is %.2f m from path start (limit %.2f m). Expected: (%.2f, %.2f) Actual:"
                    + " (%.2f, %.2f)",
                startError,
                m_limits.maxStartErrorM,
                expectedStart.getX(),
                expectedStart.getY(),
                m_drive.getPose().getX(),
                m_drive.getPose().getY());
            allPassed = false;
            cancel("Not at start position (%.2f m away)", startError);
          }
        }

        // Cache path poses for runtime nearest-point check (alliance-aware)
        List<Pose2d> rawPoses = m_path.getPathPoses();
        if (m_shouldFlip) {
          m_pathPoses = new ArrayList<>(rawPoses.size());
          for (Pose2d p : rawPoses) {
            m_pathPoses.add(FlippingUtil.flipFieldPose(p));
          }
        } else {
          m_pathPoses = rawPoses;
        }

        // Cache waypoint anchors for progress tracking (alliance-aware)
        List<Waypoint> waypoints = m_path.getWaypoints();
        if (waypoints != null && waypoints.size() >= 2) {
          m_waypointAnchors = new ArrayList<>();
          for (Waypoint wp : waypoints) {
            Translation2d anchor =
                m_shouldFlip ? FlippingUtil.flipFieldPosition(wp.anchor()) : wp.anchor();
            m_waypointAnchors.add(anchor);
          }
          SmartDashboard.putString(
              "Auto/Waypoint", String.format("Start → WP 1/%d", m_waypointAnchors.size() - 1));
        }

        // Compute allowed speed for runtime monitoring
        m_allowedSpeed = pathMaxVel * (1.0 + m_limits.speedOvershootFraction);
      } else {
        // No path metadata — use limit as the allowed speed
        m_allowedSpeed = m_limits.maxVelocityMps * (1.0 + m_limits.speedOvershootFraction);
      }

      SmartDashboard.putBoolean("Auto/PreflightPassed", allPassed);

      if (m_cancelled) {
        SmartDashboard.putString("Auto/Status", "BLOCKED: " + m_cancelReason);
      } else {
        SmartDashboard.putString("Auto/Status", allPassed ? "Running" : "Running (warnings)");
        m_inner.initialize();
      }
    }

    @Override
    public void execute() {
      if (m_cancelled) return;

      m_inner.execute();

      // --- Runtime monitoring ---
      ChassisSpeeds speeds = m_drive.getVelocity();
      double currentSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      SmartDashboard.putNumber("Auto/CurrentSpeedMps", currentSpeed);
      Logger.recordOutput("Auto/CurrentSpeedMps", currentSpeed);

      // CHECK: Speed overshoot
      boolean speedOK = currentSpeed <= m_allowedSpeed;
      SmartDashboard.putBoolean("Auto/SpeedOK", speedOK);
      if (!speedOK) {
        cancel("Speed %.2f m/s exceeds allowed %.2f m/s", currentSpeed, m_allowedSpeed);
        return;
      }

      // CHECK: Collision detection — sustained deceleration while moving.
      // A ball pickup causes a 1-2 cycle spike; a wall hit sustains for many cycles.
      // Require consecutive cycles above threshold before cancelling.
      long nowNanos = System.nanoTime();
      double dtSeconds = (nowNanos - m_prevTimeNanos) / 1e9;
      if (dtSeconds > 0.005) { // skip if dt is too small (avoid division noise)
        double decel = (m_prevSpeed - currentSpeed) / dtSeconds;
        SmartDashboard.putNumber("Auto/DecelMps2", decel);
        Logger.recordOutput("Auto/DecelMps2", decel);

        boolean decelHigh =
            decel > m_limits.collisionDecelMps2 && m_prevSpeed > m_limits.collisionMinSpeedMps;
        if (decelHigh) {
          m_collisionCycles++;
        } else {
          m_collisionCycles = 0;
        }

        SmartDashboard.putBoolean(
            "Auto/NoCollision", m_collisionCycles < m_limits.collisionCyclesRequired);
        if (m_collisionCycles >= m_limits.collisionCyclesRequired) {
          cancel(
              "Collision detected! Decel %.1f m/s² sustained for %d cycles, was moving %.2f m/s",
              decel, m_collisionCycles, m_prevSpeed);
          m_prevSpeed = currentSpeed;
          m_prevTimeNanos = nowNanos;
          return;
        }
      }
      m_prevSpeed = currentSpeed;
      m_prevTimeNanos = nowNanos;

      // CHECK: Distance from nearest path pose (off-path detection)
      if (m_pathPoses != null && !m_pathPoses.isEmpty()) {
        double minDist = Double.MAX_VALUE;
        Pose2d currentPose = m_drive.getPose();
        for (Pose2d pathPose : m_pathPoses) {
          double dist = currentPose.getTranslation().getDistance(pathPose.getTranslation());
          if (dist < minDist) {
            minDist = dist;
          }
        }
        SmartDashboard.putNumber("Auto/PathErrorM", minDist);
        Logger.recordOutput("Auto/PathErrorM", minDist);

        boolean onPath = minDist <= m_limits.maxPositionErrorM;
        SmartDashboard.putBoolean("Auto/OnPath", onPath);
        if (!onPath) {
          cancel("Off path by %.2f m (limit %.2f m)", minDist, m_limits.maxPositionErrorM);
        }
      }

      // Waypoint progress tracking
      if (m_waypointAnchors != null && m_nextWaypoint < m_waypointAnchors.size()) {
        Translation2d robotPos = m_drive.getPose().getTranslation();
        Translation2d target = m_waypointAnchors.get(m_nextWaypoint);
        double distToNext = robotPos.getDistance(target);

        // Consider waypoint reached when within 0.3m
        if (distToNext < 0.3) {
          m_nextWaypoint++;
        }

        int totalSegments = m_waypointAnchors.size() - 1;
        if (m_nextWaypoint >= m_waypointAnchors.size()) {
          SmartDashboard.putString(
              "Auto/Waypoint", String.format("WP %d/%d (arriving)", totalSegments, totalSegments));
        } else {
          SmartDashboard.putString(
              "Auto/Waypoint",
              String.format(
                  "WP %d→%d/%d (%.1fm away)",
                  m_nextWaypoint, m_nextWaypoint + 1, totalSegments, distToNext));
        }
      }
    }

    @Override
    public boolean isFinished() {
      return m_cancelled || m_inner.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
      if (!m_cancelled) {
        m_inner.end(interrupted);
      }

      SmartDashboard.putBoolean("Auto/Running", false);

      if (m_cancelled) {
        SmartDashboard.putString("Auto/Status", "CANCELLED: " + m_cancelReason);
        Logger.recordOutput("Auto/Status", "CANCELLED: " + m_cancelReason);
        // Stop the robot
        m_drive.drive(0, 0, 0, true, 0.02);
      } else if (interrupted) {
        SmartDashboard.putString("Auto/Status", "Interrupted");
      } else {
        SmartDashboard.putString("Auto/Status", "Done");
      }
    }

    private void cancel(String fmt, Object... args) {
      m_cancelReason = String.format(fmt, args);
      m_cancelled = true;
      SmartDashboard.putString("Auto/CancelReason", m_cancelReason);
      warn("AUTO CANCELLED: %s", m_cancelReason);
    }
  }
}
