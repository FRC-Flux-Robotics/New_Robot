package frc.lib.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/**
 * Clean interface for drivetrain consumers (other subsystems, autonomous routines).
 * Depend on this interface, not on the concrete SwerveDrive implementation.
 */
public interface DriveInterface {

    // --- Teleop driving ---

    /** Continuous field-centric driving. vx/vy in m/s, omega in rad/s. */
    Command driveFieldCentric(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega);

    /** Continuous robot-centric driving. vx/vy in m/s, omega in rad/s. */
    Command driveRobotCentric(DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega);

    /** Lock wheels in X pattern. Runs until interrupted. */
    Command brake();

    /** Zero all motor outputs. Runs until interrupted. */
    Command stop();

    // --- Simple point-to-point (PID, straight line) ---

    /**
     * Drive straight to a target pose using PID controllers on X, Y, rotation.
     * No obstacle avoidance. Good for short-range alignment (< 2m).
     * Finishes when within toleranceMeters of target.
     */
    Command driveToPose(Pose2d target, double toleranceMeters);

    // --- Path planning (PathPlanner, on-the-fly) ---

    /**
     * Generate a path to the target pose at runtime using PathPlanner pathfinding.
     * Handles obstacle avoidance if constraints are configured.
     * Use for long-range moves where straight-line isn't safe.
     *
     * <p>TODO: Change parameter type to PathPlannerPath when vendordep is added (Sprint 1).
     */
    Command pathfindToPose(Pose2d target);

    // --- Pre-computed paths ---

    /**
     * Follow a pre-built path (designed in PathPlanner GUI).
     * Use for autonomous routines with known waypoints.
     *
     * @param path PathPlannerPath object (passed as Object until vendordep is added in Sprint 1)
     */
    Command followPath(Object path);

    // --- State queries ---

    /** Current robot pose on the field (meters, radians). */
    Pose2d getPose();

    /** Current translational and rotational velocity. */
    ChassisSpeeds getVelocity();

    /** Current gyro heading. */
    Rotation2d getHeading();

    /** True if robot is moving above a small threshold. */
    boolean isMoving();

    // --- Pose management ---

    /** Reset pose estimate to a known position (e.g., at match start). */
    void resetPose(Pose2d pose);

    /** Reset heading to 0 (current facing = forward). */
    void resetHeading();

    /** Fuse a vision measurement into the pose estimator. */
    void addVisionMeasurement(Pose2d visionPose, double timestampSeconds);

    /** Access the active drivetrain configuration. */
    DrivetrainConfig getConfig();
}
