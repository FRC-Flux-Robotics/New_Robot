package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.drivetrain.DriveInterface;
import frc.lib.drivetrain.DriveState;
import frc.lib.drivetrain.DrivetrainConfig;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SafeAutoBuilderTest {
  private StubDrive drive;
  private TrackingCommand inner;

  @BeforeAll
  static void initHAL() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setup() {
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    SafeAutoBuilder.initDashboard();
    drive = new StubDrive();
    inner = new TrackingCommand(drive);
  }

  @AfterEach
  void cleanup() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  private void startPath(Command wrapper) {
    wrapper.initialize();
    SimHooks.stepTiming(0.3);
    wrapper.execute(); // Finish the settle command and initialize the wrapped path.
  }

  @Test
  void runtimeSafetyStopInterruptsInnerCommandAndStopsDrive() {
    Command wrapper = SafeAutoBuilder.wrap(inner, null, drive);
    startPath(wrapper);
    drive.velocity = new ChassisSpeeds(6, 0, 0); // Above 4.0 * 1.30 limit.
    wrapper.execute();
    assertTrue(wrapper.isFinished());
    wrapper.end(false);
    assertEquals(List.of(true), inner.ends);
    assertFalse(inner.eventActive, "Path event cleanup must run after a safety stop");
    assertEquals(0.0, drive.lastX);
  }

  @Test
  void sameWrapperCanRunAgainAfterSafetyStop() {
    Command wrapper = SafeAutoBuilder.wrap(inner, null, drive);
    startPath(wrapper);
    drive.velocity = new ChassisSpeeds(6, 0, 0);
    wrapper.execute();
    wrapper.end(false);
    drive.velocity = new ChassisSpeeds();
    startPath(wrapper);
    wrapper.execute();
    assertFalse(wrapper.isFinished());
    assertEquals(2, inner.starts);
    assertEquals("", SmartDashboard.getString("Auto/CancelReason", "missing"));
    inner.finished = true;
    wrapper.execute();
    wrapper.end(false);
    assertEquals(List.of(true, false), inner.ends);
    assertFalse(inner.eventActive);
  }

  @Test
  void sameWrapperCanFinishNormallyTwice() {
    Command wrapper = SafeAutoBuilder.wrap(inner, null, drive);
    for (int i = 0; i < 2; i++) {
      startPath(wrapper);
      assertFalse(wrapper.isFinished());
      inner.finished = true;
      wrapper.execute();
      assertTrue(wrapper.isFinished());
      wrapper.end(false);
    }
    assertEquals(2, inner.starts);
    assertEquals(List.of(false, false), inner.ends);
  }

  @Test
  void externalInterruptionCleansUpAndStopsDrive() {
    Command wrapper = SafeAutoBuilder.wrap(inner, null, drive);
    startPath(wrapper);
    wrapper.execute();
    assertEquals(1.0, drive.lastX);
    wrapper.end(true);
    assertEquals(List.of(true), inner.ends);
    assertEquals(0.0, drive.lastX);
  }

  @Test
  void rejectedStartDoesNotEndAnUninitializedCommand() {
    drive.pose = new Pose2d(5, 0, Rotation2d.kZero);
    Command wrapper = SafeAutoBuilder.wrap(inner, path(), drive);
    startPath(wrapper);
    wrapper.execute();
    assertTrue(wrapper.isFinished());
    wrapper.end(false);
    assertEquals(0, inner.starts);
    assertTrue(inner.ends.isEmpty());
    assertEquals(0.0, drive.lastX);
    assertTrue(SmartDashboard.getString("Auto/CancelReason", "").contains("Not at start"));
  }

  @Test
  void blockedWrapperCanRunAfterPositionIsCorrected() {
    drive.pose = new Pose2d(5, 0, Rotation2d.kZero);
    Command wrapper = SafeAutoBuilder.wrap(inner, path(), drive);
    startPath(wrapper);
    wrapper.execute();
    wrapper.end(false);
    drive.pose = Pose2d.kZero;
    startPath(wrapper);
    wrapper.execute();
    assertEquals(1, inner.starts);
    assertFalse(wrapper.isFinished());
    wrapper.end(true);
    assertEquals(List.of(true), inner.ends);
  }

  @Test
  void interruptionDuringSettleDoesNotTouchInnerCommand() {
    Command wrapper = SafeAutoBuilder.wrap(inner, null, drive);
    wrapper.initialize();
    wrapper.end(true);
    assertEquals(0, inner.starts);
    assertTrue(inner.ends.isEmpty());
  }

  private static PathPlannerPath path() {
    return new PathPlannerPath(
        PathPlannerPath.waypointsFromPoses(Pose2d.kZero, new Pose2d(2, 0, Rotation2d.kZero)),
        new PathConstraints(1, 1, 1, 1),
        new IdealStartingState(0, Rotation2d.kZero),
        new GoalEndState(0, Rotation2d.kZero));
  }

  private static class TrackingCommand extends Command {
    private final StubDrive drive;
    int starts;
    boolean finished;
    boolean eventActive;
    final List<Boolean> ends = new ArrayList<>();

    TrackingCommand(StubDrive drive) {
      this.drive = drive;
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      starts++;
      finished = false;
      eventActive = true;
    }

    @Override
    public void execute() {
      drive.drive(1, 0, 0, false, 0.02);
    }

    @Override
    public boolean isFinished() {
      return finished;
    }

    @Override
    public void end(boolean interrupted) {
      ends.add(interrupted);
      eventActive = false;
    }
  }

  private static class StubDrive implements DriveInterface {
    Pose2d pose = Pose2d.kZero;
    ChassisSpeeds velocity = new ChassisSpeeds();
    double lastX;

    @Override
    public void drive(double x, double y, double rot, boolean fieldRelative, double period) {
      lastX = x;
    }

    @Override
    public Pose2d getPose() {
      return pose;
    }

    @Override
    public ChassisSpeeds getVelocity() {
      return velocity;
    }

    @Override
    public Rotation2d getHeading() {
      return pose.getRotation();
    }

    @Override
    public DriveState getDriveState() {
      return new DriveState(pose, velocity, getHeading());
    }

    @Override
    public void updateOdometry() {}

    @Override
    public void resetHeading() {}

    @Override
    public void resetPose(Pose2d newPose) {
      pose = newPose;
    }

    @Override
    public void setOperatorForward(Rotation2d forward) {}

    @Override
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}

    @Override
    public DrivetrainConfig getConfig() {
      return null;
    }

    @Override
    public double getMaxSpeed() {
      return 4;
    }

    @Override
    public double getMaxAngularSpeed() {
      return Math.PI;
    }
  }
}
