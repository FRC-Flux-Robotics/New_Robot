package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FieldPositionsTest {
  @BeforeAll
  static void initHAL() {
    HAL.initialize(500, 0);
    FieldPositions.init();
  }

  @BeforeEach
  void setBlue() {
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void resetDriverStation() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
  }

  @Test
  void redPoseUsesRotationalFieldSymmetry() {
    Pose2d red = FieldPositions.mirror(new Pose2d(2.0, 7.0, Rotation2d.fromDegrees(45)));
    assertEquals(14.54, red.getX(), 0.001);
    assertEquals(1.07, red.getY(), 0.001);
    assertEquals(0.0, red.getRotation().minus(Rotation2d.fromDegrees(-135)).getDegrees(), 0.001);
  }

  @Test
  void allRedStartPresetsAgreeWithPathPlanner() {
    // The side starts expose an error that a near-center HUB start can hide.
    for (String name : new String[] {"Left", "Right", "HUB"}) {
      Pose2d blue = FieldPositions.resolve(name);
      Pose2d expected = FlippingUtil.flipFieldPose(blue);
      assertEquals(expected, FieldPositions.mirror(blue), name);
    }
  }

  @Test
  void redPresetRemainsWithinDepotStartToleranceAfterBackup() {
    // Deployed side paths start at x=3.1; the robot backs up 11 inches first.
    for (String name : new String[] {"Left", "Right"}) {
      Pose2d blue = FieldPositions.resolve(name);
      Pose2d red = FieldPositions.mirror(blue);
      Translation2d afterBackup = red.getTranslation().plus(new Translation2d(0.2794, 0));
      Translation2d pathStart = FlippingUtil.flipFieldPosition(new Translation2d(3.1, blue.getY()));
      assertTrue(afterBackup.getDistance(pathStart) < 1.0, name);
    }
  }

  @Test
  void flipIsOwnInverse() {
    Pose2d original = new Pose2d(3.5, 4.1, Rotation2d.fromDegrees(37));
    Pose2d back = FieldPositions.mirror(FieldPositions.mirror(original));
    assertEquals(original.getX(), back.getX(), 0.001);
    assertEquals(original.getY(), back.getY(), 0.001);
    assertEquals(0.0, original.getRotation().minus(back.getRotation()).getDegrees(), 0.001);
  }

  @Test
  void redTranslationAndPoseUseTheSameTransform() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    Translation2d blue = new Translation2d(2, 6.55);
    assertEquals(FlippingUtil.flipFieldPosition(blue), FieldPositions.forAlliance(blue));
    assertEquals(
        FieldPositions.forAlliance(new Pose2d(blue, Rotation2d.kZero)).getTranslation(),
        FieldPositions.forAlliance(blue));
  }

  @Test
  void bluePoseAndTranslationRemainUnchanged() {
    Pose2d blue = new Pose2d(2, 6.55, Rotation2d.fromDegrees(30));
    assertEquals(blue, FieldPositions.forAlliance(blue));
    assertEquals(blue.getTranslation(), FieldPositions.forAlliance(blue.getTranslation()));
  }

  @Test
  void resolveOriginBlueAllianceIsZero() {
    assertEquals(Pose2d.kZero, FieldPositions.resolve("Origin"));
  }

  @Test
  void resolveUnknownReturnsNull() {
    assertNull(FieldPositions.resolve("NonExistent"));
  }
}
