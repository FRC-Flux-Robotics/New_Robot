package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class CameraConfigTest {

  @Test
  void validConfigCreatesSuccessfully() {
    var transform = new Transform3d(new Translation3d(0.3, 0, 0.25), new Rotation3d());
    var config = new CameraConfig("TestCam", transform);
    assertEquals("TestCam", config.name());
    assertEquals(transform, config.robotToCamera());
  }

  @Test
  void nullNameThrows() {
    var transform = new Transform3d(new Translation3d(), new Rotation3d());
    assertThrows(IllegalArgumentException.class, () -> new CameraConfig(null, transform));
  }

  @Test
  void blankNameThrows() {
    var transform = new Transform3d(new Translation3d(), new Rotation3d());
    assertThrows(IllegalArgumentException.class, () -> new CameraConfig("  ", transform));
  }

  @Test
  void nullTransformThrows() {
    assertThrows(IllegalArgumentException.class, () -> new CameraConfig("Cam", null));
  }
}
