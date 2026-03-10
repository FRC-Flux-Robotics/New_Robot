package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class VisionConfigTest {

  @Test
  void defaultsMatchExpectedValues() {
    VisionConfig config = VisionConfig.defaults();
    assertEquals(0.2, config.maxAmbiguity);
    assertEquals(4.0, config.maxSingleTagDistM);
    assertEquals(0.75, config.zHeightThresholdM);
    assertEquals(0.02, config.linearStdDevCoeff);
    assertEquals(0.06, config.angularStdDevCoeff);
    assertEquals(17.0, config.fieldMaxX);
    assertEquals(8.7, config.fieldMaxY);
  }

  @Test
  void builderOverridesDefaults() {
    VisionConfig config =
        VisionConfig.builder()
            .maxAmbiguity(0.3)
            .maxSingleTagDist(5.0)
            .zHeightThreshold(1.0)
            .linearStdDevCoeff(0.05)
            .angularStdDevCoeff(0.1)
            .fieldBounds(20.0, 10.0)
            .build();
    assertEquals(0.3, config.maxAmbiguity);
    assertEquals(5.0, config.maxSingleTagDistM);
    assertEquals(1.0, config.zHeightThresholdM);
    assertEquals(0.05, config.linearStdDevCoeff);
    assertEquals(0.1, config.angularStdDevCoeff);
    assertEquals(20.0, config.fieldMaxX);
    assertEquals(10.0, config.fieldMaxY);
  }

  @Test
  void rejectsNonPositiveAmbiguity() {
    assertThrows(IllegalArgumentException.class, () -> VisionConfig.builder().maxAmbiguity(0));
    assertThrows(IllegalArgumentException.class, () -> VisionConfig.builder().maxAmbiguity(-1));
  }

  @Test
  void rejectsNonPositiveSingleTagDist() {
    assertThrows(IllegalArgumentException.class, () -> VisionConfig.builder().maxSingleTagDist(0));
  }

  @Test
  void rejectsNonPositiveZHeightThreshold() {
    assertThrows(IllegalArgumentException.class, () -> VisionConfig.builder().zHeightThreshold(0));
    assertThrows(IllegalArgumentException.class, () -> VisionConfig.builder().zHeightThreshold(-1));
  }

  @Test
  void rejectsNonPositiveStdDevCoeffs() {
    assertThrows(IllegalArgumentException.class, () -> VisionConfig.builder().linearStdDevCoeff(0));
    assertThrows(
        IllegalArgumentException.class, () -> VisionConfig.builder().angularStdDevCoeff(0));
  }

  @Test
  void rejectsNonPositiveFieldBounds() {
    assertThrows(IllegalArgumentException.class, () -> VisionConfig.builder().fieldBounds(0, 8.7));
    assertThrows(IllegalArgumentException.class, () -> VisionConfig.builder().fieldBounds(17.0, 0));
  }
}
