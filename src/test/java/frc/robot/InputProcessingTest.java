package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class InputProcessingTest {

    private static final double EPSILON = 1e-9;
    private static final double DEADBAND = 0.02;
    private static final double EXPO = 2.0;

    // --- applyInputCurve tests ---

    @Test
    void applyInputCurve_zeroReturnsZero() {
        assertEquals(0.0, InputProcessing.applyInputCurve(0.0, DEADBAND, EXPO), EPSILON);
    }

    @Test
    void applyInputCurve_fullInputReturnsOne() {
        assertEquals(1.0, InputProcessing.applyInputCurve(1.0, DEADBAND, EXPO), EPSILON);
        assertEquals(-1.0, InputProcessing.applyInputCurve(-1.0, DEADBAND, EXPO), EPSILON);
    }

    @Test
    void applyInputCurve_squaredCurveForNormalValues() {
        // 0.5 -> 0.25, sign preserved
        assertEquals(0.25, InputProcessing.applyInputCurve(0.5, DEADBAND, EXPO), EPSILON);
        assertEquals(-0.25, InputProcessing.applyInputCurve(-0.5, DEADBAND, EXPO), EPSILON);

        // 0.8 -> 0.64
        assertEquals(0.64, InputProcessing.applyInputCurve(0.8, DEADBAND, EXPO), EPSILON);
        assertEquals(-0.64, InputProcessing.applyInputCurve(-0.8, DEADBAND, EXPO), EPSILON);
    }

    @Test
    void applyInputCurve_belowDeadbandReturnsZero() {
        assertEquals(0.0, InputProcessing.applyInputCurve(0.005, DEADBAND, EXPO), EPSILON);
        assertEquals(0.0, InputProcessing.applyInputCurve(-0.005, DEADBAND, EXPO), EPSILON);
        assertEquals(0.0, InputProcessing.applyInputCurve(0.01, DEADBAND, EXPO), EPSILON);
        assertEquals(0.0, InputProcessing.applyInputCurve(-0.01, DEADBAND, EXPO), EPSILON);
    }

    @Test
    void applyInputCurve_atDeadbandReturnsZero() {
        assertEquals(0.0, InputProcessing.applyInputCurve(0.02, DEADBAND, EXPO), EPSILON);
        assertEquals(0.0, InputProcessing.applyInputCurve(-0.02, DEADBAND, EXPO), EPSILON);
    }

    @Test
    void applyInputCurve_aboveDeadbandReturnsSquared() {
        // 0.03 is above deadband, should return 0.03^2 = 0.0009
        assertEquals(0.0009, InputProcessing.applyInputCurve(0.03, DEADBAND, EXPO), EPSILON);
        assertEquals(-0.0009, InputProcessing.applyInputCurve(-0.03, DEADBAND, EXPO), EPSILON);
    }

    @Test
    void applyInputCurve_linearExpo() {
        // expo=1.0 should return raw value (above deadband)
        assertEquals(0.5, InputProcessing.applyInputCurve(0.5, DEADBAND, 1.0), EPSILON);
        assertEquals(-0.8, InputProcessing.applyInputCurve(-0.8, DEADBAND, 1.0), EPSILON);
    }

    @Test
    void applyInputCurve_cubicExpo() {
        // expo=3.0: 0.5 -> 0.125
        assertEquals(0.125, InputProcessing.applyInputCurve(0.5, DEADBAND, 3.0), EPSILON);
        assertEquals(-0.125, InputProcessing.applyInputCurve(-0.5, DEADBAND, 3.0), EPSILON);
    }

    // --- clampStickMagnitude tests ---

    @Test
    void clampStickMagnitude_diagonalClampedToUnitCircle() {
        double[] result = InputProcessing.clampStickMagnitude(1.0, 1.0);
        double mag = Math.hypot(result[0], result[1]);
        assertEquals(1.0, mag, EPSILON, "Diagonal input should be clamped to unit circle");
        assertEquals(result[0], result[1], EPSILON);
    }

    @Test
    void clampStickMagnitude_cardinalUnaffected() {
        double[] result = InputProcessing.clampStickMagnitude(1.0, 0.0);
        assertEquals(1.0, result[0], EPSILON);
        assertEquals(0.0, result[1], EPSILON);

        result = InputProcessing.clampStickMagnitude(0.0, 1.0);
        assertEquals(0.0, result[0], EPSILON);
        assertEquals(1.0, result[1], EPSILON);
    }

    @Test
    void clampStickMagnitude_smallInputsUnchanged() {
        double[] result = InputProcessing.clampStickMagnitude(0.5, 0.3);
        assertEquals(0.5, result[0], EPSILON);
        assertEquals(0.3, result[1], EPSILON);
    }
}
