package frc.lib.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.junit.jupiter.api.Test;

class PIDGainsTest {

    @Test
    void constructsWithValidGains() {
        PIDGains gains = new PIDGains(100, 0, 0.5, 0.1, 1.5, 0);
        assertEquals(100, gains.kP);
        assertEquals(0, gains.kI);
        assertEquals(0.5, gains.kD);
        assertEquals(0.1, gains.kS);
        assertEquals(1.5, gains.kV);
        assertEquals(0, gains.kA);
    }

    @Test
    void allowsZeroKP() {
        PIDGains gains = new PIDGains(0, 0, 0, 0, 0, 0);
        assertEquals(0, gains.kP);
    }

    @Test
    void rejectsNegativeKP() {
        assertThrows(IllegalArgumentException.class, () -> new PIDGains(-1, 0, 0, 0, 0, 0));
    }

    @Test
    void convertsToSlot0Configs() {
        PIDGains gains = new PIDGains(100, 0.5, 0.2, 0.1, 1.5, 0.01);
        Slot0Configs slot0 = gains.toSlot0Configs();
        assertEquals(100, slot0.kP);
        assertEquals(0.5, slot0.kI);
        assertEquals(0.2, slot0.kD);
        assertEquals(0.1, slot0.kS);
        assertEquals(1.5, slot0.kV);
        assertEquals(0.01, slot0.kA);
    }
}
