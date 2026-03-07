package frc.lib.drivetrain;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/** Immutable PID + feedforward gains for a motor controller. */
public final class PIDGains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kS;
    public final double kV;
    public final double kA;

    public PIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        if (kP < 0) {
            throw new IllegalArgumentException("kP must be >= 0, got: " + kP);
        }
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /** Convert to CTRE Phoenix 6 Slot0Configs for motor controller configuration. */
    public Slot0Configs toSlot0Configs() {
        return new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    }
}
