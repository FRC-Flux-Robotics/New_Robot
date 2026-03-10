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
        if (kD < 0) {
            throw new IllegalArgumentException("kD must be >= 0, got: " + kD);
        }
        if (kV < 0) {
            throw new IllegalArgumentException("kV must be >= 0, got: " + kV);
        }
        if (kA < 0) {
            throw new IllegalArgumentException("kA must be >= 0, got: " + kA);
        }
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    @Override
    public String toString() {
        return "PIDGains(kP=" + kP + ", kI=" + kI + ", kD=" + kD
                + ", kS=" + kS + ", kV=" + kV + ", kA=" + kA + ")";
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
