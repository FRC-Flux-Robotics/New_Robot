package frc.lib.drivetrain;

public class PIDGains {
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kS;
  public final double kV;
  public final double kA;

  public PIDGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }

  public PIDGains(double kP, double kI, double kD) {
    this(kP, kI, kD, 0, 0, 0);
  }
}
