package frc.robot;

public class PIDCtrl {
    public static double DefaultDifTime = 0.002;  //sec

    private double kP;
    private double kD;
    private double kI;
    private double dTime;

    private double error1;
    private double error2;

    public PIDCtrl(double kp, double kd, double ki, double dtime) {
        error1 = error2 = 0.0;
        kP = kp;
        kD = kd;
        kI = ki;
        dTime = dtime;
        if (dTime == 0)
            dTime = DefaultDifTime;
    }

    public void reset() {
        error1 = error2 = 0.0;
    }

    public void pid(double kp, double kd, double ki) {
        kP = kp;
        kD = kd;
        kI = ki;
    }

    public void delta(double dtime) {
        dTime = dtime;
        if (dTime == 0)
            dTime = DefaultDifTime;
    }

    public double calculateDif(double value, double target) {
        double err = target - value;
        double res = (kP + kI * dTime) * err - kP * error1 + (err + 2 * error1 + error2) * kD / dTime;
        error2 = error1;
        error1 = err;
        return res;
    }

    public double calculateDif(double value, double target, double dt) {
        double err = target - value;
        if (dt == 0)
            dt = dTime;
        double res = (kP + kI * dt) * err - kP * error1 + (err + 2 * error1 + error2) * kD / dt;
        error2 = error1;
        error1 = err;
        return res;
    }

    public double calculateDif2(double value, double target, double dt) {
        if (dt == 0)
            dt = dTime;
        double err = target - value;
        error2 += err * dt;  // error2 keeps integral part
        double res = kP * err + kI * error2 + kD * (err - error1) / dt;
        error1 = err;
        return res;
    }

    static public double limitRange(double x, double xMax) {
        if (xMax < 0)
            xMax = -xMax;
        if (x > xMax)
            x = xMax;
        if (x < -xMax)
            x = -xMax;
        return x;
    }

    static public double limitSignedRange(double x, double xMin, double xMax) {
        if (x > xMax)
            x = xMax;
        if (x < xMin)
            x = xMin;
        return x;
    }
}
