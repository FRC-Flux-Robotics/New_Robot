package frc.lib.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
    @AutoLog
    class DrivetrainIOInputs {
        // Gyro
        public double gyroYawDeg = 0;
        public double gyroRateDegPerSec = 0;

        // Odometry
        public double odometryPeriodSec = 0;

        // Per-module arrays [FL, FR, BL, BR]
        public double[] drivePositionRad = new double[4];
        public double[] driveVelocityRadPerSec = new double[4];
        public double[] steerAngleDeg = new double[4];
        public double[] driveTempC = new double[4];
        public double[] steerTempC = new double[4];
        public double[] driveCurrentA = new double[4];
    }

    void updateInputs(DrivetrainIOInputsAutoLogged inputs);
}
