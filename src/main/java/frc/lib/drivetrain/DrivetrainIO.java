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
        public double[] driveVelocityMps = new double[4];
        public double[] steerAngleDeg = new double[4];
        public double[] driveTempC = new double[4];
        public double[] steerTempC = new double[4];
        public double[] driveCurrentA = new double[4];
        public double[] steerCurrentA = new double[4];

        // Battery
        public double batteryVoltage = 12.0;

        // Vision (per-camera arrays, sized by camera count)
        public boolean[] visionConnected = new boolean[0];
        public boolean[] visionHasEstimate = new boolean[0];
        public double[] visionPoseX = new double[0];
        public double[] visionPoseY = new double[0];
        public double[] visionPoseRotDeg = new double[0];
        public double[] visionTimestampSec = new double[0];
        public int[] visionTagCount = new int[0];
        public double[] visionAmbiguity = new double[0];
    }

    void updateInputs(DrivetrainIOInputsAutoLogged inputs);
}
