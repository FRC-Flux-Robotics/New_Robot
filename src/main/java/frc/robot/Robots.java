package frc.robot;

import frc.lib.drivetrain.DrivetrainConfig;
import frc.lib.drivetrain.ModuleConfig;
import frc.lib.drivetrain.PIDGains;

/**
 * Static DrivetrainConfig instances for each robot.
 * Values sourced from CTRE Tuner X measurements (PID, gearing) and mechanical specs (CAN IDs, positions).
 */
public final class Robots {

    public static final DrivetrainConfig CORAL = DrivetrainConfig.builder()
            .canBus("CANdace")
            .pigeonId(24)
            .frontLeft(new ModuleConfig(7, 8, 23, 0.124267578125, 0.2921, 0.2921, false, false, false))
            .frontRight(new ModuleConfig(1, 2, 20, -0.291015625, 0.2921, -0.2921, true, false, false))
            .backLeft(new ModuleConfig(5, 6, 22, 0.048828125, -0.2921, 0.2921, false, false, false))
            .backRight(new ModuleConfig(3, 4, 21, -0.371826171875, -0.2921, -0.2921, true, false, false))
            .gearing(6.394736842105262, 12.1, 4.5, 0.0508) // coupling 4.5 verified via Tuner X
            .speed(4.99, 0.75 * 2 * Math.PI)
            .steerPID(new PIDGains(100, 0, 0.5, 0.1, 1.5, 0))
            .drivePID(new PIDGains(0.1, 0, 0, 0, 0.124, 0))
            .currentLimits(40, 35, 20, 120)
            .deadband(0.1, 0.1)
            .build();

    // TODO: Calibrate encoder offsets via Phoenix Tuner X before use
    // public static final DrivetrainConfig FLUX_2026 = DrivetrainConfig.builder()
    //         .canBus("*")
    //         .pigeonId(20)
    //         .frontLeft(new ModuleConfig(2, 1, 21, 0.0, 0.2921, 0.2921, false, false, false))
    //         .frontRight(new ModuleConfig(4, 3, 22, 0.0, 0.2921, -0.2921, true, false, false))
    //         .backLeft(new ModuleConfig(6, 5, 23, 0.0, -0.2921, 0.2921, false, false, false))
    //         .backRight(new ModuleConfig(8, 7, 24, 0.0, -0.2921, -0.2921, true, false, false))
    //         .gearing(6.394736842105262, 12.1, 4.5, 0.0508)
    //         .speed(4.99, 0.75 * 2 * Math.PI)
    //         .steerPID(new PIDGains(100, 0, 0.5, 0.1, 1.5, 0))
    //         .drivePID(new PIDGains(0.1, 0, 0, 0, 0.124, 0))
    //         .currentLimits(40, 35, 20, 120)
    //         .deadband(0.1, 0.1)
    //         .build();

    private Robots() {}
}
