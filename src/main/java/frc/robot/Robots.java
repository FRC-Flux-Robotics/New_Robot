package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
            .frontLeft(new ModuleConfig(/* drive */ 7, /* steer */ 8, /* encoder */ 23,
                    /* offset */ 0.121337890625, /* x */ 0.2921, /* y */ 0.2921,
                    /* invDrive */ false, /* invSteer */ false, /* invEnc */ false))
            .frontRight(new ModuleConfig(/* drive */ 1, /* steer */ 2, /* encoder */ 20,
                    /* offset */ -0.294921875, /* x */ 0.2921, /* y */ -0.2921,
                    /* invDrive */ true, /* invSteer */ false, /* invEnc */ false))
            .backLeft(new ModuleConfig(/* drive */ 5, /* steer */ 6, /* encoder */ 22,
                    /* offset */ 0.040771484375, /* x */ -0.2921, /* y */ 0.2921,
                    /* invDrive */ false, /* invSteer */ false, /* invEnc */ false))
            .backRight(new ModuleConfig(/* drive */ 3, /* steer */ 4, /* encoder */ 21,
                    /* offset */ -0.376953125, /* x */ -0.2921, /* y */ -0.2921,
                    /* invDrive */ true, /* invSteer */ false, /* invEnc */ false))
            .gearing(6.394736842105262, 12.1, 4.5, 0.0508) // coupling 4.5 verified via Tuner X
            .speed(4.99, 0.75 * 2 * Math.PI)
            .steerPID(new PIDGains(100, 0, 0.5, 0.1, 1.5, 0))
            .drivePID(new PIDGains(0.1, 0, 0, 0, 0.124, 0))
            .currentLimits(40, 35, 20, 120)
            .deadband(0.05, 0.1)
            .mass(74.0, 6.0) // ~163 lbs robot, MOI estimate for ~28" frame
            .camera("OV9281", new Transform3d(0.3, 0, 0.25, new Rotation3d(0, Math.toRadians(-15), 0)))
            .build();

    // TODO: Calibrate encoder offsets via Phoenix Tuner X before use
    // public static final DrivetrainConfig FLUX_2026 = DrivetrainConfig.builder()
    //         .canBus("*")
    //         .pigeonId(20)
    //         .frontLeft(new ModuleConfig(/* drive */ 2, /* steer */ 1, /* encoder */ 21,
    //                 /* offset */ 0.0, /* x */ 0.2921, /* y */ 0.2921,
    //                 /* invDrive */ false, /* invSteer */ false, /* invEnc */ false))
    //         .frontRight(new ModuleConfig(/* drive */ 4, /* steer */ 3, /* encoder */ 22,
    //                 /* offset */ 0.0, /* x */ 0.2921, /* y */ -0.2921,
    //                 /* invDrive */ true, /* invSteer */ false, /* invEnc */ false))
    //         .backLeft(new ModuleConfig(/* drive */ 6, /* steer */ 5, /* encoder */ 23,
    //                 /* offset */ 0.0, /* x */ -0.2921, /* y */ 0.2921,
    //                 /* invDrive */ false, /* invSteer */ false, /* invEnc */ false))
    //         .backRight(new ModuleConfig(/* drive */ 8, /* steer */ 7, /* encoder */ 24,
    //                 /* offset */ 0.0, /* x */ -0.2921, /* y */ -0.2921,
    //                 /* invDrive */ true, /* invSteer */ false, /* invEnc */ false))
    //         .gearing(6.394736842105262, 12.1, 4.5, 0.0508)
    //         .speed(4.99, 0.75 * 2 * Math.PI)
    //         .steerPID(new PIDGains(100, 0, 0.5, 0.1, 1.5, 0))
    //         .drivePID(new PIDGains(0.1, 0, 0, 0, 0.124, 0))
    //         .currentLimits(40, 35, 20, 120)
    //         .deadband(0.1, 0.1)
    //         .build();

    private Robots() {}
}
