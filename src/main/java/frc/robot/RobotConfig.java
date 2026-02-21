package frc.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

/**
 * Hardware configuration for the swerve drive robot.
 * Defines CAN bus assignments, Pigeon 2 IMU, and swerve module configurations.
 */
public final class RobotConfig {
    /** CAN bus name for system devices (typically "rio" for RoboRIO-integrated CAN). */
    public final String systemCANBus;

    /** CAN bus name for drivetrain devices (e.g., "Drivetrain" for CANivore). */
    public final String driveCANBus;

    /** CAN ID of the Pigeon 2 IMU. */
    public final int pigeonId;

    /** Optional Pigeon 2 configuration. Null uses defaults. */
    public final Pigeon2Configuration pigeonConfigs;

    public final SwerveModuleConfig frontLeft;
    public final SwerveModuleConfig frontRight;
    public final SwerveModuleConfig backLeft;
    public final SwerveModuleConfig backRight;

    public static final boolean InvertLeftSide = false;
    public static final boolean InvertRightSide = true;

    /** FLUX 2026 robot configuration. Encoder offsets must be calibrated via Phoenix Tuner X. */
    public static final RobotConfig FluxRobot2026 = new RobotConfig(
            "*",
            "rio",
            20,
            //                     drive, steer, encoder, offset, x,     y,     invertSide,       steerInv, encInv
            new SwerveModuleConfig(2,     1,     21,      0.0,    11.5,  11.5,  InvertLeftSide,   false,    false),  // Front Left
            new SwerveModuleConfig(4,     3,     22,      0.0,    11.5,  -11.5, InvertRightSide,  false,    false),  // Front Right
            new SwerveModuleConfig(6,     5,     23,      0.0,    -11.5, 11.5,  InvertLeftSide,   false,    false),  // Back Left
            new SwerveModuleConfig(8,     7,     24,      0.0,    -11.5, -11.5, InvertRightSide,  false,    false)); // Back Right

    public RobotConfig(
            String driveCANBus,
            String systemCANBus,
            int pigeonId,
            SwerveModuleConfig fl,
            SwerveModuleConfig fr,
            SwerveModuleConfig bl,
            SwerveModuleConfig br) {
        if (driveCANBus == null || driveCANBus.isEmpty()) {
            throw new IllegalArgumentException("driveCANBus must not be null or empty");
        }
        if (systemCANBus == null || systemCANBus.isEmpty()) {
            throw new IllegalArgumentException("systemCANBus must not be null or empty");
        }
        if (pigeonId < 0 || pigeonId > 62) {
            throw new IllegalArgumentException("pigeonId must be 0-62, got: " + pigeonId);
        }
        if (fl == null || fr == null || bl == null || br == null) {
            throw new IllegalArgumentException("All module configs must not be null");
        }

        this.driveCANBus = driveCANBus;
        this.systemCANBus = systemCANBus;
        this.pigeonId = pigeonId;
        pigeonConfigs = null;
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
    }
}
