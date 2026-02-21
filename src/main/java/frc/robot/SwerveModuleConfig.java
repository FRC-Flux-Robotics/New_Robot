package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class SwerveModuleConfig {
    public final int driveMotorId;
    public final int steerMotorId;
    public final int encoderId;
    public final boolean invertSide;
    public final boolean steerMotorInverted;
    public final boolean encoderInverted;
    public final Angle encoderOffset;

    public final Distance xPos;
    public final Distance yPos;

    public SwerveModuleConfig(
            int driveMotorId,
            int steerMotorId,
            int encoderId,
            double encoderOffsetAngle,
            double xPos,
            double yPos,
            boolean invertSide,
            boolean steerMotorInverted,
            boolean encoderInverted) {
        // Validate CAN IDs (CTRE valid range is 0-62)
        if (driveMotorId < 0 || driveMotorId > 62) {
            throw new IllegalArgumentException("driveMotorId must be 0-62, got: " + driveMotorId);
        }
        if (steerMotorId < 0 || steerMotorId > 62) {
            throw new IllegalArgumentException("steerMotorId must be 0-62, got: " + steerMotorId);
        }
        if (encoderId < 0 || encoderId > 62) {
            throw new IllegalArgumentException("encoderId must be 0-62, got: " + encoderId);
        }

        // Validate encoder offset (should be within one rotation)
        if (encoderOffsetAngle < -1.0 || encoderOffsetAngle > 1.0) {
            throw new IllegalArgumentException(
                    "encoderOffsetAngle must be -1.0 to 1.0 rotations, got: " + encoderOffsetAngle);
        }

        this.driveMotorId = driveMotorId;
        this.steerMotorId = steerMotorId;
        this.encoderId = encoderId;
        this.encoderOffset = Rotations.of(encoderOffsetAngle);
        this.invertSide = invertSide;
        this.steerMotorInverted = steerMotorInverted;
        this.encoderInverted = encoderInverted;
        this.xPos = Inches.of(xPos);
        this.yPos = Inches.of(yPos);
    }
}
