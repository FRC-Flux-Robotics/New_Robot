package frc.lib.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import java.util.function.IntFunction;
import java.util.function.Supplier;

public class DrivetrainIOTalonFX implements DrivetrainIO {
    private final Supplier<SwerveDriveState> stateSupplier;
    private final IntFunction<SwerveModule<TalonFX, TalonFX, CANcoder>> moduleSupplier;

    public DrivetrainIOTalonFX(
            Supplier<SwerveDriveState> stateSupplier,
            IntFunction<SwerveModule<TalonFX, TalonFX, CANcoder>> moduleSupplier) {
        this.stateSupplier = stateSupplier;
        this.moduleSupplier = moduleSupplier;
    }

    @Override
    public void updateInputs(DrivetrainIOInputsAutoLogged inputs) {
        SwerveDriveState state = stateSupplier.get();

        // Gyro
        inputs.gyroYawDeg = state.Pose.getRotation().getDegrees();
        inputs.gyroRateDegPerSec = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);

        // Odometry
        inputs.odometryPeriodSec = state.OdometryPeriod;

        // Per-module data
        for (int i = 0; i < 4; i++) {
            SwerveModule<TalonFX, TalonFX, CANcoder> module = moduleSupplier.apply(i);

            inputs.steerAngleDeg[i] = state.ModuleStates[i].angle.getDegrees();
            inputs.driveVelocityRadPerSec[i] = state.ModuleStates[i].speedMetersPerSecond;
            inputs.drivePositionRad[i] = module.getDriveMotor().getPosition().getValueAsDouble()
                    * 2.0 * Math.PI;

            inputs.driveTempC[i] = module.getDriveMotor().getDeviceTemp().getValueAsDouble();
            inputs.steerTempC[i] = module.getSteerMotor().getDeviceTemp().getValueAsDouble();
            inputs.driveCurrentA[i] = module.getDriveMotor().getStatorCurrent().getValueAsDouble();
        }
    }
}
