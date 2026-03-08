package frc.lib.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.RobotController;
import java.util.List;
import java.util.Optional;
import java.util.function.IntFunction;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class DrivetrainIOTalonFX implements DrivetrainIO {
    private final Supplier<SwerveDriveState> stateSupplier;
    private final IntFunction<SwerveModule<TalonFX, TalonFX, CANcoder>> moduleSupplier;
    private final double driveGearRatio;
    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] poseEstimators;

    public DrivetrainIOTalonFX(
            Supplier<SwerveDriveState> stateSupplier,
            IntFunction<SwerveModule<TalonFX, TalonFX, CANcoder>> moduleSupplier,
            double driveGearRatio,
            List<CameraConfig> cameraConfigs,
            AprilTagFieldLayout fieldLayout) {
        this.stateSupplier = stateSupplier;
        this.moduleSupplier = moduleSupplier;
        this.driveGearRatio = driveGearRatio;

        if (cameraConfigs != null && fieldLayout != null) {
            cameras = new PhotonCamera[cameraConfigs.size()];
            poseEstimators = new PhotonPoseEstimator[cameraConfigs.size()];
            for (int i = 0; i < cameraConfigs.size(); i++) {
                CameraConfig cfg = cameraConfigs.get(i);
                cameras[i] = new PhotonCamera(cfg.name());
                poseEstimators[i] = new PhotonPoseEstimator(
                        fieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        cfg.robotToCamera());
                poseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            }
        } else {
            cameras = new PhotonCamera[0];
            poseEstimators = new PhotonPoseEstimator[0];
        }
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
            inputs.driveVelocityMps[i] = state.ModuleStates[i].speedMetersPerSecond;
            inputs.drivePositionRad[i] = module.getDriveMotor().getPosition().getValueAsDouble()
                    * 2.0 * Math.PI / driveGearRatio;

            inputs.driveTempC[i] = module.getDriveMotor().getDeviceTemp().getValueAsDouble();
            inputs.steerTempC[i] = module.getSteerMotor().getDeviceTemp().getValueAsDouble();
            inputs.driveCurrentA[i] = module.getDriveMotor().getStatorCurrent().getValueAsDouble();
            inputs.steerCurrentA[i] = module.getSteerMotor().getStatorCurrent().getValueAsDouble();
        }

        // Battery voltage
        inputs.batteryVoltage = RobotController.getBatteryVoltage();

        // Vision
        int camCount = cameras.length;
        if (camCount > 0) {
            inputs.visionConnected = new boolean[camCount];
            inputs.visionHasEstimate = new boolean[camCount];
            inputs.visionPoseX = new double[camCount];
            inputs.visionPoseY = new double[camCount];
            inputs.visionPoseRotDeg = new double[camCount];
            inputs.visionTimestampSec = new double[camCount];
            inputs.visionTagCount = new int[camCount];
            inputs.visionAmbiguity = new double[camCount];

            for (int i = 0; i < camCount; i++) {
                inputs.visionConnected[i] = cameras[i].isConnected();

                List<PhotonPipelineResult> results = cameras[i].getAllUnreadResults();
                if (results.isEmpty()) {
                    continue;
                }

                PhotonPipelineResult latest = results.get(results.size() - 1);
                Optional<EstimatedRobotPose> estimate = poseEstimators[i].update(latest);
                if (estimate.isPresent()) {
                    EstimatedRobotPose est = estimate.get();
                    inputs.visionHasEstimate[i] = true;
                    inputs.visionPoseX[i] = est.estimatedPose.getX();
                    inputs.visionPoseY[i] = est.estimatedPose.getY();
                    inputs.visionPoseRotDeg[i] =
                            est.estimatedPose.getRotation().toRotation2d().getDegrees();
                    inputs.visionTimestampSec[i] = est.timestampSeconds;
                    inputs.visionTagCount[i] = est.targetsUsed.size();
                    if (!est.targetsUsed.isEmpty()) {
                        inputs.visionAmbiguity[i] = est.targetsUsed.get(0).getPoseAmbiguity();
                    }
                }
            }
        }
    }
}
