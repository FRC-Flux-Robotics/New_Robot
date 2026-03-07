package frc.lib.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Unified configuration for a swerve drivetrain. Holds all hardware IDs, tuning parameters,
 * mechanical constants, and current limits in one place. Use {@link #builder()} to construct.
 */
public final class DrivetrainConfig {

    // --- CAN bus ---
    public final String canBus;
    public final int pigeonId;

    // --- Swerve modules (FL, FR, BL, BR) ---
    public final ModuleConfig frontLeft;
    public final ModuleConfig frontRight;
    public final ModuleConfig backLeft;
    public final ModuleConfig backRight;

    // --- Mechanical ---
    public final double driveGearRatio;
    public final double steerGearRatio;
    public final double couplingRatio;
    public final double wheelRadiusMeters;

    // --- Speed ---
    public final double maxSpeedMps;
    public final double maxAngularRateRadPerSec;

    // --- PID tuning ---
    public final PIDGains steerGains;
    public final PIDGains driveGains;

    // --- Current limits ---
    public final double driveStatorCurrentLimit;
    public final double driveSupplyCurrentLimit;
    public final double steerStatorCurrentLimit;
    public final double slipCurrentAmps;

    // --- Deadband ---
    public final double translationDeadband;
    public final double rotationDeadband;

    // Cached factory instance (built lazily)
    private SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            cachedFactory;

    private DrivetrainConfig(Builder b) {
        this.canBus = b.canBus;
        this.pigeonId = b.pigeonId;
        this.frontLeft = b.frontLeft;
        this.frontRight = b.frontRight;
        this.backLeft = b.backLeft;
        this.backRight = b.backRight;
        this.driveGearRatio = b.driveGearRatio;
        this.steerGearRatio = b.steerGearRatio;
        this.couplingRatio = b.couplingRatio;
        this.wheelRadiusMeters = b.wheelRadiusMeters;
        this.maxSpeedMps = b.maxSpeedMps;
        this.maxAngularRateRadPerSec = b.maxAngularRateRadPerSec;
        this.steerGains = b.steerGains;
        this.driveGains = b.driveGains;
        this.driveStatorCurrentLimit = b.driveStatorCurrentLimit;
        this.driveSupplyCurrentLimit = b.driveSupplyCurrentLimit;
        this.steerStatorCurrentLimit = b.steerStatorCurrentLimit;
        this.slipCurrentAmps = b.slipCurrentAmps;
        this.translationDeadband = b.translationDeadband;
        this.rotationDeadband = b.rotationDeadband;
    }

    public static Builder builder() {
        return new Builder();
    }

    /** Build CTRE SwerveDrivetrainConstants from this config. */
    public SwerveDrivetrainConstants toSwerveDrivetrainConstants() {
        return new SwerveDrivetrainConstants()
                .withCANBusName(canBus)
                .withPigeon2Id(pigeonId);
    }

    /** Build a PathPlanner RobotConfig from this config. */
    public RobotConfig toRobotConfig() {
        com.pathplanner.lib.config.ModuleConfig ppModuleConfig =
                new com.pathplanner.lib.config.ModuleConfig(
                        Meters.of(wheelRadiusMeters),
                        MetersPerSecond.of(maxSpeedMps),
                        1.0, // wheelCOF placeholder
                        DCMotor.getKrakenX60(1).withReduction(driveGearRatio),
                        Amps.of(driveStatorCurrentLimit),
                        1);

        return new RobotConfig(
                Kilograms.of(74), // ~163 lbs robot mass estimate
                KilogramSquareMeters.of(6.0), // MOI estimate for ~28" frame
                ppModuleConfig,
                modulePosition(frontLeft),
                modulePosition(frontRight),
                modulePosition(backLeft),
                modulePosition(backRight));
    }

    private Translation2d modulePosition(ModuleConfig module) {
        return new Translation2d(
                Meters.of(module.xPositionMeters),
                Meters.of(module.yPositionMeters));
    }

    /** Build CTRE SwerveModuleConstantsFactory from this config. */
    public SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            toModuleConstantsFactory() {
        TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(driveStatorCurrentLimit))
                                .withStatorCurrentLimitEnable(true)
                                .withSupplyCurrentLimit(Amps.of(driveSupplyCurrentLimit))
                                .withSupplyCurrentLimitEnable(true));

        TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(steerStatorCurrentLimit))
                                .withStatorCurrentLimitEnable(true));

        return new SwerveModuleConstantsFactory<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(driveGearRatio)
                .withSteerMotorGearRatio(steerGearRatio)
                .withCouplingGearRatio(couplingRatio)
                .withWheelRadius(Meters.of(wheelRadiusMeters))
                .withSteerMotorGains(steerGains.toSlot0Configs())
                .withDriveMotorGains(driveGains.toSlot0Configs())
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSlipCurrent(Amps.of(slipCurrentAmps))
                .withSpeedAt12Volts(MetersPerSecond.of(maxSpeedMps))
                .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withSteerInertia(KilogramSquareMeters.of(0.01))
                .withDriveInertia(KilogramSquareMeters.of(0.01))
                .withSteerFrictionVoltage(Volts.of(0.2))
                .withDriveFrictionVoltage(Volts.of(0.2));
    }

    private SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            getModuleConstantsFactory() {
        if (cachedFactory == null) {
            cachedFactory = toModuleConstantsFactory();
        }
        return cachedFactory;
    }

    /** Create CTRE SwerveModuleConstants for a single module. */
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
            createModuleConstants(ModuleConfig module) {
        return getModuleConstantsFactory()
                .createModuleConstants(
                        module.steerMotorId,
                        module.driveMotorId,
                        module.encoderId,
                        Rotations.of(module.encoderOffsetRotations),
                        Meters.of(module.xPositionMeters),
                        Meters.of(module.yPositionMeters),
                        module.invertDrive,
                        module.invertSteer,
                        module.invertEncoder);
    }

    public static final class Builder {
        private String canBus;
        private int pigeonId = -1;
        private ModuleConfig frontLeft;
        private ModuleConfig frontRight;
        private ModuleConfig backLeft;
        private ModuleConfig backRight;
        private double driveGearRatio;
        private double steerGearRatio;
        private double couplingRatio;
        private double wheelRadiusMeters;
        private double maxSpeedMps;
        private double maxAngularRateRadPerSec;
        private PIDGains steerGains;
        private PIDGains driveGains;
        private double driveStatorCurrentLimit;
        private double driveSupplyCurrentLimit;
        private double steerStatorCurrentLimit;
        private double slipCurrentAmps;
        private double translationDeadband;
        private double rotationDeadband;

        private Builder() {}

        public Builder canBus(String canBus) {
            this.canBus = canBus;
            return this;
        }

        public Builder pigeonId(int id) {
            if (id < 0 || id > 62) {
                throw new IllegalArgumentException("pigeonId must be 0-62, got: " + id);
            }
            this.pigeonId = id;
            return this;
        }

        public Builder frontLeft(ModuleConfig module) {
            this.frontLeft = module;
            return this;
        }

        public Builder frontRight(ModuleConfig module) {
            this.frontRight = module;
            return this;
        }

        public Builder backLeft(ModuleConfig module) {
            this.backLeft = module;
            return this;
        }

        public Builder backRight(ModuleConfig module) {
            this.backRight = module;
            return this;
        }

        public Builder gearing(
                double driveGearRatio,
                double steerGearRatio,
                double couplingRatio,
                double wheelRadiusMeters) {
            if (driveGearRatio <= 0) {
                throw new IllegalArgumentException("driveGearRatio must be > 0, got: " + driveGearRatio);
            }
            if (steerGearRatio <= 0) {
                throw new IllegalArgumentException("steerGearRatio must be > 0, got: " + steerGearRatio);
            }
            if (couplingRatio < 0) {
                throw new IllegalArgumentException("couplingRatio must be >= 0, got: " + couplingRatio);
            }
            if (wheelRadiusMeters <= 0) {
                throw new IllegalArgumentException(
                        "wheelRadiusMeters must be > 0, got: " + wheelRadiusMeters);
            }
            this.driveGearRatio = driveGearRatio;
            this.steerGearRatio = steerGearRatio;
            this.couplingRatio = couplingRatio;
            this.wheelRadiusMeters = wheelRadiusMeters;
            return this;
        }

        public Builder speed(double maxSpeedMps, double maxAngularRateRadPerSec) {
            if (maxSpeedMps <= 0) {
                throw new IllegalArgumentException("maxSpeedMps must be > 0, got: " + maxSpeedMps);
            }
            if (maxAngularRateRadPerSec <= 0) {
                throw new IllegalArgumentException(
                        "maxAngularRateRadPerSec must be > 0, got: " + maxAngularRateRadPerSec);
            }
            this.maxSpeedMps = maxSpeedMps;
            this.maxAngularRateRadPerSec = maxAngularRateRadPerSec;
            return this;
        }

        public Builder steerPID(PIDGains gains) {
            this.steerGains = gains;
            return this;
        }

        public Builder drivePID(PIDGains gains) {
            this.driveGains = gains;
            return this;
        }

        public Builder currentLimits(
                double driveStator, double driveSupply, double steerStator, double slip) {
            if (driveStator <= 0) {
                throw new IllegalArgumentException(
                        "driveStatorCurrentLimit must be > 0, got: " + driveStator);
            }
            if (driveSupply <= 0) {
                throw new IllegalArgumentException(
                        "driveSupplyCurrentLimit must be > 0, got: " + driveSupply);
            }
            if (steerStator <= 0) {
                throw new IllegalArgumentException(
                        "steerStatorCurrentLimit must be > 0, got: " + steerStator);
            }
            if (slip <= 0) {
                throw new IllegalArgumentException("slipCurrentAmps must be > 0, got: " + slip);
            }
            this.driveStatorCurrentLimit = driveStator;
            this.driveSupplyCurrentLimit = driveSupply;
            this.steerStatorCurrentLimit = steerStator;
            this.slipCurrentAmps = slip;
            return this;
        }

        public Builder deadband(double translation, double rotation) {
            if (translation < 0 || translation > 1) {
                throw new IllegalArgumentException(
                        "translationDeadband must be 0.0-1.0, got: " + translation);
            }
            if (rotation < 0 || rotation > 1) {
                throw new IllegalArgumentException(
                        "rotationDeadband must be 0.0-1.0, got: " + rotation);
            }
            this.translationDeadband = translation;
            this.rotationDeadband = rotation;
            return this;
        }

        public DrivetrainConfig build() {
            requireNonNull(canBus, "canBus");
            if (pigeonId < 0) {
                throw new IllegalStateException("pigeonId must be set");
            }
            requireNonNull(frontLeft, "frontLeft");
            requireNonNull(frontRight, "frontRight");
            requireNonNull(backLeft, "backLeft");
            requireNonNull(backRight, "backRight");
            if (driveGearRatio <= 0) {
                throw new IllegalStateException("gearing must be set");
            }
            if (maxSpeedMps <= 0) {
                throw new IllegalStateException("speed must be set");
            }
            requireNonNull(steerGains, "steerGains");
            requireNonNull(driveGains, "driveGains");
            if (driveStatorCurrentLimit <= 0) {
                throw new IllegalStateException("currentLimits must be set");
            }
            return new DrivetrainConfig(this);
        }

        private static void requireNonNull(Object value, String name) {
            if (value == null) {
                throw new IllegalStateException(name + " must be set");
            }
        }
    }
}
