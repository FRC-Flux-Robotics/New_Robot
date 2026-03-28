package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FuelConstants;

public class VelocityMech extends SubsystemBase {
    private final String name;
    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut brake = new NeutralOut();

    private int direction = FuelConstants.Forward;
    private boolean running = false;
    private boolean targetVelocityChanged = false;
    private double velocityRPM = 0;
    private double targetVelocity = DefaultVelocityRPS;
    private double velocity = 0;

    public static final double DefaultVelocityRPS = 50.0;
    public static final double MaxMotorRPM = 6000;

    public static final double DefaultKP = 0.11;  // An error of 1 rotation per second results in 0.11 V output
    public static final double DefaultKD = 0.0;
    public static final double DefaultKI = 0.0;
    public static final double DefaultKV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    public static final double DefaultKS = 0.01; // To account for friction, add 0.1 V of static feedforward
    public static final double DefaultMaxOutput = 1.0;
    public static final double DefaultMinOutput = -1.0;
    public static final double DeltaRPM = 100;

    // PID coefficients
    public double kP = DefaultKP;
    public double kD = DefaultKD;
    public double kI = DefaultKI;
    public double kV = DefaultKV;
    public double kS = DefaultKS;
    public double kMaxOutput = DefaultMaxOutput;
    public double kMinOutput = DefaultMinOutput;

    public double rpmDelta = DeltaRPM;

    private boolean atSpeed = false;

    public VelocityMech(CANBus canBus, String name, int motorId) {
        this.name = name;
        motor = new TalonFX(motorId, canBus);
        config = new TalonFXConfiguration();

        setConfig();
    }

    public void setTargetSpeed(double speed)
    {
        if (targetVelocity != speed)
        {
            targetVelocity = speed;
            targetVelocityChanged = true;
        }
    }

    public boolean atTarget(double speed)
    {
        return atSpeed;
    }

    public double getVelocity()
    {
        return motor.getVelocity().getValue().magnitude();
    }

    @Override
    public void periodic() {
        if (running && targetVelocityChanged)
        {
            double vel = direction == FuelConstants.Backward ? -targetVelocity : targetVelocity;
            motor.setControl(velocityVoltage.withVelocity(vel));
        }

        double vel = 60 * getVelocity();
        if (vel != velocity)
        {
            String prefix = name + "/";
            SmartDashboard.putNumber(prefix + "RPM", vel);
            velocity = vel;
        }
        double target = direction == FuelConstants.Backward ? -targetVelocity : targetVelocity;
        atSpeed = Math.abs(vel - target) <= rpmDelta;
    }

    public void reset()
    {
        running = false;
        targetVelocityChanged = false;
        atSpeed = false;
        velocityRPM = 0;
    }

    public void run() {
        run(FuelConstants.Forward);
    }

    public void run(int direction) {
        if (running)
            return;

        this.direction = direction;
        double setRpm = direction == FuelConstants.Backward ? -targetVelocity : targetVelocity;
        {
            motor.setControl(velocityVoltage.withVelocity(setRpm));
        }
        running = true;
    }

    public void setSpeed(double speed) {
        targetVelocityChanged = true;
        speed = targetVelocity;

        motor.setControl(velocityVoltage.withVelocity(speed));
    }

    public void stop() {
        motor.setControl(brake);
        reset();
    }

    public void setConfig()
    {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Voltage.withPeakForwardVoltage(Volts.of(FuelConstants.VelocityPeakVoltage)).withPeakReverseVoltage(Volts.of(-FuelConstants.VelocityPeakVoltage));
        config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(FuelConstants.VelocityCurrentLimit)).withSupplyCurrentLimitEnable(true));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public void putParams() {
        String prefix = name + "/";

        SmartDashboard.putNumber(prefix + "Set RPM", 60.0 * targetVelocity);
        SmartDashboard.putNumber(prefix + "RPM", velocity);

        SmartDashboard.putNumber(prefix + "kP", kP);
        SmartDashboard.putNumber(prefix + "kD", kD);
        SmartDashboard.putNumber(prefix + "kI", kI);
        SmartDashboard.putNumber(prefix + "kV", kV);
        SmartDashboard.putNumber(prefix + "kS", kS);
        SmartDashboard.putNumber(prefix + "MaxOutput", kMaxOutput);
        SmartDashboard.putNumber(prefix + "MinOutput", kMinOutput);

        SmartDashboard.putNumber(prefix + "RpmDelta", rpmDelta);

        SmartDashboard.setPersistent(prefix + "Set RPM");
        SmartDashboard.setPersistent(prefix + "RPM");
        SmartDashboard.setPersistent(prefix + "kP");
        SmartDashboard.setPersistent(prefix + "kD");
        SmartDashboard.setPersistent(prefix + "kI");
        SmartDashboard.setPersistent(prefix + "kV");
        SmartDashboard.setPersistent(prefix + "kS");
        SmartDashboard.setPersistent(prefix + "MaxOutput");
        SmartDashboard.setPersistent(prefix + "MinOutput");
        SmartDashboard.setPersistent(prefix + "RpmDelta");

        double v = motor.getRotorVelocity().getValueAsDouble();
        double p = motor.getRotorPosition().getValueAsDouble();
        SmartDashboard.putNumber(prefix + "Rotor V", v);
        SmartDashboard.putNumber(prefix + "Rotor P", p);
    }

    public void getParams() {
        String prefix = name + "/";

        kP = SmartDashboard.getNumber(prefix + "kP", DefaultKP);
        kD = SmartDashboard.getNumber(prefix + "kD", DefaultKD);
        kI = SmartDashboard.getNumber(prefix + "kI", DefaultKI);
        kV = SmartDashboard.getNumber(prefix + "kV", DefaultKV);
        kS = SmartDashboard.getNumber(prefix + "kS", DefaultKS);
        kMaxOutput = SmartDashboard.getNumber(prefix + "MaxOutput", kMaxOutput);
        kMinOutput = SmartDashboard.getNumber(prefix + "MinOutput", kMinOutput);
        rpmDelta = SmartDashboard.getNumber(prefix + "RpmDelta", DeltaRPM);

        setConfig();

        double vel = SmartDashboard.getNumber(prefix + "Set RPM", DefaultVelocityRPS);
        vel /= 60.0;
        setTargetSpeed(vel);
    }
}
