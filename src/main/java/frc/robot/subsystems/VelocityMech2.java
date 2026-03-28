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

public class VelocityMech2 extends SubsystemBase {
    private final String name;
    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFXConfiguration config1;
    private final TalonFXConfiguration config2;

    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut brake = new NeutralOut();

    private int direction = FuelConstants.Backward;
    private boolean running = false;
    private boolean targetVelocityChanged = false;
    private double targetVelocity = DefaultVelocityRPM;
    private double velocity = 0;
    private double velocity2 = 0;

    public static final double DefaultVelocityRPM = 3000.0;
    public static final double MaxMotorRPM = 6000;

    public static final double DefaultKP = 0.5;
    public static final double DefaultKD = 0.0005;
    public static final double DefaultKI = 0.0;
    public static final double DefaultKV = 0.12;
    public static final double DefaultKS = 0.01;
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

    public VelocityMech2(CANBus canBus, String name, int motorId1, int motorId2) {
        this.name = name;
        motor1 = new TalonFX(motorId1, canBus);
        motor2 = new TalonFX(motorId2, canBus);
        config1 = new TalonFXConfiguration();
        config2 = new TalonFXConfiguration();

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

    public boolean atTarget() {
        return atSpeed;
    }

    public boolean running()
    {
        return running;
    }

    @Override
    public void periodic() {
        if (running && targetVelocityChanged)
        {
            double vel = direction == FuelConstants.Backward ? -targetVelocity : targetVelocity;
            motor1.setControl(velocityVoltage.withVelocity(vel));
            motor2.setControl(velocityVoltage.withVelocity(-vel));
        }

        double vel = 60 * getVelocity();
        {
            String prefix = name + "/";
            SmartDashboard.putNumber(prefix + "RPM_1", vel);
            velocity = vel;
        }
        vel = 60 * getVelocity2();
        {
            String prefix = name + "/";
            SmartDashboard.putNumber(prefix + "RPM_2", vel);
            velocity2 = vel;
        }
        double target = direction == FuelConstants.Backward ? -targetVelocity : targetVelocity;
        atSpeed = Math.abs(vel - target) <= rpmDelta;
    }

    public void reset()
    {
        running = false;
        targetVelocityChanged = false;
        atSpeed = false;
    }

    public void setSpeed(double speed) {
        targetVelocityChanged = true;
        running = true;
        speed = targetVelocity;

        speed = direction == FuelConstants.Backward ? -speed : speed;

        motor1.setControl(velocityVoltage.withVelocity(speed));
        motor2.setControl(velocityVoltage.withVelocity(-speed));
    }

    public void speedUp(double speedStep)
    {
        targetVelocity += speedStep;
        if (targetVelocity > FuelConstants.MaxMotorRPStoSet)
        {
            targetVelocity = FuelConstants.MaxMotorRPStoSet;
        }
        setSpeed(targetVelocity);
    }

    public void speedDown(double speedStep)
    {
        targetVelocity -= speedStep;
        if (targetVelocity <= 0)
        {
            targetVelocity = 0;
            stop();
        }
        else
            setSpeed(targetVelocity);
    }

    public void stop() {
        motor1.setControl(brake);
        motor2.setControl(brake);
        reset();
    }

    public double getVelocity()
    {
        return motor1.getVelocity().getValue().magnitude();
    }

    public double getVelocity2()
    {
        return motor2.getVelocity().getValue().magnitude();
    }

    public void setConfig()
    {
        configureMotor(motor1, config1);
        configureMotor(motor2, config2);
    }

    private void configureMotor(TalonFX motor, TalonFXConfiguration config) {
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

        SmartDashboard.putNumber(prefix + "Set RPM", targetVelocity);
        SmartDashboard.putNumber(prefix + "RPM_1", 60 * velocity);
        SmartDashboard.putNumber(prefix + "RPM_2", 60 * velocity2);

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

        double v = motor1.getRotorVelocity().getValueAsDouble();
        double p = motor2.getRotorPosition().getValueAsDouble();
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

        double vel = SmartDashboard.getNumber(prefix + "Set RPM", DefaultVelocityRPM);
        setTargetSpeed(vel);
    }
}
