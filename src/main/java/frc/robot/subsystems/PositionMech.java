package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FuelConstants;

public class PositionMech extends SubsystemBase {
    private final String name;
    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    private final MotionMagicVoltage mmReq = new MotionMagicVoltage(0);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut brake = new NeutralOut();

    private double targetPosition = DefaultPosition;
    private double position = 0;
    private boolean running = false;
    private boolean targetPositionChanged = false;

    public static final double DefaultPosition = 100.0;
    public static final double MaxMotorRPM = 6000;

    public static final double DefaultKP = 2.4;  // An error of 1 rotation results in 2.4 V output
    public static final double DefaultKD = 0.1;  // A velocity of 1 rps results in 0.1 V output
    public static final double DefaultKI = 0.0;
    public static final double DefaultKV = 0.0;
    public static final double DefaultKS = 0.0;
    public static final double DefaultMaxOutput = 1.0;
    public static final double DefaultMinOutput = -1.0;
    public static final double DeltaPos = 1;

    public static final int JogStep = -1;

    // PID coefficients
    public double kP = DefaultKP;
    public double kD = DefaultKD;
    public double kI = DefaultKI;
    public double kV = DefaultKV;
    public double kS = DefaultKS;
    public double kMaxOutput = DefaultMaxOutput;
    public double kMinOutput = DefaultMinOutput;

    public double posDelta = DeltaPos;
    private int jogStep = JogStep;

    private boolean atPosition = false;

    public PositionMech(CANBus canBus, String name, int motorId) {
        this.name = name;
        motor = new TalonFX(motorId, canBus);

        config = new TalonFXConfiguration();
        setConfig();
    }

    public void setTargetPosition(double pos)
    {
        targetPosition = pos;
    }

    public boolean atTarget()
    {
        return atPosition;
    }

    public double getPosition()
    {
        return motor.getPosition().getValue().in(Rotations);
    }

    public void resetEncoders()
    {
        motor.setPosition(Rotations.of(0.0));
    }

    @Override
    public void periodic() {
        if (running && targetPositionChanged)
        {
            double pos = targetPosition;
            motor.setControl(positionVoltage.withPosition(pos));
        }

        double pos = getPosition();
        if (pos != position)
        {
            String prefix = name + "/";
            SmartDashboard.putNumber(prefix + "Pos", pos);
            position = pos;
        }
        atPosition = Math.abs(pos - targetPosition) <= posDelta;
    }

    public void reset()
    {
        running = false;
        targetPositionChanged = false;
        atPosition = false;
    }

    public void jogUp(double step)
    {
        double pos = getPosition();
        pos += step;
        targetPosition = pos;
        motor.setControl(positionVoltage.withPosition(pos));
    }

    public void jogDown(double step)
    {
        double pos = getPosition();
        pos -= step;
        targetPosition = pos;

        motor.setControl(positionVoltage.withPosition(pos));
    }

    public void run(double pos) {
        running = true;
        motor.setControl(positionVoltage.withPosition(pos));
    }

    public void run1(double pos) {
        running = true;
        motor.setControl(mmReq.withPosition(pos).withSlot(0));
    }

    public void stop() {
        motor.setControl(brake);
        running = false;
        targetPositionChanged = false;
    }

    public void setConfig()
    {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Voltage.withPeakForwardVoltage(Volts.of(FuelConstants.PositionPeakVoltage)).withPeakReverseVoltage(Volts.of(-FuelConstants.PositionPeakVoltage));
        config.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(FuelConstants.PositionCurrentLimit)).withSupplyCurrentLimitEnable(true));

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

        SmartDashboard.setPersistent(prefix + "Pos");
        SmartDashboard.setPersistent(prefix + "kP");
        SmartDashboard.setPersistent(prefix + "kD");
        SmartDashboard.setPersistent(prefix + "kI");
        SmartDashboard.setPersistent(prefix + "kV");
        SmartDashboard.setPersistent(prefix + "kS");
        SmartDashboard.setPersistent(prefix + "MaxOutput");
        SmartDashboard.setPersistent(prefix + "MinOutput");
        SmartDashboard.setPersistent(prefix + "PosDelta");

        SmartDashboard.putNumber(prefix + "Target Pos", targetPosition);
        SmartDashboard.putNumber(prefix + "Position", position);

        SmartDashboard.putNumber(prefix + "kP", kP);
        SmartDashboard.putNumber(prefix + "kD", kD);
        SmartDashboard.putNumber(prefix + "kI", kI);
        SmartDashboard.putNumber(prefix + "kV", kV);
        SmartDashboard.putNumber(prefix + "kS", kS);
        SmartDashboard.putNumber(prefix + "MaxOutput", kMaxOutput);
        SmartDashboard.putNumber(prefix + "MinOutput", kMinOutput);

        SmartDashboard.putNumber(prefix + "PosDelta", posDelta);

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
        posDelta = SmartDashboard.getNumber(prefix + "posDelta", DeltaPos);

        setConfig();

        double pos = SmartDashboard.getNumber(prefix + "Target Pos", DefaultPosition);
        if (pos != targetPosition)
        {
            targetPosition = pos;
            targetPositionChanged = true;
        }
    }
}
