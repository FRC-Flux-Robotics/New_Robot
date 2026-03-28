package frc.robot.commands;

import frc.robot.FuelConstants;
import frc.robot.subsystems.VelocityMech2;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** A command to toggle the shooter on/off. */
public class ShootCommand extends Command {
    private final VelocityMech2 velocityMech;
    private final DoubleSupplier speed;
    private final int direction;

    public ShootCommand(VelocityMech2 velocityMech, DoubleSupplier speed, int direction) {
        this.velocityMech = velocityMech;
        this.speed = speed;
        this.direction = direction;

        double v = speed.getAsDouble();
        velocityMech.setTargetSpeed(v);
        addRequirements(velocityMech);
    }

    @Override
    public void initialize()
    {
        if (velocityMech.running())
        {
            velocityMech.stop();
        }
        else
        {
            velocityMech.reset();

            double v = speed.getAsDouble();
            if (direction != FuelConstants.Forward)
                v = -v;
             velocityMech.setSpeed(v);
        }
    }

    @Override
    public void execute()
    {
    }

    @Override
    public void end(boolean interrupted)
    {
        velocityMech.stop();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
