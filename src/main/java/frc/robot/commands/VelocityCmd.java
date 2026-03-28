package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.FuelConstants;
import frc.robot.subsystems.VelocityMech;

public class VelocityCmd extends Command
{
    private final VelocityMech velocityMech;
    private final DoubleSupplier speed;
    private final int direction;

    public VelocityCmd(VelocityMech velocityMech, DoubleSupplier speed, int direction)
    {
        this.velocityMech = velocityMech;
        this.speed = speed;
        this.direction = direction;

        velocityMech.setTargetSpeed(speed.getAsDouble());
        addRequirements(velocityMech);
    }

    @Override
    public void initialize()
    {
        velocityMech.reset();
        double v = speed.getAsDouble();
        if (direction != FuelConstants.Forward)
            v = -v;
        velocityMech.run(direction);
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
