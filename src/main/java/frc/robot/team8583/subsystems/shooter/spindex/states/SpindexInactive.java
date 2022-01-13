package frc.robot.team8583.subsystems.shooter.spindex.states;

import frc.robot.team8583.subsystems.shooter.spindex.Spindex;

public class SpindexInactive extends SpindexState
{
    public SpindexInactive(Spindex spindex)
    {
        super(spindex);
        spindex.stopMotor();
    }

    @Override
    public void update(double timestamp)
    {
    }

    @Override
    public void toggle()
    {
        start();
    }

    @Override
    protected boolean shouldAutoReverse()
    {
        return false;
    }

    @Override
    public void start()
    {
        spindex.setState(new SpindexSlowCounterclockwise(spindex, Double.NaN));
    }

    @Override
    public String toString()
    {
        return "Inactive";
    }
}
