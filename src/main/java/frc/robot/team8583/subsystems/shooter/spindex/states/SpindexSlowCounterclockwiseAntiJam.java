package frc.robot.team8583.subsystems.shooter.spindex.states;

import frc.robot.team8583.subsystems.shooter.spindex.Spindex;

public class SpindexSlowCounterclockwiseAntiJam extends SpindexState
{
    public SpindexSlowCounterclockwiseAntiJam(Spindex spindex, double previousAutoReverseTimestamp)
    {
        super(spindex, previousAutoReverseTimestamp);
        spindex.setSpin(false, false, true);
    }

    @Override
    public void toggle()
    {
        spindex.stop();
    }

    @Override
    public void accelerate()
    {
        spindex.setState(new SpindexFastCounterclockwiseAntiJam(spindex, previousAutoReverseTimestamp));
    }

    @Override
    public void disableAntiJam()
    {
        spindex.setState(new SpindexSlowCounterclockwise(spindex, previousAutoReverseTimestamp));
    }

    @Override
    protected boolean shouldAutoReverse()
    {
        return false;
    }

    @Override
    public String toString()
    {
        return "Slow Counterclockwise Anti-Jam";
    }
}
