package frc.robot.team8583.subsystems.shooter.spindex.states;

import frc.robot.team8583.subsystems.shooter.spindex.Spindex;

public class SpindexSlowClockwiseAntiJam extends SpindexState
{
    public SpindexSlowClockwiseAntiJam(Spindex spindex, double previousAutoReverseTimestamp)
    {
        super(spindex, previousAutoReverseTimestamp);
        spindex.setSpin(true, false, true);
    }

    @Override
    public void toggle()
    {
        spindex.stop();
    }

    @Override
    public void accelerate()
    {
        spindex.setState(new SpindexFastClockwiseAntiJam(spindex, previousAutoReverseTimestamp));
    }

    @Override
    public void disableAntiJam()
    {
        spindex.setState(new SpindexSlowClockwise(spindex, previousAutoReverseTimestamp));
    }

    @Override
    protected boolean shouldAutoReverse()
    {
        return false;
    }

    @Override
    public String toString()
    {
        return "Slow Clockwise Anti-Jam";
    }
}
