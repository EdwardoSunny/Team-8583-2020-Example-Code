package frc.robot.team8583.subsystems.shooter.spindex.states;

import frc.robot.team8583.subsystems.shooter.spindex.Spindex;

public class SpindexFastClockwiseAntiJam extends SpindexState
{
    public SpindexFastClockwiseAntiJam(Spindex spindex, double previousAutoReverseTimestamp)
    {
        super(spindex, previousAutoReverseTimestamp);
        spindex.setSpin(true, true, true);
    }

    @Override
    public void toggle()
    {
        spindex.stop();
    }

    @Override
    public void decelerte()
    {
        spindex.setState(new SpindexSlowClockwiseAntiJam(spindex, previousAutoReverseTimestamp));
    }

    @Override
    public void disableAntiJam()
    {
        spindex.setState(new SpindexFastClockwise(spindex, previousAutoReverseTimestamp));
    }

    @Override
    protected boolean shouldAutoReverse()
    {
        return false;
    }

    @Override
    public String toString()
    {
        return "Fast Clockwise Anti-Jam";
    }
}
