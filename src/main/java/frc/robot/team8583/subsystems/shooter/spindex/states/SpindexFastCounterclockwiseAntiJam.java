package frc.robot.team8583.subsystems.shooter.spindex.states;

import frc.robot.team8583.subsystems.shooter.spindex.Spindex;

public class SpindexFastCounterclockwiseAntiJam extends SpindexState
{
    public SpindexFastCounterclockwiseAntiJam(Spindex spindex, double previousAutoReverseTimestamp)
    {
        super(spindex, previousAutoReverseTimestamp);
        spindex.setSpin(false, true, true);
    }

    @Override
    public void toggle()
    {
        spindex.stop();
    }

    @Override
    public void decelerte()
    {
        spindex.setState(new SpindexSlowCounterclockwiseAntiJam(spindex, previousAutoReverseTimestamp));
    }

    @Override
    public void disableAntiJam()
    {
        spindex.setState(new SpindexFastCounterclockwise(spindex, previousAutoReverseTimestamp));
    }

    @Override
    public boolean shouldAutoReverse()
    {
        return false;
    }

    @Override
    public String toString()
    {
        return "Fast Counterclockwise Anti-Jam";
    }
}
