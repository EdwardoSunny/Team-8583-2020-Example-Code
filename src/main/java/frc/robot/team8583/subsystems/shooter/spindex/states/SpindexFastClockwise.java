package frc.robot.team8583.subsystems.shooter.spindex.states;

import frc.robot.team8583.Constants;
import frc.robot.team8583.subsystems.shooter.spindex.Spindex;

public class SpindexFastClockwise extends SpindexState
{
    public SpindexFastClockwise(Spindex spindex, double previousAutoReverseTimestamp)
    {
        super(spindex, previousAutoReverseTimestamp);
        spindex.setSpin(true, true, false);
    }

    @Override
    public void toggle()
    {
        spindex.stop();
    }

    @Override
    public void decelerte()
    {
        spindex.setState(new SpindexSlowClockwise(spindex, previousAutoReverseTimestamp));
    }

    @Override
    public void enableAntiJam()
    {
        spindex.setState(new SpindexFastClockwiseAntiJam(spindex, previousAutoReverseTimestamp));
    }

    @Override
    protected boolean shouldAutoReverse()
    {
        return spindex.shouldAutoReverse(Constants.Shooter.Spindex.AUTO_REVERSE_THRESHOLD_CURRENT_FAST);
    }

    @Override
    protected void reverse(double timestamp)
    {
        spindex.setState(new SpindexSlowCounterclockwise(spindex, timestamp));
    }

    @Override
    public String toString()
    {
        return "Fast Clockwise";
    }
}
