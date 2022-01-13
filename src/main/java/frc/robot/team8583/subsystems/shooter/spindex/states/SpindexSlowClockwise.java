package frc.robot.team8583.subsystems.shooter.spindex.states;

import frc.robot.team8583.Constants;
import frc.robot.team8583.subsystems.shooter.spindex.Spindex;

public class SpindexSlowClockwise extends SpindexState
{
    public SpindexSlowClockwise(Spindex spindex, double previousAutoReverseTimestamp)
    {
        super(spindex, previousAutoReverseTimestamp);
        spindex.setSpin(true, false, false);
    }

    @Override
    public void toggle()
    {
        spindex.stop();
    }

    @Override
    public void accelerate()
    {
        spindex.setState(new SpindexFastClockwise(spindex, previousAutoReverseTimestamp));
    }

    @Override
    public void enableAntiJam()
    {
        spindex.setState(new SpindexSlowClockwiseAntiJam(spindex, previousAutoReverseTimestamp));
    }

    @Override
    protected boolean shouldAutoReverse()
    {
        return spindex.shouldAutoReverse(Constants.Shooter.Spindex.AUTO_REVERSE_THRESHOLD_CURRENT_SLOW);
    }

    @Override
    protected void reverse(double timestamp)
    {
        spindex.setState(new SpindexSlowCounterclockwise(spindex, timestamp));
    }

    @Override
    public String toString()
    {
        return "Slow Clockwise";
    }
}
