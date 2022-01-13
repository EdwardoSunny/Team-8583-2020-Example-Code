package frc.robot.team8583.subsystems.shooter.spindex.states;

import frc.robot.team8583.Constants;
import frc.robot.team8583.subsystems.shooter.spindex.Spindex;

public class SpindexSlowCounterclockwise extends SpindexState
{
    public SpindexSlowCounterclockwise(Spindex spindex, double previousAutoReverseTimestamp)
    {
        super(spindex, previousAutoReverseTimestamp);
        spindex.setSpin(false, false, false);
    }

    @Override
    public void toggle()
    {
        spindex.stop();
    }

    @Override
    public void accelerate()
    {
        spindex.setState(new SpindexFastCounterclockwise(spindex, previousAutoReverseTimestamp));
    }

    @Override
    public void enableAntiJam()
    {
        spindex.setState(new SpindexSlowCounterclockwiseAntiJam(spindex, previousAutoReverseTimestamp));
    }

    @Override
    protected boolean shouldAutoReverse()
    {
        return spindex.shouldAutoReverse(Constants.Shooter.Spindex.AUTO_REVERSE_THRESHOLD_CURRENT_SLOW);
    }

    @Override
    protected void reverse(double timestamp)
    {
        spindex.setState(new SpindexSlowClockwise(spindex, timestamp));
    }

    @Override
    public String toString()
    {
        return "Slow Counterclockwise";
    }
}
