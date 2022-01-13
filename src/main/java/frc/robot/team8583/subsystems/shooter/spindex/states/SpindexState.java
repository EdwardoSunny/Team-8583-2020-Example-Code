package frc.robot.team8583.subsystems.shooter.spindex.states;

import frc.robot.team8583.Constants;
import frc.robot.team8583.subsystems.shooter.spindex.Spindex;

public abstract class SpindexState
{
    protected final Spindex spindex;
    protected final double previousAutoReverseTimestamp;

    public SpindexState(Spindex spindex)
    {
        this.spindex = spindex;
        this.previousAutoReverseTimestamp = Double.NaN;
    }

    public SpindexState(Spindex spindex, double previousAutoReverseTimestamp)
    {
        this.spindex = spindex;
        this.previousAutoReverseTimestamp = previousAutoReverseTimestamp;
    }

    public void update(double timestamp)
    {
        if (shouldAutoReverse() && (Double.isNaN(previousAutoReverseTimestamp)
                || timestamp - previousAutoReverseTimestamp > Constants.Shooter.Spindex.AUTO_REVERSE_MIN_PERIOD))
        {
            reverse(timestamp);
        }
    }

    public void start()
    {
    }

    public void toggle()
    {
    }

    public void accelerate()
    {
    }

    public void decelerte()
    {
    }

    public void enableAntiJam()
    {
    }

    public void disableAntiJam()
    {
    }

    protected abstract boolean shouldAutoReverse();

    protected void reverse(double timestamp)
    {
    }
}
