package frc.robot.team8583.subsystems.collector.states;

import frc.robot.team8583.subsystems.collector.Collector;

public abstract class CollectorState
{
    protected final Collector collector;

    public CollectorState(Collector collector)
    {
        this.collector = collector;
    }

    public void update(double timestamp)
    {
    }
}
