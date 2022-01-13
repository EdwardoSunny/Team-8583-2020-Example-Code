package frc.robot.team8583.subsystems.collector.states;

import frc.robot.team8583.subsystems.collector.Collector;

public class CollectorDisabled extends CollectorState
{
    public CollectorDisabled(Collector collector)
    {
        super(collector);
        collector.stop();
        collector.releaseElevation();
    }

    @Override
    public String toString()
    {
        return "Disabled";
    }
}
