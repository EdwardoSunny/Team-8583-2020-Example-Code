package frc.robot.team8583.subsystems.climber.states;

import frc.robot.team8583.subsystems.climber.Climber;

public abstract class ClimberState
{
    protected final Climber climber;

    public ClimberState(Climber climber)
    {
        this.climber = climber;
    }

    public void update(double timestamp)
    {
    }
}
