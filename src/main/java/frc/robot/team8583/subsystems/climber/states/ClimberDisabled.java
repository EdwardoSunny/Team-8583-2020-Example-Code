package frc.robot.team8583.subsystems.climber.states;

import frc.robot.team8583.subsystems.climber.Climber;

public class ClimberDisabled extends ClimberState
{
    public ClimberDisabled(Climber climber)
    {
        super(climber);
        climber.stop();
        climber.releaseLock();
    }

    @Override
    public String toString()
    {
        return "Disabled";
    }
}
