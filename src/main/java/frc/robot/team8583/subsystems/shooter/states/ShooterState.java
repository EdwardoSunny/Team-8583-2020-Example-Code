package frc.robot.team8583.subsystems.shooter.states;

import frc.robot.team8583.subsystems.shooter.Shooter;

public abstract class ShooterState
{
    protected final Shooter shooter;

    public ShooterState(Shooter shooter)
    {
        this.shooter = shooter;
    }

    public void update()
    {
    }
}
