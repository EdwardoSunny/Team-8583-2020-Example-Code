package frc.robot.team8583.subsystems.shooter.turret.states;

import frc.robot.team8583.subsystems.shooter.turret.Turret;

public abstract class TurretState
{
    protected final Turret turret;

    public TurretState(Turret turret)
    {
        this.turret = turret;
    }

    public void update()
    {
    }

    public abstract boolean readyToLaunch();
}
