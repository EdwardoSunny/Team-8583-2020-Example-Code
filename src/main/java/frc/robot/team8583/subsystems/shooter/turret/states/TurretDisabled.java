package frc.robot.team8583.subsystems.shooter.turret.states;

import frc.robot.team8583.subsystems.shooter.turret.Turret;

public class TurretDisabled extends TurretState
{
    public TurretDisabled(Turret turret)
    {
        super(turret);
        turret.disableVision();
        turret.stopYawGimbal();
        turret.stopPitchGimbal();
    }

    @Override
    public boolean readyToLaunch()
    {
        return false;
    }

    @Override
    public String toString()
    {
        return "Disabled";
    }
}
