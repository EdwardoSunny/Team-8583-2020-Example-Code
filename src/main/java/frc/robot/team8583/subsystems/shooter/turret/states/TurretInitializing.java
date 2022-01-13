package frc.robot.team8583.subsystems.shooter.turret.states;

import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team8583.subsystems.shooter.turret.Turret;

public class TurretInitializing extends TurretState
{
    private boolean hasInitialHeading = false;

    public TurretInitializing(Turret turret)
    {
        super(turret);
        turret.enableVision();
        turret.stopYawGimbal();
        turret.stopPitchGimbal();
        turret.setState(new TurretActive(turret));
    }

    public TurretInitializing(Turret turret, Rotation2d initialHeading, boolean isFieldCentric)
    {
        super(turret);
        hasInitialHeading = true;
        turret.enableVision();
        if (isFieldCentric)
        {
            turret.setFieldCentricYawTarget(initialHeading);
        }
        else
        {
            turret.setRobotCentricYawTarget(initialHeading);
        }
    }

    @Override
    public synchronized void update()
    {
        if (!hasInitialHeading || turret.yawOnTarget())
        {
            turret.stopYawGimbal();
            turret.setState(new TurretActive(turret));
            return;
        }
    }

    @Override
    public boolean readyToLaunch()
    {
        return false;
    }

    @Override
    public String toString()
    {
        return "Initializing";
    }
}
