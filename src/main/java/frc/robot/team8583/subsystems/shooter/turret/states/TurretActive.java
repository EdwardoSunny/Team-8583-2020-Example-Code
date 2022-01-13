package frc.robot.team8583.subsystems.shooter.turret.states;

import frc.robot.team8583.devices.DriverInterface;
import frc.robot.team8583.drivers.StatefulXboxController;
import frc.robot.team8583.drivers.StatefulXboxController.ButtonId;
import frc.robot.team8583.subsystems.shooter.turret.Turret;

public class TurretActive extends TurretState
{
    private StatefulXboxController codriverController = DriverInterface.getCodriverController();

    public TurretActive(Turret turret)
    {
        super(turret);
        turret.reenableVision();
    }

    @Override
    public synchronized void update()
    {
        if (codriverController.getButton(ButtonId.BUTTON_A).wasPressed())
        {
            turret.setState(new TurretInactive(turret));
            return;
        }

        if (turret.lostVisualTarget())
        {
            turret.stopYawGimbal();
        }
        else
        {
            turret.followVisualTarget();
        }
    }

    @Override
    public boolean readyToLaunch()
    {
        return true;
    }

    @Override
    public String toString()
    {
        return "Active";
    }
}
