package frc.robot.team8583.subsystems.shooter.turret.states;

import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team8583.devices.DriverInterface;
import frc.robot.team8583.drivers.StatefulXboxController;
import frc.robot.team8583.drivers.StatefulXboxController.ButtonId;
import frc.robot.team8583.subsystems.shooter.turret.Turret;

public class TurretInactive extends TurretState
{
    private StatefulXboxController codriverController = DriverInterface.getCodriverController();

    public TurretInactive(Turret turret)
    {
        super(turret);
        turret.disableVision();
        turret.stopYawGimbal();
        turret.stopPitchGimbal();
    }

    @Override
    public synchronized void update()
    {
        if (codriverController.getButton(ButtonId.BUTTON_A).wasPressed())
        {
            turret.setState(new TurretInitializing(turret, Rotation2d.fromDegrees(180.0), false));
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
        return "Inactive";
    }
}
