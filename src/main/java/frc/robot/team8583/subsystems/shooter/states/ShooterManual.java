package frc.robot.team8583.subsystems.shooter.states;

import frc.robot.team8583.devices.DriverInterface;
import frc.robot.team8583.drivers.StatefulXboxController;
import frc.robot.team8583.drivers.StatefulXboxController.ButtonId;
import frc.robot.team8583.subsystems.shooter.Shooter;

public class ShooterManual extends ShooterState
{
    private final StatefulXboxController codriverController = DriverInterface.getCodriverController();

    public ShooterManual(Shooter shooter)
    {
        super(shooter);
        shooter.stopTurret();
        shooter.stopBallDrive();
        shooter.stopBallFeeder();
    }

    @Override
    public synchronized void update()
    {
        shooter.setLaunch(codriverController.getButton(ButtonId.TRIGGER_RIGHT).isBeingPressed());

        if (codriverController.getButton(ButtonId.BUTTON_B).wasPressed())
        {
            shooter.toggleSpindex();
        }

        if (codriverController.getButton(ButtonId.BUTTON_Y).isBeingPressed())
        {
            shooter.enableSpindexAntiJam();
        }
        else
        {
            shooter.disableSpindexAntiJam();
        }

        if (codriverController.getButton(ButtonId.TRIGGER_LEFT).isBeingPressed())
        {
            shooter.setBallDriveReverse();
            shooter.setBallFeederEject();
        }
    }

    @Override
    public String toString()
    {
        return "Manual";
    }
}
