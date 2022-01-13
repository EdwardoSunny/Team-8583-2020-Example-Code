package frc.robot.team8583.subsystems.climber.states;

import frc.robot.team8583.devices.DriverInterface;
import frc.robot.team8583.drivers.StatefulXboxController;
import frc.robot.team8583.drivers.StatefulXboxController.ButtonId;
import frc.robot.team8583.subsystems.climber.Climber;

public class ClimberManual extends ClimberState
{
    private final StatefulXboxController driverController = DriverInterface.getDriverController();

    public ClimberManual(Climber climber)
    {
        super(climber);
        climber.setLock(false);
    }

    @Override
    public void update(double timestamp)
    {
        if (driverController.getButton(ButtonId.TRIGGER_RIGHT).isBeingPressed())
        {
            climber.setRetractClimber();
        }
        else if (driverController.getButton(ButtonId.BUTTON_X).isBeingPressed()
                && driverController.getButton(ButtonId.BUTTON_Y).isBeingPressed())
        {
            climber.setExtendClimber();
        }
        else
        {
            climber.stop();
        }
    }

    @Override
    public String toString()
    {
        return "Manual";
    }
}
