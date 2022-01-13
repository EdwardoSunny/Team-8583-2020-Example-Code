package frc.robot.team8583.subsystems.climber.states;

import frc.robot.team8583.devices.DriverInterface;
import frc.robot.team8583.drivers.StatefulXboxController;
import frc.robot.team8583.drivers.StatefulXboxController.ButtonId;
import frc.robot.team8583.subsystems.climber.Climber;

public class ClimberInactive extends ClimberState
{
    private final StatefulXboxController driverController = DriverInterface.getDriverController();

    public ClimberInactive(Climber climber)
    {
        super(climber);
        climber.stop();
        climber.setLock(true);
    }

    @Override
    public void update(double timestamp)
    {
        if (driverController.getButton(ButtonId.BUMPER_LEFT).isBeingPressed()
                && driverController.getButton(ButtonId.BUMPER_RIGHT).isBeingPressed())
        {
            climber.setState(new ClimberManual(climber));
            return;
        }
    }

    @Override
    public String toString()
    {
        return "Inactive";
    }
}
