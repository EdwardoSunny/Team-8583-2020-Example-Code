package frc.robot.team8583.subsystems.collector.states;

import frc.robot.team8583.Constants;
import frc.robot.team8583.devices.DriverInterface;
import frc.robot.team8583.drivers.StatefulXboxController;
import frc.robot.team8583.drivers.StatefulXboxController.ButtonId;
import frc.robot.team8583.subsystems.collector.Collector;

public class CollectorManual extends CollectorState
{
    private boolean isElevated = Constants.Collector.INITIAL_ELEVATION;
    private final StatefulXboxController codriverController = DriverInterface.getCodriverController();

    public CollectorManual(Collector collector)
    {
        super(collector);
        collector.setElevate(isElevated);
    }

    @Override
    public void update(double timestamp)
    {
        if (codriverController.getButton(ButtonId.BUMPER_LEFT).isBeingPressed())
        {
            collector.setInject();
        }
        else if (codriverController.getButton(ButtonId.BUMPER_RIGHT).isBeingPressed())
        {
            collector.setEject();
        }
        else
        {
            collector.stop();
        }

        if (codriverController.getButton(ButtonId.BUTTON_X).wasPressed())
        {
            isElevated = !isElevated;
            collector.setElevate(isElevated);
        }
    }

    @Override
    public String toString()
    {
        return "Manual";
    }
}
