package frc.robot.team8583.devices;

import frc.robot.team8583.Constants;
import frc.robot.team8583.drivers.StatefulXboxController;

public class DriverInterface
{
    private static StatefulXboxController driverController = null;
    private static StatefulXboxController codriverController = null;

    public static synchronized StatefulXboxController getDriverController()
    {
        if (driverController == null)
        {
            driverController = new StatefulXboxController(Constants.DriverInterface.DRIVER_CONTROLLER_PORT,
                    Constants.DriverInterface.TRIGGER_PRESS_THRESHOLD);
        }
        return driverController;
    }

    public static synchronized StatefulXboxController getCodriverController()
    {
        if (codriverController == null)
        {
            codriverController = new StatefulXboxController(Constants.DriverInterface.CODRIVER_CONTROLLER_PORT,
                    Constants.DriverInterface.TRIGGER_PRESS_THRESHOLD);
        }
        return codriverController;
    }

    public static synchronized void update()
    {
        getDriverController().updateButtons();
        getCodriverController().updateButtons();
    }
}
