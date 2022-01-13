package frc.robot.team8583.devices;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team8583.Constants;
import frc.robot.team8583.Ports;
import frc.robot.team8583.drivers.NavX;
import frc.robot.team8583.drivers.Pigeon;

public class Ahrs
{
    private static Ahrs instance = null;

    public static synchronized Ahrs getInstance()
    {
        if (instance == null)
        {
            instance = new Ahrs();
        }
        return instance;
    }

    //private static Pigeon pigeon = new Pigeon(Ports.Can.PIGEON);
    private static NavX navx = new NavX();
    private static Rotation2d referenceHeading = Rotation2d.identity();

    private Ahrs()
    {
        initNavX();
        //initPegeon();
    }

    private void initNavX()
    {
        double initTime = Timer.getFPGATimestamp();
        while (!navx.isReady())
        {
            if (Timer.getFPGATimestamp() - initTime > Constants.Ahrs.INIT_TIMEOUT)
            {
                System.out.print("Warning: NavX initialization timed out with ");
                System.out.print(Timer.getFPGATimestamp() - initTime);
                System.out.println(" seconds");
                break;
            }
            Timer.delay(Constants.Looper.DELTA_TIME);
        }
        setRobotHeading(Rotation2d.fromDegrees(Constants.Robot.INIT_HEADING));
        Timer.delay(Constants.Ahrs.INIT_DELAY);
    }

    /*private void initPegeon()
    {
        double initTime = Timer.getFPGATimestamp();
        while (!pigeon.isReady())
        {
            if (Timer.getFPGATimestamp() - initTime > Constants.Ahrs.INIT_TIMEOUT)
            {
                System.out.print("Warning: Pigeon initialization timed out with ");
                System.out.print(Timer.getFPGATimestamp() - initTime);
                System.out.println(" seconds");
                break;
            }
            Timer.delay(Constants.Looper.DELTA_TIME);
        }
        pigeon.configStatusFramePeriod(Constants.Ahrs.STATUS_FRAME_PERIOD, false, Constants.Can.TIMEOUT);
        pigeon.configTemperatureCompensation(Constants.Ahrs.TEMPERATURE_COMPENSATION);
        setRobotHeading(Rotation2d.fromDegrees(Constants.Robot.INIT_HEADING));
        Timer.delay(Constants.Ahrs.INIT_DELAY);
    }*/

    public boolean isOperational()
    {
        return navx.isReady();
    }

    /*public boolean isOperational()
    {
        return pigeon.isReady();
    }*/

    public Rotation2d getRobotHeading()
    {
        return Rotation2d.fromDegrees(navx.getFusedYaw()).rotateBy(referenceHeading.inverse());
    }

    /*public Rotation2d getRobotHeading()
    {
        return Rotation2d.fromDegrees(pigeon.getFusedYaw()).rotateBy(referenceHeading.inverse());
    }*/

    public Rotation2d getRobotAngularVelocity()
    {
        return Rotation2d.fromDegrees(navx.getGyroZ());
    }

    /*public Rotation2d getRobotAngularVelocity()
    {
        return Rotation2d.fromDegrees(pigeon.getGyroZ());
    }*/

    public synchronized void setRobotHeading(Rotation2d heading)
    {
        referenceHeading = getRobotHeading().rotateBy(heading.inverse());
    }

    public synchronized void resetRobotHeading()
    {
        setRobotHeading(Rotation2d.identity());
    }
}
