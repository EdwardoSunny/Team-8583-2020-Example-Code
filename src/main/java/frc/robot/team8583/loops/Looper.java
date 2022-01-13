package frc.robot.team8583.loops;

import java.util.ArrayList;
import java.util.List;

import frc.robot.team8583.Constants;
import frc.robot.team8583.lib.util.CrashTrackingRunnable;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper
{
    private final double period = Constants.Looper.DELTA_TIME;
    private final Notifier notifier;
    private final List<Loop> loops;
    private final Object taskRunningLock = new Object();
    private boolean isRunning;
    private double timestamp = 0;
    private double deltaTime = 0;

    private final CrashTrackingRunnable crashTrackingRunnable = new CrashTrackingRunnable()
    {
        @Override
        public void runCrashTracked()
        {
            synchronized (taskRunningLock)
            {
                if (isRunning)
                {
                    double currentTimestamp = Timer.getFPGATimestamp();
                    allOnLoop(currentTimestamp);
                    deltaTime = currentTimestamp - timestamp;
                    timestamp = currentTimestamp;
                }
            }
        }
    };

    public Looper()
    {
        notifier = new Notifier(crashTrackingRunnable);
        isRunning = false;
        loops = new ArrayList<>();
    }

    public double getDeltaTime()
    {
        return deltaTime;
    }

    @Override
    public synchronized void register(Loop loop)
    {
        synchronized (taskRunningLock)
        {
            loops.add(loop);
        }
    }

    public synchronized void start()
    {
        if (!isRunning)
        {
            synchronized (taskRunningLock)
            {
                allOnStart(Timer.getFPGATimestamp());
                isRunning = true;
            }
            notifier.startPeriodic(period);
        }
    }

    public synchronized void restart()
    {
        if (isRunning)
        {
            stop();
        }
        start();
    }

    public synchronized void stop()
    {
        if (isRunning)
        {
            notifier.stop();
            synchronized (taskRunningLock)
            {
                isRunning = false;
                allOnStop(Timer.getFPGATimestamp());
            }
        }
    }

    public void allOnStart(double timestamp)
    {
        for (Loop loop : loops)
        {
            loop.onStart(timestamp);
        }
    }

    public void allOnLoop(double timestamp)
    {
        for (Loop loop : loops)
        {
            loop.onLoop(timestamp);
        }
    }

    public void allOnStop(double timestamp)
    {
        for (Loop loop : loops)
        {
            loop.onStop(timestamp);
        }
    }

    public void logToSmartDashboard()
    {
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putNumber("Looper Delta Time", deltaTime);
        }
    }
}
