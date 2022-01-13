package frc.robot.team8583.subsystems;

import java.util.List;

import frc.robot.team8583.loops.ILooper;
import frc.robot.team8583.loops.Loop;
import frc.robot.team8583.loops.Looper;

public class SubsystemGroup extends Subsystem
{
    private final List<Subsystem> subsystems;

    public SubsystemGroup(List<Subsystem> subsystems)
    {
        this.subsystems = subsystems;
    }

    public List<Subsystem> getSubsystems()
    {
        return subsystems;
    }

    @Override
    public void stop()
    {
        subsystems.forEach((subsystem) -> subsystem.stop());
    }

    @Override
    public void disable()
    {
        subsystems.forEach((subsystem) -> subsystem.disable());
    }

    @Override
    public void readPeriodicInputs()
    {
        subsystems.forEach((subsystem) -> subsystem.readPeriodicInputs());
    }

    @Override
    public void writePeriodicOutputs()
    {
        subsystems.forEach((subsystem) -> subsystem.writePeriodicOutputs());
    }

    @Override
    public void resetSensors()
    {
        subsystems.forEach((subsystem) -> subsystem.resetSensors());
    }

    @Override
    public void logToSmartDashboard()
    {
        subsystems.forEach((subsystem) -> subsystem.logToSmartDashboard());
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        Looper subsystemEnabledLooper = new Looper();
        subsystems.forEach((subsystem) -> subsystem.registerEnabledLoops(subsystemEnabledLooper));
        Loop loop = new Loop()
        {
            @Override
            public void onStart(double timestamp)
            {
                subsystemEnabledLooper.allOnStart(timestamp);
            }

            @Override
            public void onLoop(double timestamp)
            {
                for (Subsystem subsystem : subsystems)
                {
                    subsystem.readPeriodicInputs();
                }
                subsystemEnabledLooper.allOnLoop(timestamp);
                for (Subsystem subsystem : subsystems)
                {
                    subsystem.writePeriodicOutputs();
                }
            }

            @Override
            public void onStop(double timestamp)
            {
                subsystemEnabledLooper.allOnStop(timestamp);
            }
        };
        enabledLooper.register(loop);
    }

    @Override
    public void registerDisabledLoops(ILooper enabledLooper)
    {
        Looper subsystemDisabledLooper = new Looper();
        subsystems.forEach((subsystem) -> subsystem.registerDisabledLoops(subsystemDisabledLooper));
        Loop loop = new Loop()
        {
            @Override
            public void onStart(double timestamp)
            {
                subsystemDisabledLooper.allOnStart(timestamp);
            }

            @Override
            public void onLoop(double timestamp)
            {
                for (Subsystem subsystem : subsystems)
                {
                    subsystem.readPeriodicInputs();
                }
                subsystemDisabledLooper.allOnLoop(timestamp);
                for (Subsystem subsystem : subsystems)
                {
                    subsystem.writePeriodicOutputs();
                }
            }

            @Override
            public void onStop(double timestamp)
            {
                subsystemDisabledLooper.allOnStop(timestamp);
            }
        };
        enabledLooper.register(loop);
    }

    @Override
    public boolean selfTest()
    {
        boolean passesTest = true;
        for (Subsystem subsystem : subsystems)
        {
            passesTest &= subsystem.selfTest();
        }
        return passesTest;
    }
}
