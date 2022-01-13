package frc.robot.team8583.subsystems;

import frc.robot.team8583.loops.ILooper;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * instantializing all member components at the start of the match.
 */
public abstract class Subsystem
{
    public void stop()
    {
    }

    public void disable()
    {
        stop();
    }

    public void readPeriodicInputs()
    {
    }

    public void writePeriodicOutputs()
    {
    }

    public void resetSensors()
    {
    }

    public void logToSmartDashboard()
    {
    }

    public void registerEnabledLoops(ILooper enabledLooper)
    {
    }

    public void registerDisabledLoops(ILooper disabledLooper)
    {
    }

    public boolean selfTest()
    {
        return true;
    }
}