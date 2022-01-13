package frc.robot.team8583.subsystems.shooter.spindex;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team8583.Constants;
import frc.robot.team8583.Ports;
import frc.robot.team8583.drivers.LazyTalonSRX;
import frc.robot.team8583.lib.util.Util;
import frc.robot.team8583.loops.ILooper;
import frc.robot.team8583.loops.Loop;
import frc.robot.team8583.subsystems.Subsystem;
import frc.robot.team8583.subsystems.shooter.spindex.states.SpindexInactive;
import frc.robot.team8583.subsystems.shooter.spindex.states.SpindexState;

public class Spindex extends Subsystem
{
    private class PeriodicInput
    {
        public double motorStatorCurrent = 0.0;
    }

    private class PeriodicOutput
    {
        public double motorOutput = 0.0;
    }

    private SpindexState state;
    private final PeriodicInput periodicInput = new PeriodicInput();
    private final PeriodicOutput periodicOutput = new PeriodicOutput();
    private final LazyTalonSRX motor = new LazyTalonSRX(Ports.Can.SPINDEX_MOTOR);

    public Spindex()
    {
        configMotor();
        setState(new SpindexInactive(this));
    }

    private synchronized void configMotor()
    {
        motor.configStatusFramePeriod(Constants.Shooter.Spindex.STATUS_FRAME_PERIOD, false, false,
                Constants.Can.TIMEOUT);
        motor.setInverted(Constants.Shooter.Spindex.INVERT_MOTOR);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.configVoltageCompSaturation(Constants.Shooter.Spindex.SATURATION_VOLTAGE, Constants.Can.TIMEOUT);
        motor.configVoltageMeasurementFilter(Constants.Shooter.Spindex.VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE,
                Constants.Can.TIMEOUT);
        motor.enableVoltageCompensation(Constants.Shooter.Spindex.ENABLE_VOLTAGE_COMPENSATION);
        motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.Shooter.Spindex.CURRENT_LIMITER_TARGET_CURRENT,
                        Constants.Shooter.Spindex.CURRENT_LIMITER_THRESHOLD_CURRENT,
                        Constants.Shooter.Spindex.CURRENT_LIMITER_TRIGGER_DELAY),
                Constants.Can.TIMEOUT);
        Timer.delay(Constants.Can.MOTOR_CONTROLLER_CONFIG_DELAY);
    }

    public synchronized void setState(SpindexState state)
    {
        this.state = state;
    }

    public SpindexState getState()
    {
        return state;
    }

    public synchronized void update(double timestamp)
    {
        state.update(timestamp);
    }

    private synchronized void setMotorOutput(double output)
    {
        periodicOutput.motorOutput = output;
    }

    public synchronized void stopMotor()
    {
        setMotorOutput(0.0);
    }

    public synchronized void setSpin(boolean invert, boolean fast, boolean antiJam)
    {
        double output = Constants.Shooter.Spindex.SLOW_SPIN_OPEN_LOOP_OUTPUT;
        if (antiJam)
        {
            output = Constants.Shooter.Spindex.ANTI_JAM_OPEN_LOOP_OUTPUT;
        }
        else if (fast)
        {
            output = Constants.Shooter.Spindex.FAST_SPIN_OPEN_LOOP_OUTPUT;
        }

        setMotorOutput(Util.conditionalInvert(output, invert ^ antiJam));
    }

    public synchronized void startSpin()
    {
        state.start();
    }

    public synchronized void toggleSpin()
    {
        state.toggle();
    }

    public synchronized void accelerateSpin()
    {
        state.accelerate();
    }

    public synchronized void decelerateSpin()
    {
        state.decelerte();
    }

    public synchronized void enableAntiJam()
    {
        state.enableAntiJam();
    }

    public synchronized void disableAntiJam()
    {
        state.disableAntiJam();
    }

    public boolean shouldAutoReverse(double thresholdCurrent)
    {
        return Math.abs(periodicInput.motorStatorCurrent) > thresholdCurrent;
    }

    @Override
    public synchronized void stop()
    {
        setState(new SpindexInactive(this));
    }

    @Override
    public synchronized void readPeriodicInputs()
    {
        periodicInput.motorStatorCurrent = motor.getStatorCurrent();
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        motor.set(ControlMode.PercentOutput, periodicOutput.motorOutput);
    }

    @Override
    public void logToSmartDashboard()
    {
        SmartDashboard.putString("Shooter Spindex State", state.toString());
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putNumber("Shooter Spindex Motor Voltage", motor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Shooter Spindex Motor Current", motor.getSupplyCurrent());
            SmartDashboard.putNumber("Shooter Spindex Motor Stator Current", motor.getStatorCurrent());
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        Spindex thisInstance = this;
        Loop loop = new Loop()
        {
            @Override
            public void onStart(double timestamp)
            {
                setState(new SpindexInactive(thisInstance));
            }

            @Override
            public void onLoop(double timestamp)
            {
                update(timestamp);
            }

            @Override
            public void onStop(double timestamp)
            {
                stop();
            }
        };
        enabledLooper.register(loop);
    }
}
