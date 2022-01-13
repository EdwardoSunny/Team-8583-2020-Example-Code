package frc.robot.team8583.subsystems.collector;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team8583.Constants;
import frc.robot.team8583.Ports;
import frc.robot.team8583.devices.TalonSRXManager;
import frc.robot.team8583.drivers.LazyTalonSRX;
import frc.robot.team8583.loops.Loop;
import frc.robot.team8583.subsystems.Subsystem;
import frc.robot.team8583.subsystems.collector.states.CollectorState;
import frc.robot.team8583.subsystems.collector.states.CollectorDisabled;
import frc.robot.team8583.subsystems.collector.states.CollectorManual;
import frc.robot.team8583.loops.ILooper;

public class Collector extends Subsystem
{
    private static Collector instance = null;

    public static Collector getInstance()
    {
        if (instance == null)
        {
            instance = new Collector();
        }
        return instance;
    }

    private class PeriodicOutput
    {
        public double motorOutput = 0.0;
        public boolean solenoidPullOutput = false;
        public boolean solenoidPushOutput = false;
    }

    private CollectorState state;
    private PeriodicOutput periodicOutput = new PeriodicOutput();
    private final LazyTalonSRX motor = TalonSRXManager.getLazyTalonSRX(Ports.Can.COLLECTOR_MOTOR);
    private final Solenoid solenoidPull = new Solenoid(Ports.Can.PCM, Ports.Pcm.COLLECTOR_ELEVATE_PULL);
    private final Solenoid solenoidPush = new Solenoid(Ports.Can.PCM, Ports.Pcm.COLLECTOR_ELEVATE_PUSH);

    private Collector()
    {
        configMotor();
        setState(new CollectorDisabled(this));
    }

    public synchronized void setState(CollectorState state)
    {
        this.state = state;
    }

    public CollectorState getState()
    {
        return state;
    }

    public synchronized void update(double timestamp)
    {
        state.update(timestamp);
    }

    private synchronized void configMotor()
    {
        motor.configStatusFramePeriod(Constants.Collector.STATUS_FRAME_PERIOD, false, false, Constants.Can.TIMEOUT);
        motor.setInverted(Constants.Collector.INVERT_MOTOR);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configVoltageCompSaturation(Constants.Collector.SATURATION_VOLTAGE, Constants.Can.TIMEOUT);
        motor.configVoltageMeasurementFilter(Constants.Collector.VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE,
                Constants.Can.TIMEOUT);
        motor.enableVoltageCompensation(Constants.Collector.ENABLE_VOLTAGE_COMPENSATION);
        motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.Collector.CURRENT_LIMITER_TARGET_CURRENT,
                        Constants.Collector.CURRENT_LIMITER_THRESHOLD_CURRENT,
                        Constants.Collector.CURRENT_LIMITER_TRIGGER_DELAY),
                Constants.Can.TIMEOUT);
        Timer.delay(Constants.Can.MOTOR_CONTROLLER_CONFIG_DELAY);
    }

    private synchronized void setMotorOutput(double output)
    {
        periodicOutput.motorOutput = output;
    }

    public synchronized void setInject()
    {
        setMotorOutput(Constants.Collector.INJECT_OPEN_LOOP_OUTPUT);
    }

    public synchronized void setEject()
    {
        setMotorOutput(-Constants.Collector.EJECT_OPEN_LOOP_OUTPUT);
    }

    public synchronized void setElevate(boolean elevate)
    {
        periodicOutput.solenoidPullOutput = elevate;
        periodicOutput.solenoidPushOutput = !elevate;
    }

    public synchronized void releaseElevation()
    {
        periodicOutput.solenoidPullOutput = false;
        periodicOutput.solenoidPushOutput = false;
    }

    @Override
    public synchronized void stop()
    {
        setMotorOutput(0.0);
    }

    @Override
    public synchronized void disable()
    {
        setState(new CollectorDisabled(this));
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        motor.set(ControlMode.PercentOutput, periodicOutput.motorOutput);
        solenoidPull.set(periodicOutput.solenoidPullOutput);
        solenoidPush.set(periodicOutput.solenoidPushOutput);
    }

    @Override
    public synchronized void logToSmartDashboard()
    {
        SmartDashboard.putString("Collector State", state.toString());
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putNumber("Collector Motor Voltage", motor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Collector Motor Current", motor.getSupplyCurrent());
            SmartDashboard.putBoolean("Collector Pull Solenoid Output", solenoidPull.get());
            SmartDashboard.putBoolean("Collector Push Solenoid Output", solenoidPull.get());
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        Collector thisInstance = this;
        Loop loop = new Loop()
        {
            @Override
            public void onStart(double timestamp)
            {
                setState(new CollectorManual(thisInstance));
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
