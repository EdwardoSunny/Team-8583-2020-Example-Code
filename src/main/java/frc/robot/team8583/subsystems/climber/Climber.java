package frc.robot.team8583.subsystems.climber;

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
import frc.robot.team8583.subsystems.climber.states.ClimberState;
import frc.robot.team8583.subsystems.climber.states.ClimberDisabled;
import frc.robot.team8583.subsystems.climber.states.ClimberInactive;
import frc.robot.team8583.loops.ILooper;

public class Climber extends Subsystem
{
    private static Climber instance = null;

    public static Climber getInstance()
    {
        if (instance == null)
        {
            instance = new Climber();
        }
        return instance;
    }

    private class PeriodicOutput
    {
        public double motorOutput = 0.0;
        public boolean solenoidPullOutput = false;
        public boolean solenoidPushOutput = false;
    }

    private ClimberState state;
    private PeriodicOutput periodicOutput = new PeriodicOutput();
    private final LazyTalonSRX motor = TalonSRXManager.getLazyTalonSRX(Ports.Can.CLIMBER_MOTOR);
    private final Solenoid solenoidPull = new Solenoid(Ports.Can.PCM, Ports.Pcm.CLIMBER_LOCK_PULL);
    private final Solenoid solenoidPush = new Solenoid(Ports.Can.PCM, Ports.Pcm.CLIMBER_LOCK_PUSH);

    private Climber()
    {
        configMotor();
        setState(new ClimberDisabled(this));
    }

    public synchronized void setState(ClimberState state)
    {
        this.state = state;
    }

    public ClimberState getState()
    {
        return state;
    }

    public synchronized void update(double timestamp)
    {
        state.update(timestamp);
    }

    private synchronized void configMotor()
    {
        motor.configStatusFramePeriod(Constants.Climber.STATUS_FRAME_PERIOD, false, false, Constants.Can.TIMEOUT);
        motor.setInverted(Constants.Climber.INVERT_MOTOR);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configVoltageCompSaturation(Constants.Climber.SATURATION_VOLTAGE, Constants.Can.TIMEOUT);
        motor.configVoltageMeasurementFilter(Constants.Climber.VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE,
                Constants.Can.TIMEOUT);
        motor.enableVoltageCompensation(Constants.Climber.ENABLE_VOLTAGE_COMPENSATION);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                Constants.Climber.CURRENT_LIMITER_TARGET_CURRENT, Constants.Climber.CURRENT_LIMITER_THRESHOLD_CURRENT,
                Constants.Climber.CURRENT_LIMITER_TRIGGER_DELAY), Constants.Can.TIMEOUT);
        Timer.delay(Constants.Can.MOTOR_CONTROLLER_CONFIG_DELAY);
    }

    private synchronized void setMotorOutput(double output)
    {
        periodicOutput.motorOutput = output;
    }

    public synchronized void setExtendClimber()
    {
        setMotorOutput(Constants.Climber.CLIMBER_EXTEND_OPEN_LOOP_OUTPUT);
    }

    public synchronized void setRetractClimber()
    {
        setMotorOutput(-Constants.Climber.CLIMBER_RETRACT_OPEN_LOOP_OUTPUT);
    }

    public synchronized void setLock(boolean lock)
    {
        periodicOutput.solenoidPullOutput = lock;
        periodicOutput.solenoidPushOutput = !lock;
    }

    public synchronized void releaseLock()
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
        setState(new ClimberInactive(this));
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
        SmartDashboard.putString("Climber State", state.toString());
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putNumber("Climber Motor Voltage", motor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Climber Motor Current", motor.getSupplyCurrent());
            SmartDashboard.putBoolean("Climber Pull Solenoid Output", solenoidPull.get());
            SmartDashboard.putBoolean("Climber Push Solenoid Output", solenoidPull.get());
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        Climber thisInstance = this;
        Loop loop = new Loop()
        {
            @Override
            public void onStart(double timestamp)
            {
                setState(new ClimberInactive(thisInstance));
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
