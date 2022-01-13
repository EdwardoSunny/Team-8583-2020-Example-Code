package frc.robot.team8583.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team8583.Constants;
import frc.robot.team8583.Ports;
import frc.robot.team8583.drivers.LazyTalonSRX;
import frc.robot.team8583.subsystems.Subsystem;

public class BallFeeder extends Subsystem
{
    private class PeriodicOutput
    {
        public double motorOutput = 0.0;
    }

    private PeriodicOutput periodicOutput = new PeriodicOutput();
    private final LazyTalonSRX motor = new LazyTalonSRX(Ports.Can.BALL_FEEDER_MOTOR);

    public BallFeeder()
    {
        configMotor();
    }

    private synchronized void configMotor()
    {
        motor.configStatusFramePeriod(Constants.Shooter.BallFeeder.STATUS_FRAME_PERIOD, false, false,
                Constants.Can.TIMEOUT);
        motor.setInverted(Constants.Shooter.BallFeeder.INVERT_MOTOR);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configVoltageCompSaturation(Constants.Shooter.BallFeeder.SATURATION_VOLTAGE, Constants.Can.TIMEOUT);
        motor.configVoltageMeasurementFilter(Constants.Shooter.BallFeeder.VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE,
                Constants.Can.TIMEOUT);
        motor.enableVoltageCompensation(Constants.Shooter.BallFeeder.ENABLE_VOLTAGE_COMPENSATION);
        motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.Shooter.BallFeeder.CURRENT_LIMITER_TARGET_CURRENT,
                        Constants.Shooter.BallFeeder.CURRENT_LIMITER_THRESHOLD_CURRENT,
                        Constants.Shooter.BallFeeder.CURRENT_LIMITER_TRIGGER_DELAY),
                Constants.Can.TIMEOUT);
        Timer.delay(Constants.Can.MOTOR_CONTROLLER_CONFIG_DELAY);
    }

    private synchronized void setMotorOutput(double output)
    {
        periodicOutput.motorOutput = output;
    }

    public synchronized void setInject()
    {
        setMotorOutput(Constants.Shooter.BallFeeder.INJECT_OPEN_LOOP_OUTPUT);
    }

    public synchronized void setEject()
    {
        setMotorOutput(Constants.Shooter.BallFeeder.EJECT_OPEN_LOOP_OUTPUT);
    }

    public boolean isStucked()
    {
        double currentAmps = motor.getStatorCurrent();
        if (currentAmps > 60.0)
        {
            return true;
        }
        else
        {
            return false;
        }

    }

    @Override
    public synchronized void stop()
    {
        setMotorOutput(0.0);
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        motor.set(ControlMode.PercentOutput, periodicOutput.motorOutput);
    }

    @Override
    public void logToSmartDashboard()
    {
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putNumber("Shooter Ball Feeder Motor Voltage", motor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Shooter Ball Feeder Motor Current", motor.getSupplyCurrent());
            SmartDashboard.putNumber("Shooter Ball Feeder Motor Output Current", motor.getStatorCurrent());
        }
    }
}
