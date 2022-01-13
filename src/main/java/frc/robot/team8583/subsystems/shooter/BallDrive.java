package frc.robot.team8583.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team8583.Constants;
import frc.robot.team8583.Ports;
import frc.robot.team8583.drivers.LazyTalonFX;
import frc.robot.team8583.lib.util.Util;
import frc.robot.team8583.subsystems.Subsystem;

public class BallDrive extends Subsystem
{
    private class PeriodicInput
    {
        public int leftMotorEncoderVelocity = 0;
        public int rightMotorEncoderVelocity = 0;
    }

    private class PeriodicOutput
    {
        public ControlMode motorControlMode = ControlMode.PercentOutput;
        public double motorSetpoint = 0;
    }

    private final LazyTalonFX leftMotor = new LazyTalonFX(Ports.Can.BALL_DRIVE_LEFT_MOTOR);
    private final LazyTalonFX rightMotor = new LazyTalonFX(Ports.Can.BALL_DRIVE_RIGHT_MOTOR);
    private final PeriodicInput periodicInput = new PeriodicInput();
    private final PeriodicOutput periodicOutput = new PeriodicOutput();

    public BallDrive()
    {
        configMotors();
    }

    private synchronized void configMotors()
    {
        configMotor(leftMotor, Constants.Shooter.BallDrive.INVERT_LEFT_MOTOR);
        configMotor(rightMotor, Constants.Shooter.BallDrive.INVERT_RIGHT_MOTOR);
    }

    private synchronized void configMotor(LazyTalonFX motor, boolean invert)
    {
        motor.configStatusFramePeriod(Constants.Shooter.BallDrive.STATUS_FRAME_PERIOD, true, false,
                Constants.Can.TIMEOUT);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.Can.TIMEOUT);
        motor.setInverted(invert);
        motor.configVelocityMeasurementPeriod(Constants.Shooter.BallDrive.SPEED_MEASUREMENT_PERIOD,
                Constants.Can.TIMEOUT);
        motor.configVelocityMeasurementWindow(Constants.Shooter.BallDrive.SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE,
                Constants.Can.TIMEOUT);
        motor.configVoltageCompSaturation(Constants.Shooter.BallDrive.SATURATION_VOLTAGE, Constants.Can.TIMEOUT);
        motor.configVoltageMeasurementFilter(Constants.Shooter.BallDrive.VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE,
                Constants.Can.TIMEOUT);
        motor.enableVoltageCompensation(Constants.Shooter.BallDrive.ENABLE_VOLTAGE_COMPENSATION);
        motor.configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, Constants.Shooter.BallDrive.CURRENT_LIMITER_TARGET_CURRENT,
                        Constants.Shooter.BallDrive.CURRENT_LIMITER_THRESHOLD_CURRENT,
                        Constants.Shooter.BallDrive.CURRENT_LIMITER_TRIGGER_DELAY),
                Constants.Can.TIMEOUT);
        motor.configAllowableClosedloopError(0, Constants.Shooter.BallDrive.CONTROL_ERROR_TOLERANCE,
                Constants.Can.TIMEOUT);
        motor.config_kP(0, Constants.Shooter.BallDrive.PID0_KP, Constants.Can.TIMEOUT);
        motor.config_kI(0, Constants.Shooter.BallDrive.PID0_KI, Constants.Can.TIMEOUT);
        motor.config_kD(0, Constants.Shooter.BallDrive.PID0_KD, Constants.Can.TIMEOUT);
        motor.config_kF(0, Constants.Shooter.BallDrive.PID0_KF, Constants.Can.TIMEOUT);
        Timer.delay(Constants.Can.MOTOR_CONTROLLER_CONFIG_DELAY);
    }

    private double encoderVelocityToInchesPerSecond(int encoderUnitsPer100ms)
    {
        return encoderUnitsPer100ms / Constants.Shooter.BallDrive.ENCODER_UNITS_PER_INCH * 10.0;
    }

    private int inchesPerSecondToEncoderVelocity(double inchesPerSecond)
    {
        return Util.roundToInt(inchesPerSecond / 10.0 * Constants.Shooter.BallDrive.ENCODER_UNITS_PER_INCH);
    }

    private int getMotorEncoderAverageVelocity()
    {
        return Util
                .roundToInt((periodicInput.leftMotorEncoderVelocity + periodicInput.rightMotorEncoderVelocity) * 0.5);
    }

    public double getFlywheelSurfaceSpeed()
    {
        return encoderVelocityToInchesPerSecond(getMotorEncoderAverageVelocity());
    }

    public synchronized void setFlywheelSurfaceSpeedTarget(double speedInchesPerSecond)
    {
        periodicOutput.motorControlMode = ControlMode.Velocity;
        periodicOutput.motorSetpoint = inchesPerSecondToEncoderVelocity(speedInchesPerSecond);
    }

    public synchronized void setFlywheelSurfaceSpeedTargetByProjectileSpeed(double projectileSpeedInchesPerSecond)
    {
        setFlywheelSurfaceSpeedTarget(
                projectileSpeedInchesPerSecond / Constants.Shooter.BallDrive.PROJECTILE_SPEED_FACTOR);
    }

    public synchronized void setNormalizedFlywheelSurfaceSpeedTarget(double normalizedSpeed)
    {
        periodicOutput.motorControlMode = ControlMode.Velocity;
        periodicOutput.motorSetpoint = Constants.Shooter.BallDrive.MAX_CRUISE_SPEED * normalizedSpeed;
    }

    public synchronized void setFlywheelOpenLoop(double output)
    {
        periodicOutput.motorControlMode = ControlMode.PercentOutput;
        periodicOutput.motorSetpoint = output;
    }

    public synchronized void setFlywheelReverse()
    {
        setFlywheelOpenLoop(-Constants.Shooter.BallDrive.REVERSE_OPEN_LOOP_OUTPUT);
    }

    public boolean flywheelSurfaceSpeedOnTarget()
    {
        if (periodicOutput.motorControlMode == ControlMode.Velocity)
        {
            return Util.epsilonEquals(periodicOutput.motorSetpoint, getMotorEncoderAverageVelocity(),
                    inchesPerSecondToEncoderVelocity(
                            Constants.Shooter.BallDrive.SURFACE_SPEED_ON_TARGET_ERROR_TOLERANCE));
        }
        return false;
    }

    @Override
    public synchronized void stop()
    {
        setFlywheelOpenLoop(0.0);
    }

    @Override
    public synchronized void readPeriodicInputs()
    {
        periodicInput.leftMotorEncoderVelocity = leftMotor.getSelectedSensorVelocity();
        periodicInput.rightMotorEncoderVelocity = rightMotor.getSelectedSensorVelocity();
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        leftMotor.set(periodicOutput.motorControlMode, periodicOutput.motorSetpoint);
        rightMotor.set(periodicOutput.motorControlMode, periodicOutput.motorSetpoint);
    }

    @Override
    public void logToSmartDashboard()
    {
        SmartDashboard.putNumber("Shooter Ball Drive Flywheel Surface Speed", getFlywheelSurfaceSpeed());
        SmartDashboard.putBoolean("Shooter Ball Drive Flywheel Surface Speed On Target",
                flywheelSurfaceSpeedOnTarget());
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putNumber("Shooter Ball Drive Left Motor Encoder Velocity",
                    leftMotor.getSelectedSensorVelocity());
            SmartDashboard.putNumber("Shooter Ball Drive Right Motor Encoder Velocity",
                    rightMotor.getSelectedSensorVelocity());
            SmartDashboard.putNumber("Shooter Ball Drive Motor Encoder Velocity Difference",
                    leftMotor.getSelectedSensorVelocity() - rightMotor.getSelectedSensorVelocity());

            SmartDashboard.putNumber("Shooter Ball Drive Left Motor Current", leftMotor.getStatorCurrent());
            SmartDashboard.putNumber("Shooter Ball Drive Right Motor Current", rightMotor.getStatorCurrent());
        }
    }
}
