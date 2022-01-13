package frc.robot.team8583.subsystems.shooter.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team8583.Constants;
import frc.robot.team8583.Ports;
import frc.robot.team8583.drivers.LazyTalonSRX;
import frc.robot.team8583.lib.util.Util;
import frc.robot.team8583.subsystems.Subsystem;

public class PitchGimbal extends Subsystem
{
    private class PeriodicInput
    {
        public int motorEncoderPosition = 0;
    }

    private class PeriodicOutput
    {
        public ControlMode motorControlMode = ControlMode.PercentOutput;
        public double motorSetpoint = 0.0;
    }

    private final PeriodicInput periodicInput = new PeriodicInput();
    private final PeriodicOutput periodicOutput = new PeriodicOutput();
    private final LazyTalonSRX motor = new LazyTalonSRX(Ports.Can.TURRET_PITCH_MOTOR);

    public PitchGimbal()
    {
        configMotor();
    }

    private synchronized void configMotor()
    {
        motor.configStatusFramePeriod(Constants.Shooter.Turret.PitchGimbal.STATUS_FRAME_PERIOD, true, false,
                Constants.Can.TIMEOUT);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, Constants.Can.TIMEOUT);
        motor.setInverted(Constants.Shooter.Turret.PitchGimbal.INVERT_MOTOR);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setSensorPhase(Constants.Shooter.Turret.PitchGimbal.INVERT_SENSOR);
        motor.configForwardSoftLimitThreshold(Constants.Shooter.Turret.PitchGimbal.ENCODER_POSITION_SOFT_LIMIT_HIGH,
                Constants.Can.TIMEOUT);
        motor.configForwardSoftLimitEnable(true, Constants.Can.TIMEOUT);
        motor.configReverseSoftLimitThreshold(Constants.Shooter.Turret.PitchGimbal.ENCODER_POSITION_SOFT_LIMIT_LOW,
                Constants.Can.TIMEOUT);
        motor.configReverseSoftLimitEnable(true, Constants.Can.TIMEOUT);
        motor.configVelocityMeasurementPeriod(Constants.Shooter.Turret.PitchGimbal.SPEED_MEASUREMENT_PERIOD,
                Constants.Can.TIMEOUT);
        motor.configVelocityMeasurementWindow(Constants.Shooter.Turret.PitchGimbal.SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE,
                Constants.Can.TIMEOUT);
        motor.configMotionAcceleration(Util.roundToInt(Constants.Shooter.Turret.PitchGimbal.MAX_ACCELERATION),
                Constants.Can.TIMEOUT);
        motor.configMotionCruiseVelocity(Util.roundToInt(Constants.Shooter.Turret.PitchGimbal.MAX_CRUISE_SPEED),
                Constants.Can.TIMEOUT);
        motor.configVoltageCompSaturation(Constants.Shooter.Turret.PitchGimbal.SATURATION_VOLTAGE,
                Constants.Can.TIMEOUT);
        motor.configVoltageMeasurementFilter(
                Constants.Shooter.Turret.PitchGimbal.VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE, Constants.Can.TIMEOUT);
        motor.enableVoltageCompensation(Constants.Shooter.Turret.PitchGimbal.ENABLE_VOLTAGE_COMPENSATION);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                Constants.Shooter.Turret.PitchGimbal.CURRENT_LIMITER_TARGET_CURRENT,
                Constants.Shooter.Turret.PitchGimbal.CURRENT_LIMITER_THRESHOLD_CURRENT,
                Constants.Shooter.Turret.PitchGimbal.CURRENT_LIMITER_TRIGGER_DELAY), Constants.Can.TIMEOUT);
        motor.configAllowableClosedloopError(0, Constants.Shooter.Turret.PitchGimbal.CONTROL_ERROR_TOLERANCE,
                Constants.Can.TIMEOUT);
        motor.config_kP(0, Constants.Shooter.Turret.PitchGimbal.PID0_KP, Constants.Can.TIMEOUT);
        motor.config_kI(0, Constants.Shooter.Turret.PitchGimbal.PID0_KI, Constants.Can.TIMEOUT);
        motor.config_kD(0, Constants.Shooter.Turret.PitchGimbal.PID0_KD, Constants.Can.TIMEOUT);
        motor.config_kF(0, 1023.0 / Constants.Shooter.Turret.PitchGimbal.MAX_SPEED, Constants.Can.TIMEOUT);
        motor.setSelectedSensorPosition(0, 0, Constants.Can.TIMEOUT);
        Timer.delay(Constants.Can.MOTOR_CONTROLLER_CONFIG_DELAY);
    }

    private double encoderUnitsToDegrees(double encoderUnits)
    {
        return encoderUnits / Constants.Shooter.Turret.PitchGimbal.ENCODER_UNITS_PER_DEGREE
                + Constants.Shooter.Turret.PitchGimbal.LAUNCH_ANGLE_OFFSET;
    }

    private int degreesToEncoderUnits(double degrees)
    {
        return Util.roundToInt((degrees - Constants.Shooter.Turret.PitchGimbal.LAUNCH_ANGLE_OFFSET)
                * Constants.Shooter.Turret.PitchGimbal.ENCODER_UNITS_PER_DEGREE);
    }

    private double getRawHeading()
    {
        return encoderUnitsToDegrees(periodicInput.motorEncoderPosition);
    }

    public Rotation2d getHeading()
    {
        return Rotation2d.fromDegrees(getRawHeading());
    }

    private synchronized void setHeadingTarget(double headingDegrees)
    {
        periodicOutput.motorControlMode = ControlMode.MotionMagic;
        periodicOutput.motorSetpoint = Util.limit(degreesToEncoderUnits(headingDegrees),
                Constants.Shooter.Turret.PitchGimbal.ENCODER_POSITION_OPERATIONAL_LOW,
                Constants.Shooter.Turret.PitchGimbal.ENCODER_POSITION_OPERATIONAL_HIGH);
    }

    public synchronized void setHeadingTarget(Rotation2d heading)
    {
        setHeadingTarget(heading.getUnboundedDegrees());
    }

    public boolean headingOnTarget()
    {
        if (periodicOutput.motorControlMode == ControlMode.MotionMagic)
        {
            return Util.epsilonEquals(periodicOutput.motorSetpoint, periodicInput.motorEncoderPosition,
                    degreesToEncoderUnits(Constants.Shooter.Turret.PitchGimbal.HEADING_ON_TARGET_ERROR_TOLERANCE));
        }
        return false;
    }

    public synchronized void setOpenLoop(double normalizedOutput)
    {
        periodicOutput.motorControlMode = ControlMode.PercentOutput;
        periodicOutput.motorSetpoint = normalizedOutput;
    }

    @Override
    public synchronized void readPeriodicInputs()
    {
        periodicInput.motorEncoderPosition = motor.getSelectedSensorPosition();
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        motor.set(periodicOutput.motorControlMode, periodicOutput.motorSetpoint);
    }

    @Override
    public synchronized void stop()
    {
        setOpenLoop(0.0);
    }

    @Override
    public void logToSmartDashboard()
    {
        SmartDashboard.putNumber("Shooter Turret Pitch", getHeading().getDegrees());
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putNumber("Shooter Turret Pitch Motor Encoder Position", motor.getSelectedSensorPosition());
            SmartDashboard.putNumber("Shooter Turret Pitch Motor Encoder Velocity", motor.getSelectedSensorVelocity());
            SmartDashboard.putNumber("Shooter Turret Pitch Motor Voltage", motor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Shooter Turret Pitch Motor Current", motor.getSupplyCurrent());
            SmartDashboard.putString("Shooter Turret Pitch Motor Control Mode", motor.getControlMode().toString());
            SmartDashboard.putBoolean("Shooter Turret Pitch Heading On Target", headingOnTarget());
            if (motor.getControlMode() == ControlMode.MotionMagic)
            {
                SmartDashboard.putNumber("Shooter Turret Pitch Closed Loop Target", motor.getClosedLoopTarget());
                SmartDashboard.putNumber("Shooter Turret Pitch Closed Loop Error",
                        encoderUnitsToDegrees(motor.getClosedLoopError()));
            }
        }
    }

    @Override
    public boolean selfTest()
    {
        return Util.epsilonEquals(encoderUnitsToDegrees(motor.getSelectedSensorPosition()), encoderUnitsToDegrees(0.0),
                Constants.Shooter.Turret.PitchGimbal.HEADING_ON_TARGET_ERROR_TOLERANCE);
    }

}
