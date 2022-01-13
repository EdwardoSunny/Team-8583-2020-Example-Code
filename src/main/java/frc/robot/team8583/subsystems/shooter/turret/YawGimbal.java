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
import frc.robot.team8583.drivers.LazyTalonFX;
import frc.robot.team8583.lib.util.Util;
import frc.robot.team8583.subsystems.Subsystem;

public class YawGimbal extends Subsystem
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
    private final LazyTalonFX motor = new LazyTalonFX(Ports.Can.TURRET_YAW_MOTOR);

    public YawGimbal()
    {
        configMotor();
    }

    private synchronized void configMotor()
    {
        motor.configStatusFramePeriod(Constants.Shooter.Turret.YawGimbal.STATUS_FRAME_PERIOD, true, false,
                Constants.Can.TIMEOUT);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.Can.TIMEOUT);
        motor.setInverted(Constants.Shooter.Turret.YawGimbal.INVERT_MOTOR);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configVelocityMeasurementPeriod(Constants.Shooter.Turret.YawGimbal.SPEED_MEASUREMENT_PERIOD,
                Constants.Can.TIMEOUT);
        motor.configVelocityMeasurementWindow(Constants.Shooter.Turret.YawGimbal.SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE,
                Constants.Can.TIMEOUT);
        motor.configMotionAcceleration(Util.roundToInt(Constants.Shooter.Turret.YawGimbal.MAX_ACCELERATION),
                Constants.Can.TIMEOUT);
        motor.configMotionCruiseVelocity(Util.roundToInt(Constants.Shooter.Turret.YawGimbal.MAX_CRUISE_SPEED),
                Constants.Can.TIMEOUT);
        motor.configMotionSCurveStrength(Constants.Shooter.Turret.YawGimbal.CURVE_STRENGTH, Constants.Can.TIMEOUT);
        motor.configVoltageCompSaturation(Constants.Shooter.Turret.YawGimbal.SATURATION_VOLTAGE, Constants.Can.TIMEOUT);
        motor.configVoltageMeasurementFilter(Constants.Shooter.Turret.YawGimbal.VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE,
                Constants.Can.TIMEOUT);
        motor.enableVoltageCompensation(Constants.Shooter.Turret.YawGimbal.ENABLE_VOLTAGE_COMPENSATION);
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                Constants.Shooter.Turret.YawGimbal.CURRENT_LIMITER_TARGET_CURRENT,
                Constants.Shooter.Turret.YawGimbal.CURRENT_LIMITER_THRESHOLD_CURRENT,
                Constants.Shooter.Turret.YawGimbal.CURRENT_LIMITER_TRIGGER_DELAY), Constants.Can.TIMEOUT);
        motor.configAllowableClosedloopError(0, Constants.Shooter.Turret.YawGimbal.CONTROL_ERROR_TOLERANCE,
                Constants.Can.TIMEOUT);
        motor.config_kP(0, Constants.Shooter.Turret.YawGimbal.PID0_KP, Constants.Can.TIMEOUT);
        motor.config_kI(0, Constants.Shooter.Turret.YawGimbal.PID0_KI, Constants.Can.TIMEOUT);
        motor.config_kD(0, Constants.Shooter.Turret.YawGimbal.PID0_KD, Constants.Can.TIMEOUT);
        motor.config_kF(0, Constants.Shooter.Turret.YawGimbal.PID0_KF, Constants.Can.TIMEOUT);
        motor.setSelectedSensorPosition(degreesToEncoderUnits(Constants.Shooter.Turret.YawGimbal.INIT_HEADING), 0,
                Constants.Can.TIMEOUT);
        Timer.delay(Constants.Can.MOTOR_CONTROLLER_CONFIG_DELAY);
    }

    private double encoderUnitsToDegrees(double encoderUnits)
    {
        return encoderUnits / Constants.Shooter.Turret.YawGimbal.ENCODER_UNITS_PER_DEGREE;
    }

    private int degreesToEncoderUnits(double degrees)
    {
        return Util.roundToInt(degrees * Constants.Shooter.Turret.YawGimbal.ENCODER_UNITS_PER_DEGREE);
    }

    private double encoderVelocityToDegreesPerSecond(int encoderUnitsPer100ms)
    {
        return encoderUnitsPer100ms / Constants.Shooter.Turret.YawGimbal.ENCODER_UNITS_PER_DEGREE * 10.0;
    }

    private int degreesPerSecondToEncoderVelocity(double degreesPerSecond)
    {
        return Util.roundToInt(degreesPerSecond / 10.0 * Constants.Shooter.Turret.YawGimbal.ENCODER_UNITS_PER_DEGREE);
    }

    private double getRawHeading()
    {
        return encoderUnitsToDegrees(periodicInput.motorEncoderPosition);
    }

    public Rotation2d getRobotCentricHeading()
    {
        return Rotation2d.fromDegrees(getRawHeading());
    }

    public Rotation2d getFieldCentricHeading(Rotation2d robotHeading)
    {
        return getRobotCentricHeading().rotateBy(robotHeading);
    }

    public Rotation2d getRobotCentricAngularVelocity()
    {
        return Rotation2d.fromDegrees(encoderVelocityToDegreesPerSecond(motor.getSelectedSensorVelocity()));
    }

    public Rotation2d getFieldCentricAngularVelocity(Rotation2d robotAngularVelocity)
    {
        return getRobotCentricAngularVelocity().rotateBy(robotAngularVelocity);
    }

    public synchronized void setRobotCentricHeadingTarget(double headingDegrees)
    {
        periodicOutput.motorControlMode = ControlMode.MotionMagic;
        periodicOutput.motorSetpoint = degreesToEncoderUnits(Util.limit(Util.boundAngleTo0To360Degrees(headingDegrees),
                Constants.Shooter.Turret.YawGimbal.MIN_ROTATION_HEADING,
                Constants.Shooter.Turret.YawGimbal.MAX_ROTATION_HEADING));
    }

    public synchronized void setRobotCentricHeadingTarget(Rotation2d heading)
    {
        setRobotCentricHeadingTarget(heading.getUnboundedDegrees());
    }

    public synchronized void setRelativeRobotCentricHeadingTarget(Rotation2d heading)
    {
        setRobotCentricHeadingTarget(getRobotCentricHeading().rotateBy(heading));
    }

    public synchronized void setFieldCentricHeadingTarget(Rotation2d heading, Rotation2d robotHeading)
    {
        setRobotCentricHeadingTarget(heading.rotateBy(robotHeading.inverse()));
    }

    public synchronized void setRelativeFieldCentricHeadingTarget(Rotation2d relativeHeading, Rotation2d robotHeading)
    {
        setFieldCentricHeadingTarget(getFieldCentricHeading(robotHeading).rotateBy(relativeHeading), robotHeading);
    }

    public boolean headingOnTarget()
    {
        if (periodicOutput.motorControlMode == ControlMode.MotionMagic)
        {
            return Util.epsilonEquals(periodicOutput.motorSetpoint, periodicInput.motorEncoderPosition,
                    degreesToEncoderUnits(Constants.Shooter.Turret.YawGimbal.HEADING_ON_TARGET_ERROR_TOLERANCE));
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
        SmartDashboard.putNumber("Shooter Turret Robot Centric Yaw", getRobotCentricHeading().getDegrees());
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putNumber("Shooter Turret Yaw Motor Encoder Position", motor.getSelectedSensorPosition());
            SmartDashboard.putNumber("Shooter Turret Yaw Motor Encoder Velocity", motor.getSelectedSensorVelocity());
            SmartDashboard.putNumber("Shooter Turret Yaw Motor Voltage", motor.getMotorOutputVoltage());
            SmartDashboard.putNumber("Shooter Turret Yaw Motor Current", motor.getSupplyCurrent());
            SmartDashboard.putString("Shooter Turret Yaw Motor Control Mode", motor.getControlMode().toString());
            SmartDashboard.putBoolean("Shooter Turret Yaw Heading On Target", headingOnTarget());
            if (motor.getControlMode() == ControlMode.MotionMagic)
            {
                SmartDashboard.putNumber("Shooter Turret Yaw Closed Loop Target", motor.getClosedLoopTarget());
                SmartDashboard.putNumber("Shooter Turret Yaw Closed Loop Error",
                        encoderUnitsToDegrees(motor.getClosedLoopError()));
            }
        }
    }

    @Override
    public boolean selfTest()
    {
        return Util.epsilonEquals(encoderUnitsToDegrees(motor.getSelectedSensorPosition()),
                Constants.Shooter.Turret.YawGimbal.INIT_HEADING,
                Constants.Shooter.Turret.YawGimbal.HEADING_ON_TARGET_ERROR_TOLERANCE);
    }
}
