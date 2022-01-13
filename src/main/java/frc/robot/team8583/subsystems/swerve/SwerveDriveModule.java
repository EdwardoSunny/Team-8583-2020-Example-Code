package frc.robot.team8583.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team254.lib.geometry.Pose2d;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team8583.Constants;
import frc.robot.team8583.devices.TalonSRXManager;
import frc.robot.team8583.drivers.LazyTalonFX;
import frc.robot.team8583.lib.util.Util;
import frc.robot.team8583.subsystems.Subsystem;

public class SwerveDriveModule extends Subsystem
{
    private class PeriodicInput
    {
        public int rotationMotorEncoderPosition = 0;
        public int translationMotorEncoderPosition = 0;
        public int previousTranslationMotorEncoderPosition = 0;
        public int translationMotorEncoderVelocity = 0;
    }

    private class PeriodicOutput
    {
        public ControlMode rotationMotorControlMode = ControlMode.PercentOutput;
        public ControlMode translationMotorControlMode = ControlMode.PercentOutput;
        public double rotationMotorSetpoint = 0.0;
        public double translationMotorSetpoint = 0.0;
    }

    private final int moduleId;
    private final String moduleName;
    private Translation2d modulePosition;
    private final int rotationCalibrationOffset;
    private final Translation2d modulePositionRelativeToTranslationCenter;
    private Pose2d estimatedRobotPose = new Pose2d();
    private final PeriodicInput periodicInput = new PeriodicInput();
    private final PeriodicOutput periodicOutput = new PeriodicOutput();
    private final LazyTalonFX translationMotor, rotationMotor;
    private final SensorCollection externalRotationEncoder;

    public SwerveDriveModule(int moduleId, int translationMotorDeviceId, int rotationMotorDeviceId,
            int externalRotationEncoderDeviceId, int rotationCalibrationOffset,
            Translation2d modulePositionRelativeToTranslationCenter)
    {
        this.moduleId = moduleId;
        moduleName = "Swerve Module " + moduleId + " ";
        translationMotor = new LazyTalonFX(translationMotorDeviceId);
        rotationMotor = new LazyTalonFX(rotationMotorDeviceId);
        externalRotationEncoder = TalonSRXManager.getSensorCollection(externalRotationEncoderDeviceId);
        this.rotationCalibrationOffset = rotationCalibrationOffset;
        this.modulePositionRelativeToTranslationCenter = modulePositionRelativeToTranslationCenter;
        configTranslationMotor();
        configRotationMotor();
        calibrateRotationEncoder();
        readPeriodicInputs();
        synchronizePreviousTranslationEncoderPosition();
        resetSensors();
    }

    public int getModuleId()
    {
        return moduleId;
    }

    private synchronized void configTranslationMotor()
    {
        translationMotor.configStatusFramePeriod(Constants.Swerve.DriveModule.Translation.STATUS_FRAME_PERIOD, true,
                false, Constants.Can.TIMEOUT);
        translationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.Can.TIMEOUT);
        translationMotor.setInverted(Constants.Swerve.DriveModule.Translation.INVERT_MOTOR);
        translationMotor.setNeutralMode(NeutralMode.Brake);
        translationMotor.configVelocityMeasurementPeriod(
                Constants.Swerve.DriveModule.Translation.SPEED_MEASUREMENT_PERIOD, Constants.Can.TIMEOUT);
        translationMotor.configVelocityMeasurementWindow(
                Constants.Swerve.DriveModule.Translation.SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE, Constants.Can.TIMEOUT);
        translationMotor.configVoltageCompSaturation(Constants.Swerve.DriveModule.Translation.SATURATION_VOLTAGE,
                Constants.Can.TIMEOUT);
        translationMotor.configVoltageMeasurementFilter(
                Constants.Swerve.DriveModule.Translation.VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE, Constants.Can.TIMEOUT);
        translationMotor
                .enableVoltageCompensation(Constants.Swerve.DriveModule.Translation.ENABLE_VOLTAGE_COMPENSATION);
        translationMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                Constants.Swerve.DriveModule.Translation.CURRENT_LIMITER_TARGET_CURRENT,
                Constants.Swerve.DriveModule.Translation.CURRENT_LIMITER_THRESHOLD_CURRENT,
                Constants.Swerve.DriveModule.Translation.CURRENT_LIMITER_TRIGGER_DELAY), Constants.Can.TIMEOUT);
        translationMotor.configAllowableClosedloopError(0,
                Constants.Swerve.DriveModule.Translation.CONTROL_ERROR_TOLERANCE, Constants.Can.TIMEOUT);
        translationMotor.config_kP(0, Constants.Swerve.DriveModule.Translation.PID0_KP, Constants.Can.TIMEOUT);
        translationMotor.config_kI(0, Constants.Swerve.DriveModule.Translation.PID0_KI, Constants.Can.TIMEOUT);
        translationMotor.config_kD(0, Constants.Swerve.DriveModule.Translation.PID0_KD, Constants.Can.TIMEOUT);
        translationMotor.config_kF(0, Constants.Swerve.DriveModule.Translation.PID0_KF, Constants.Can.TIMEOUT);
        translationMotor.setSelectedSensorPosition(0, 0, Constants.Can.TIMEOUT);
        Timer.delay(Constants.Can.MOTOR_CONTROLLER_CONFIG_DELAY);
    }

    private boolean validateTranslationMotorCalibration()
    {
        return translationMotor.getInverted() == Constants.Swerve.DriveModule.Translation.INVERT_MOTOR && Util
                .epsilonEquals(translationMotor.getSelectedSensorPosition(), 0.0, inchesToTranslationEncoderUnits(
                        Constants.Swerve.DriveModule.Translation.DISTANCE_ON_TARGET_ERROR_TOLERANCE));
    }

    private synchronized void configRotationMotor()
    {
        rotationMotor.configStatusFramePeriod(Constants.Swerve.DriveModule.Rotation.STATUS_FRAME_PERIOD, true, false,
                Constants.Can.TIMEOUT);
        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.Can.TIMEOUT);
        rotationMotor.setInverted(Constants.Swerve.DriveModule.Rotation.INVERT_MOTOR);
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.configVelocityMeasurementPeriod(Constants.Swerve.DriveModule.Rotation.SPEED_MEASUREMENT_PERIOD,
                Constants.Can.TIMEOUT);
        rotationMotor.configVelocityMeasurementWindow(
                Constants.Swerve.DriveModule.Rotation.SPEED_MEASUREMENT_FILTER_SAMPLE_SIZE, Constants.Can.TIMEOUT);
        rotationMotor.configMotionAcceleration(Util.roundToInt(Constants.Swerve.DriveModule.Rotation.MAX_ACCELERATION),
                Constants.Can.TIMEOUT);
        rotationMotor.configMotionCruiseVelocity(
                Util.roundToInt(Constants.Swerve.DriveModule.Rotation.MAX_CRUISE_SPEED), Constants.Can.TIMEOUT);
        rotationMotor.configVoltageCompSaturation(Constants.Swerve.DriveModule.Rotation.SATURATION_VOLTAGE,
                Constants.Can.TIMEOUT);
        rotationMotor.configVoltageMeasurementFilter(
                Constants.Swerve.DriveModule.Rotation.VOLTAGE_MEASUREMENT_FILTER_SAMPLE_SIZE, Constants.Can.TIMEOUT);
        rotationMotor.enableVoltageCompensation(Constants.Swerve.DriveModule.Rotation.ENABLE_VOLTAGE_COMPENSATION);
        rotationMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                Constants.Swerve.DriveModule.Rotation.CURRENT_LIMITER_TARGET_CURRENT,
                Constants.Swerve.DriveModule.Rotation.CURRENT_LIMITER_THRESHOLD_CURRENT,
                Constants.Swerve.DriveModule.Rotation.CURRENT_LIMITER_TRIGGER_DELAY), Constants.Can.TIMEOUT);
        rotationMotor.configAllowableClosedloopError(0, Constants.Swerve.DriveModule.Rotation.CONTROL_ERROR_TOLERANCE,
                Constants.Can.TIMEOUT);
        rotationMotor.config_kP(0, Constants.Swerve.DriveModule.Rotation.PID0_KP, Constants.Can.TIMEOUT);
        rotationMotor.config_kI(0, Constants.Swerve.DriveModule.Rotation.PID0_KI, Constants.Can.TIMEOUT);
        rotationMotor.config_kD(0, Constants.Swerve.DriveModule.Rotation.PID0_KD, Constants.Can.TIMEOUT);
        rotationMotor.config_kF(0, Constants.Swerve.DriveModule.Rotation.PID0_KF, Constants.Can.TIMEOUT);
        Timer.delay(Constants.Can.MOTOR_CONTROLLER_CONFIG_DELAY);
    }

    private boolean validateRotationMotorCalibration()
    {
        return rotationMotor.getInverted() == Constants.Swerve.DriveModule.Rotation.INVERT_MOTOR && Util.epsilonEquals(
                rotationMotor.getSelectedSensorPosition(), getRotationEncoderCalibrationTarget(),
                degreesToRotationEncoderUnits(Constants.Swerve.DriveModule.Rotation.HEADING_ON_TARGET_ERROR_TOLERANCE));
    }

    public synchronized void setInvertTranslationMotor(boolean invert)
    {
        translationMotor.setInverted(Constants.Swerve.DriveModule.Translation.INVERT_MOTOR ^ invert);
    }

    public synchronized void setInvertRotationMotor(boolean invert)
    {
        rotationMotor.setInverted(Constants.Swerve.DriveModule.Rotation.INVERT_MOTOR ^ invert);
    }

    private double translationEncoderUnitsToInches(double EncoderUnits)
    {
        return EncoderUnits / Constants.Swerve.DriveModule.Translation.ENCODER_UNITS_PER_INCH;
    }

    private int inchesToTranslationEncoderUnits(double inches)
    {
        return Util.roundToInt(inches * Constants.Swerve.DriveModule.Translation.ENCODER_UNITS_PER_INCH);
    }

    private double translationEncoderVelocityToInchesPerSecond(double encoderUnitsPer100ms)
    {
        return translationEncoderUnitsToInches(encoderUnitsPer100ms) * 10.0;
    }

    private int inchesPerSecondToTranslationEncoderVelocity(double inchesPerSecond)
    {
        return Util.roundToInt(inchesToTranslationEncoderUnits(inchesPerSecond / 10.0));
    }

    public double getTranslationDisplacement()
    {
        return translationEncoderUnitsToInches(periodicInput.translationMotorEncoderPosition);
    }

    public double getPreviousTranslationDisplacement()
    {
        return translationEncoderUnitsToInches(periodicInput.previousTranslationMotorEncoderPosition);
    }

    public double getTranslationVelocity()
    {
        return translationEncoderVelocityToInchesPerSecond(periodicInput.translationMotorEncoderVelocity);
    }

    public synchronized void setTranslationVelocityTarget(double velocityInchesPerSecond)
    {
        periodicOutput.translationMotorControlMode = ControlMode.Velocity;
        periodicOutput.translationMotorSetpoint = inchesPerSecondToTranslationEncoderVelocity(velocityInchesPerSecond);
    }

    public synchronized void setNormalizedTranslationVelocityTarget(double normalizedVelocity)
    {
        periodicOutput.translationMotorControlMode = ControlMode.Velocity;
        periodicOutput.translationMotorSetpoint = Constants.Swerve.DriveModule.Translation.MAX_CRUISE_SPEED
                * normalizedVelocity;
    }

    public synchronized boolean translationVelocityOnTarget()
    {
        if (periodicOutput.translationMotorControlMode == ControlMode.Velocity)
        {
            return Util.epsilonEquals(periodicOutput.translationMotorSetpoint,
                    periodicInput.translationMotorEncoderVelocity, inchesPerSecondToTranslationEncoderVelocity(
                            Constants.Swerve.DriveModule.Translation.SPEED_ON_TARGET_ERROR_TOLERANCE));
        }
        return false;
    }

    public synchronized void setTranslationOpenLoop(double normalizedOutput)
    {
        periodicOutput.translationMotorControlMode = ControlMode.PercentOutput;
        periodicOutput.translationMotorSetpoint = normalizedOutput;
    }

    private int degreesToRotationEncoderUnits(double degrees)
    {
        return Util.roundToInt(degrees * Constants.Swerve.DriveModule.Rotation.ENCODER_UNITS_PER_DEGREE);
    }

    private double rotationEncoderUnitsToDegrees(double encoderUnits)
    {
        return encoderUnits / Constants.Swerve.DriveModule.Rotation.ENCODER_UNITS_PER_DEGREE;
    }

    private double getRawRotationHeading()
    {
        return rotationEncoderUnitsToDegrees(periodicInput.rotationMotorEncoderPosition);
    }

    public Rotation2d getRobotCentricRotationHeading()
    {
        return Rotation2d
                .fromDegrees(getRawRotationHeading() - rotationEncoderUnitsToDegrees(rotationCalibrationOffset));
    }

    public Rotation2d getFieldCentricRotationHeading(Rotation2d robotHeading)
    {
        return getRobotCentricRotationHeading().rotateBy(robotHeading);
    }

    private void setRotationHeadingTarget(double headingDegrees)
    {
        periodicOutput.rotationMotorControlMode = ControlMode.MotionMagic;
        periodicOutput.rotationMotorSetpoint = degreesToRotationEncoderUnits(Util.boundAngleToClosestScope(
                headingDegrees + rotationEncoderUnitsToDegrees(rotationCalibrationOffset), getRawRotationHeading()));
    }

    public synchronized void setRotationHeadingTarget(Rotation2d heading)
    {
        setRotationHeadingTarget(heading.getUnboundedDegrees());
    }

    public boolean rotationHeadingOnTarget()
    {
        if (periodicOutput.rotationMotorControlMode == ControlMode.MotionMagic)
        {
            return Util.epsilonEquals(periodicOutput.rotationMotorSetpoint, periodicInput.rotationMotorEncoderPosition,
                    degreesToRotationEncoderUnits(
                            Constants.Swerve.DriveModule.Rotation.HEADING_ON_TARGET_ERROR_TOLERANCE));
        }
        return false;
    }

    public synchronized void setRotationOpenLoop(double normalizedOutput)
    {
        periodicOutput.rotationMotorControlMode = ControlMode.PercentOutput;
        periodicOutput.rotationMotorSetpoint = normalizedOutput;
    }

    public int getExternalRotationEncoderPosition()
    {
        return Util.conditionalInvert(externalRotationEncoder.getPulseWidthPosition(),
                Constants.Swerve.DriveModule.Rotation.INVERT_EXTERNAL_SENSOR);
    }

    public int getRotationEncoderCalibrationTarget()
    {
        return Util.roundToInt(Util.boundToNonnegativeScope(
                Util.conditionalInvert(
                        getExternalRotationEncoderPosition()
                                / Constants.Swerve.DriveModule.Rotation.ENCODER_TO_EXTERNAL_ENCODER_RATIO,
                        rotationMotor.getInverted()),
                Constants.Swerve.DriveModule.Rotation.ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION));
    }

    public synchronized void updateOdometer(Rotation2d robotHeading)
    {
        double deltaEncoderPosition = getTranslationDisplacement() - getPreviousTranslationDisplacement();
        Rotation2d rotationHeading = getFieldCentricRotationHeading(robotHeading);
        modulePosition = modulePosition.translateBy(new Translation2d(rotationHeading.cos() * deltaEncoderPosition,
                rotationHeading.sin() * deltaEncoderPosition));
        estimatedRobotPose = new Pose2d(modulePosition, robotHeading)
                .transformBy(Pose2d.fromTranslation(modulePositionRelativeToTranslationCenter.inverse()));
        synchronizePreviousTranslationEncoderPosition();
    }

    public Translation2d getModulePosition()
    {
        return modulePosition;
    }

    public Pose2d getEstimatedRobotPose()
    {
        return estimatedRobotPose;
    }

    public synchronized void setModulePosition(Translation2d modulePosition)
    {
        this.modulePosition = modulePosition;
    }

    public synchronized void setModulePositionFromRobotPose(Pose2d robotPose)
    {
        setModulePosition(robotPose.transformBy(Pose2d.fromTranslation(modulePositionRelativeToTranslationCenter))
                .getTranslation());
        estimatedRobotPose = robotPose;
    }

    public synchronized void resetModulePosition()
    {
        setModulePosition(modulePositionRelativeToTranslationCenter);
    }

    private synchronized void synchronizePreviousTranslationEncoderPosition()
    {
        periodicInput.previousTranslationMotorEncoderPosition = periodicInput.translationMotorEncoderPosition;
    }

    private synchronized void calibrateRotationEncoder()
    {
        rotationMotor.setSelectedSensorPosition(getRotationEncoderCalibrationTarget(), 0, Constants.Can.TIMEOUT);
        Timer.delay(Constants.Swerve.DriveModule.Rotation.CALIBRATION_DELAY);
    }

    @Override
    public synchronized void stop()
    {
        setTranslationOpenLoop(0.0);
    }

    @Override
    public synchronized void disable()
    {
        setTranslationOpenLoop(0.0);
        setRotationOpenLoop(0.0);
    }

    @Override
    public synchronized void readPeriodicInputs()
    {
        periodicInput.rotationMotorEncoderPosition = rotationMotor.getSelectedSensorPosition();
        periodicInput.translationMotorEncoderPosition = translationMotor.getSelectedSensorPosition();
        periodicInput.translationMotorEncoderVelocity = translationMotor.getSelectedSensorVelocity();
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        rotationMotor.set(periodicOutput.rotationMotorControlMode, periodicOutput.rotationMotorSetpoint);
        translationMotor.set(periodicOutput.translationMotorControlMode, periodicOutput.translationMotorSetpoint);
    }

    @Override
    public synchronized void resetSensors()
    {
        resetModulePosition();
    }

    @Override
    public void logToSmartDashboard()
    {
        SmartDashboard.putNumber(moduleName + "Rotation Heading", getRobotCentricRotationHeading().getDegrees());
        SmartDashboard.putNumber(moduleName + "Translation Velocity", getTranslationVelocity());
        SmartDashboard.putNumber(moduleName + "Translation Displacement", getTranslationDisplacement());
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putNumber(moduleName + "Translation Motor Encoder Position",
                    translationMotor.getSelectedSensorPosition());
            SmartDashboard.putNumber(moduleName + "Translation Motor Encoder Velocity",
                    translationMotor.getSelectedSensorVelocity());
            SmartDashboard.putNumber(moduleName + "Translation Motor Voltage",
                    translationMotor.getMotorOutputVoltage());
            SmartDashboard.putNumber(moduleName + "Translation Motor Current", translationMotor.getSupplyCurrent());
            SmartDashboard.putString(moduleName + "Translation Motor Control Mode",
                    translationMotor.getControlMode().toString());
            SmartDashboard.putBoolean(moduleName + "Translation Velocity On Target", translationVelocityOnTarget());
            if (translationMotor.getControlMode() == ControlMode.Velocity)
            {
                SmartDashboard.putNumber(moduleName + "Translation Closed Loop Target",
                        translationMotor.getClosedLoopTarget());
                SmartDashboard.putNumber(moduleName + "Translation Closed Loop Error",
                        rotationEncoderUnitsToDegrees(translationMotor.getClosedLoopError()));
            }

            SmartDashboard.putNumber(moduleName + "Rotation Motor Encoder Position",
                    rotationMotor.getSelectedSensorPosition());
            SmartDashboard.putNumber(moduleName + "Rotation Motor Encoder Velocity",
                    rotationMotor.getSelectedSensorVelocity());
            SmartDashboard.putNumber(moduleName + "Rotation Motor Encoder Calibration Target",
                    getRotationEncoderCalibrationTarget());
            SmartDashboard.putNumber(moduleName + "Rotation Motor Voltage", rotationMotor.getMotorOutputVoltage());
            SmartDashboard.putNumber(moduleName + "Rotation Motor Current", rotationMotor.getSupplyCurrent());
            SmartDashboard.putString(moduleName + "Rotation Motor Control Mode",
                    rotationMotor.getControlMode().toString());
            SmartDashboard.putBoolean(moduleName + "Rotation Heading On Target", rotationHeadingOnTarget());
            if (rotationMotor.getControlMode() == ControlMode.MotionMagic)
            {
                SmartDashboard.putNumber(moduleName + "Rotation Closed Loop Target",
                        rotationMotor.getClosedLoopTarget());
                SmartDashboard.putNumber(moduleName + "Rotation Closed Loop Error",
                        rotationEncoderUnitsToDegrees(rotationMotor.getClosedLoopError()));
            }

            SmartDashboard.putString(moduleName + "Module Position", modulePosition.toString());
            SmartDashboard.putString(moduleName + "Estimated Robot Pose", estimatedRobotPose.toString());
        }
    }

    @Override
    public boolean selfTest()
    {
        return validateTranslationMotorCalibration() && validateRotationMotorCalibration();
    }

}
