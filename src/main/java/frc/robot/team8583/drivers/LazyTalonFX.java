package frc.robot.team8583.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class LazyTalonFX extends TalonFX
{
    protected ControlMode previousControlMode = null;
    protected double previousSetpoint = Double.NaN;

    private final int MAX_STATUS_FRAME_PERIOD = 255;

    public LazyTalonFX(int deviceId)
    {
        super(deviceId);
        configFactoryDefault();
    }

    public void configStatusFramePeriod(int period, boolean enableFeedback, boolean enableDebug, int timeout)
    {
        period = Math.min(period, MAX_STATUS_FRAME_PERIOD);
        setCriticalStatusFramePeriod(period, timeout);
        setFeedbackStatusFramePeriod(enableFeedback ? period : MAX_STATUS_FRAME_PERIOD, timeout);
        setDebugStatusFramePeriod(enableDebug ? period : MAX_STATUS_FRAME_PERIOD, timeout);
    }

    private void setCriticalStatusFramePeriod(int period, int timeout)
    {
        setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, period, timeout);
    }

    private void setFeedbackStatusFramePeriod(int period, int timeout)
    {
        setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, period, timeout);
    }

    private void setDebugStatusFramePeriod(int period, int timeout)
    {
        setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, period, timeout);
        setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, period, timeout);
    }

    @Override
    public void set(ControlMode controlMode, double setpoint)
    {
        if (setpoint != previousSetpoint || controlMode != previousControlMode)
        {
            previousControlMode = controlMode;
            previousSetpoint = setpoint;
            super.set(controlMode, setpoint);
        }
    }
}
