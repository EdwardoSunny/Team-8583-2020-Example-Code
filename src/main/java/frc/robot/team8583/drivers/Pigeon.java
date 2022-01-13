package frc.robot.team8583.drivers;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import frc.robot.team8583.Constants;

public class Pigeon
{
    private final int MAX_STATUS_FRAME_PERIOD = 255;
    private PigeonIMU pigeon;

    public Pigeon(int deviceId)
    {
        pigeon = new PigeonIMU(deviceId);
    }

    public boolean isReady()
    {
        return pigeon.getState() == PigeonState.Ready;
    }

    public double getRawYaw()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return -ypr[0];
    }

    public double getFusedYaw()
    {
        return pigeon.getFusedHeading();
    }

    public double getPitch()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[1];
    }

    public double getRoll()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[2];
    }

    public double[] getRawYpr()
    {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr;
    }

    public double getGyroX()
    {
        double[] xyz = new double[3];
        pigeon.getRawGyro(xyz);
        return xyz[0];
    }

    public double getGyroY()
    {
        double[] xyz = new double[3];
        pigeon.getRawGyro(xyz);
        return xyz[1];
    }

    public double getGyroZ()
    {
        double[] xyz = new double[3];
        pigeon.getRawGyro(xyz);
        return xyz[2];
    }

    public synchronized void setYaw(double yawDegrees)
    {
        pigeon.setFusedHeading(-yawDegrees, Constants.Can.TIMEOUT);
    }

    public synchronized void configStatusFramePeriod(int period, boolean enableControl, int timeout)
    {
        period = Math.min(period, MAX_STATUS_FRAME_PERIOD);
        setRequiredStatusFramePeriod(period, timeout);
        setNonRequiredStatusFramePeriod(MAX_STATUS_FRAME_PERIOD, timeout);
        setControlFramePeriod(enableControl ? period : MAX_STATUS_FRAME_PERIOD);
    }

    private synchronized void setRequiredStatusFramePeriod(int period, int timeout)
    {
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, period, timeout);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, period, timeout);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, period, timeout);
    }

    private synchronized void setNonRequiredStatusFramePeriod(int period, int timeout)
    {
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, period, timeout);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, period, timeout);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, period, timeout);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, period, timeout);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, period, timeout);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, period, timeout);
    }

    private synchronized void setControlFramePeriod(int period)
    {
        pigeon.setControlFramePeriod(PigeonIMU_ControlFrame.Control_1, period);
    }

    public synchronized void configTemperatureCompensation(boolean enable)
    {
        pigeon.setTemperatureCompensationDisable(!enable, Constants.Can.TIMEOUT);
    }
}
