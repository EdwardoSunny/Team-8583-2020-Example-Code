package frc.robot.team8583.devices;

import java.util.Map;
import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import frc.robot.team8583.drivers.LazyTalonSRX;

public class TalonSRXManager
{
    private static Map<Integer, LazyTalonSRX> InitializedDeviceMap = new HashMap<Integer, LazyTalonSRX>();

    private TalonSRXManager()
    {
    }

    public static synchronized LazyTalonSRX getLazyTalonSRX(int deviceId)
    {
        LazyTalonSRX device = InitializedDeviceMap.get(deviceId);
        if (device == null)
        {
            device = new LazyTalonSRX(deviceId);
            InitializedDeviceMap.put(deviceId, device);
        }
        return device;
    }

    public static synchronized SensorCollection getSensorCollection(int deviceID)
    {
        return getLazyTalonSRX(deviceID).getSensorCollection();
    }
}
