package frc.robot.team8583.devices;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.team8583.Ports;

public class PneumaticCompressor
{
    private static PneumaticCompressor instance = null;

    public static synchronized PneumaticCompressor getInstance()
    {
        if (instance == null)
        {
            instance = new PneumaticCompressor();
        }
        return instance;
    }

    private final Compressor compressor = new Compressor(Ports.Can.PCM);

    public void enable()
    {
        compressor.start();
    }

    public void disable()
    {
        compressor.stop();
    }

    public boolean isEnabled()
    {
        return compressor.enabled();
    }

    public boolean pressureOnTarget()
    {
        return compressor.getPressureSwitchValue();
    }

    public double getCurrent()
    {
        return compressor.getCompressorCurrent();
    }
}
