package frc.robot.team8583.drivers;

import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team8583.drivers.Limelight;

public class Vision
{
    private final Limelight limelight = new Limelight();
    private boolean isEnabled = false;

    public Vision()
    {
        disableLighting();
    }

    public synchronized void enable()
    {
        if (!isEnabled)
        {
            isEnabled = true;
            enableLighting();
        }
    }

    public synchronized void disable()
    {
        if (isEnabled)
        {
            isEnabled = false;
            disableLighting();
        }
    }

    public boolean isEnabled()
    {
        return isEnabled;
    }

    public boolean hasTarget()
    {
        return limelight.hasTarget() && isEnabled;
    }

    public Rotation2d getTargetHeading()
    {
        return limelight.getTargetX().inverse();
    }

    public double getTargetDistance(Rotation2d cameraElevation, double cameraHeight, double targetHeight)
    {
        return (targetHeight - cameraHeight)
                / Math.tan(limelight.getTargetY().getRadians() + cameraElevation.getRadians());
    }

    public Translation2d getTargetOrientation(Rotation2d cameraElevation, double cameraHeight, double targetHeight)
    {
        return Translation2d.fromPolar(getTargetHeading(),
                getTargetDistance(cameraElevation, cameraHeight, targetHeight));
    }

    public double getLatency()
    {
        return limelight.getLatency() / 1000.0;
    }

    public synchronized void enableLighting()
    {
        limelight.setLedMode(0);
    }

    public synchronized void disableLighting()
    {
        limelight.setLedMode(1);
    }
}
