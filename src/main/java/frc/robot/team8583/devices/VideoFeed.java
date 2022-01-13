package frc.robot.team8583.devices;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.team8583.Constants;

public class VideoFeed
{
    private static VideoFeed instance = null;

    public static synchronized VideoFeed getInstance()
    {
        if (instance == null)
        {
            instance = new VideoFeed();
        }
        return instance;
    }

    private UsbCamera camera;

    public void enable()
    {
        camera = CameraServer.getInstance().startAutomaticCapture();
        configCamera();
    }

    private synchronized void configCamera()
    {
        camera.setPixelFormat(PixelFormat.kMJPEG);
        camera.setResolution(Constants.VideoFeed.RESOLUTION_WIDTH, Constants.VideoFeed.RESOLUTION_HEIGHT);
        camera.setFPS(Constants.VideoFeed.FRAMERATE);
    }
}
