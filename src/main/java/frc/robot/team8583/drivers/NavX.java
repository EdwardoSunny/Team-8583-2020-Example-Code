/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.drivers;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class NavX
{
    private AHRS NavX;

    public NavX()
    {
        NavX = new AHRS(SPI.Port.kMXP);
    }

    public boolean isReady()
    {
        return NavX.isConnected();
    }

    public double getRawYaw()
    {
        return NavX.getYaw();
    }

    public double getFusedYaw()
    {
        //System.out.println("IMU Yaw is:" + NavX.getFusedHeading());
        return -NavX.getFusedHeading();
    }

    public double getPitch()
    {
        return NavX.getPitch();
    }

    public double getRoll()
    {
        return NavX.getRoll();
    }

    public double getGyroX()
    {
        return NavX.getRawGyroX();
    }

    public double getGyroY()
    {
        return NavX.getRawGyroY();
    }

    public double getGyroZ()
    {
        return NavX.getRawGyroZ();
    }

    public synchronized void zeroYaw()
    {
        NavX.zeroYaw();
    }

}
