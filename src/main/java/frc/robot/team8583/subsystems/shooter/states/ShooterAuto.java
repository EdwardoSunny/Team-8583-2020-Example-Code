/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.subsystems.shooter.states;

import frc.robot.team8583.subsystems.shooter.Shooter;

/**
 * Add your docs here.
 */
public class ShooterAuto extends ShooterState
{
    private boolean shouldLaunch = false;

    public ShooterAuto(Shooter shooter)
    {
        super(shooter);
    }

    @Override
    public synchronized void update()
    {
        shooter.setLaunch(shouldLaunch);
    }

    public void setLaunchState(boolean launchState)
    {
        this.shouldLaunch = launchState;
    }

    public boolean getLaunchState()
    {
        return shouldLaunch;
    }

    @Override
    public String toString()
    {
        return "Auto";
    }
}
