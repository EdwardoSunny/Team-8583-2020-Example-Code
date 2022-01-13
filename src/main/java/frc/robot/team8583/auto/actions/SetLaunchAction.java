/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.auto.actions;

import frc.robot.team8583.subsystems.shooter.Shooter;

/**
 * Add your docs here.
 */
public class SetLaunchAction extends RunOnceAction
{
    boolean launchState = false;
    Shooter shooter;

    public SetLaunchAction(boolean launchState)
    {
        this.launchState = launchState;
        shooter = Shooter.getInstance();
    }

    @Override
    public void runOnce()
    {
        System.out.println("start launch state control");
        shooter.setAutoLaunch(launchState);
    }

}
