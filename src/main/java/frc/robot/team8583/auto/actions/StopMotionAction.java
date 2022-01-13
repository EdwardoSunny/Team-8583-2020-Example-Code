/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.auto.actions;

import frc.robot.team8583.subsystems.swerve.Swerve;

/**
 * Add your docs here.
 */
public class StopMotionAction extends RunOnceAction
{
    private Swerve swerve;

    public StopMotionAction()
    {
        swerve = Swerve.getInstance();
    }

    @Override
    public void runOnce()
    {
        swerve.stop();
    }

}
