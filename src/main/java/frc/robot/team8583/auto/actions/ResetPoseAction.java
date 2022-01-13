/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.team254.lib.geometry.Pose2d;
import frc.robot.team8583.subsystems.swerve.Swerve;

/**
 * Add your docs here.
 */
public class ResetPoseAction extends RunOnceAction
{
    private Pose2d newPose;
    Swerve swerve;

    public ResetPoseAction(Pose2d newPose)
    {
        this.newPose = newPose;
        swerve = Swerve.getInstance();
    }

    @Override
    public void runOnce()
    {
        swerve.setStartingPose(newPose);
        swerve.resetSensors();
    }

}
