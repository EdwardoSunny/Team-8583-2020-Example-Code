/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.auto.actions;

import frc.robot.team254.lib.geometry.Pose2dWithCurvature;
import frc.robot.team254.lib.trajectory.Trajectory;
import frc.robot.team254.lib.trajectory.timing.TimedState;
import frc.robot.team8583.subsystems.swerve.Swerve;

/**
 * Add your docs here.
 */
public class SetTrajectoryAction extends RunOnceAction
{

    Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
    double goalHeading;
    Swerve swerve;

    public SetTrajectoryAction(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goalHeading)
    {
        this.trajectory = trajectory;
        this.goalHeading = goalHeading;
        swerve = Swerve.getInstance();
    }

    @Override
    public void runOnce()
    {
        System.out.println("begin set trajectory");
        swerve.setTrajectory(trajectory, goalHeading);
    }

}
