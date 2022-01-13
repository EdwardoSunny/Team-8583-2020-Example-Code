/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.auto.actions;

import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team8583.subsystems.swerve.Swerve;
import frc.robot.team8583.subsystems.swerve.states.SwerveManual;

/**
 * Add your docs here.
 */
public class SetTargetHeadingAction implements Action
{
    double targetHeading;
    double deadband;
    Swerve swerve;

    public SetTargetHeadingAction(double targetHeading, double deadband)
    {
        this.targetHeading = targetHeading;
        this.deadband = deadband;
        swerve = Swerve.getInstance();
        swerve.enableHeadingController();
    }

    @Override
    public void start()
    {
        swerve.setState(new SwerveManual(swerve));
        swerve.setTargetHeading(new Rotation2d(targetHeading));
    }

    @Override
    public boolean isFinished()
    {
        // TODO Auto-generated method stub
        return swerve.headingOnTarget();
    }

    @Override
    public void update()
    {
        // TODO Auto-generated method stub

    }

    @Override
    public void done()
    {
        // TODO Auto-generated method stub
        swerve.disableHeadingController();
    }
}
