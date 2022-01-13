package frc.robot.team8583.subsystems.swerve.states;

import frc.robot.team8583.subsystems.swerve.Swerve;

public class SwerveNeutral extends SwerveState
{
    public SwerveNeutral(Swerve swerve)
    {
        super(swerve);
        swerve.disableHeadingController();
        swerve.stopModules();
    }

    @Override
    public String toString()
    {
        return "Neutral";
    }
}
