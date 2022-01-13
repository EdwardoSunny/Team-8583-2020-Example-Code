package frc.robot.team8583.subsystems.swerve.states;

import frc.robot.team8583.subsystems.swerve.Swerve;

public class SwerveDisabled extends SwerveState
{
    public SwerveDisabled(Swerve swerve)
    {
        super(swerve);
        swerve.disableHeadingController();
        swerve.disableModules();
    }

    @Override
    public String toString()
    {
        return "Disabled";
    }
}
