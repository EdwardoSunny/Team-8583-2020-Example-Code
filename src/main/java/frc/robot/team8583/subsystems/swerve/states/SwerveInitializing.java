package frc.robot.team8583.subsystems.swerve.states;

import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team8583.subsystems.swerve.Swerve;

public class SwerveInitializing extends SwerveState
{
    private boolean hasInitialModuleAlignmentHeading = false;

    public SwerveInitializing(Swerve swerve)
    {
        super(swerve);
        swerve.disableHeadingController();
        swerve.stopModules();
        swerve.setState(new SwerveNeutral(swerve));
    }

    public SwerveInitializing(Swerve swerve, Rotation2d initialModuleAlignmentHeading, boolean isFieldCentric)
    {
        super(swerve);
        hasInitialModuleAlignmentHeading = true;
        swerve.disableHeadingController();
        swerve.stopModules();
        swerve.alignModules(initialModuleAlignmentHeading, isFieldCentric);
    }

    @Override
    public void update(double timestamp)
    {
        if (!hasInitialModuleAlignmentHeading || swerve.moduleHeadingsOnTarget())
        {
            swerve.stopModules();
            swerve.setState(new SwerveNeutral(swerve));
            return;
        }
    }

    @Override
    public String toString()
    {
        return "Initializing";
    }
}
