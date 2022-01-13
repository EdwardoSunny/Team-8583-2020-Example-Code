package frc.robot.team8583.subsystems.swerve.states;

import frc.robot.team8583.subsystems.swerve.Swerve;

public abstract class SwerveState
{
    protected final Swerve swerve;

    public SwerveState(Swerve swerve)
    {
        this.swerve = swerve;
    }

    public void update(double timestamp)
    {
    }
}
