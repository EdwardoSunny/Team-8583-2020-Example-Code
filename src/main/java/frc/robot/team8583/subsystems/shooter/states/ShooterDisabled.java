package frc.robot.team8583.subsystems.shooter.states;

import frc.robot.team8583.subsystems.shooter.Shooter;

public class ShooterDisabled extends ShooterState
{
    public ShooterDisabled(Shooter shooter)
    {
        super(shooter);
        shooter.stopTurret();
        shooter.stopBallDrive();
        shooter.stopSpindex();
        shooter.stopBallFeeder();
    }

    @Override
    public String toString()
    {
        return "Disabled";
    }
}
