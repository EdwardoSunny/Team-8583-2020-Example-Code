/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.auto.modes;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team8583.Constants;
import frc.robot.team8583.auto.AutoModeBase;
import frc.robot.team8583.auto.AutoModeEndedException;
import frc.robot.team8583.auto.actions.ResetPoseAction;
import frc.robot.team8583.auto.actions.SetTargetHeadingAction;
import frc.robot.team8583.auto.actions.SetTrajectoryAction;
import frc.robot.team8583.auto.actions.StopMotionAction;
import frc.robot.team8583.auto.actions.WaitAction;
import frc.robot.team8583.subsystems.collector.Collector;
import frc.robot.team8583.subsystems.collector.states.CollectorDisabled;
import frc.robot.team8583.subsystems.shooter.Shooter;
import frc.robot.team8583.subsystems.shooter.states.ShooterDisabled;
import frc.robot.team8583.subsystems.swerve.Swerve;
import frc.robot.team8583.subsystems.swerve.states.SwerveInitializing;
import frc.robot.team8583.auto.actions.SetLaunchAction;

/**
 * Add your docs here.
 */
public class RightStart11BallMode extends AutoModeBase
{
    //varibale definitions
    private int stage;
    private double prevStateTime;
    private Swerve swerve;
    private Shooter shooter;
    private Collector collector;

    //contructor method
    public RightStart11BallMode()
    {
        stage = 0;
        prevStateTime = 0.0;
        swerve = Swerve.getInstance();
        shooter = Shooter.getInstance();
        collector = Collector.getInstance();
        init();
    }

    private void init()
    {
        swerve.setState(new SwerveInitializing(swerve, Rotation2d.fromDegrees(0.0), true));
        shooter.setState(new ShooterDisabled(shooter));
        collector.setState(new CollectorDisabled(collector));
        prevStateTime = Timer.getFPGATimestamp();
    }

    @Override
    protected void routine() throws AutoModeEndedException
    {
        super.startTime = Timer.getFPGATimestamp();
        runAction(new ResetPoseAction(Constants.Field.STARTING_POSE_RIGHT));
        shooter.startSpindex();
        collector.setElevate(false);
        collector.setInject();
        shooter.activateTurret();
        runAction(new WaitAction(0.7));
        runAction(new SetLaunchAction(true));
        runAction(new WaitAction(1.5));
        runAction(new SetTrajectoryAction(trajectories.startToTrenchPath, 180));
        runAction(new WaitAction(2.8));
        runAction(new StopMotionAction());
        runAction(new SetLaunchAction(true));
        runAction(new WaitAction(3.0));
        shooter.stop();

        runAction(new SetTargetHeadingAction(32, 3));
        collector.setInject();
        runAction(new SetTrajectoryAction(trajectories.centralPick3BallPath, 32));
        runAction(new WaitAction(2.6));

        runAction(new SetTrajectoryAction(trajectories.centralShootingPath, 32));
        runAction(new WaitAction(3.0));
        runAction(new SetTargetHeadingAction(180, 3));
        collector.stop();

        shooter.activateTurret();
        runAction(new WaitAction(0.7));
        runAction(new SetLaunchAction(true));
        runAction(new WaitAction(3.0));
        shooter.stop();
        System.out.println("Auto mode finished in " + currentTime() + " seconds");
    };

}
