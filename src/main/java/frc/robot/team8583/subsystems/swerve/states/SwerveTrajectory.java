/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.subsystems.swerve.states;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.team254.lib.geometry.Pose2dWithCurvature;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team254.lib.trajectory.TimedView;
import frc.robot.team254.lib.trajectory.Trajectory;
import frc.robot.team254.lib.trajectory.TrajectoryIterator;
import frc.robot.team254.lib.trajectory.timing.TimedState;
import frc.robot.team8583.Constants;
import frc.robot.team8583.DriveMotionPlanner;
import frc.robot.team8583.lib.util.Util;
import frc.robot.team8583.subsystems.swerve.Swerve;

/**
 * Add your docs here.
 */
public class SwerveTrajectory extends SwerveState
{
    private boolean hasStartedFollowing = false;
    private boolean hasFinishedPath = false;
    private double trajectoryStartTime = 0;

    private final DriveMotionPlanner motionPlanner;
    private Translation2d lastTrajectoryVector = new Translation2d();

    public double getRemainingProgress()
    {
        if (motionPlanner != null)
        {
            return motionPlanner.getRemainingProgress();
        }
        return 0.0;
    }

    public SwerveTrajectory(Swerve swerve)
    {
        super(swerve);
        motionPlanner = new DriveMotionPlanner();
    }

    public void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Translation2d followingCenter,
            double goalHeading)
    {
        hasStartedFollowing = false;
        hasFinishedPath = false;
        System.out.println("setting trajectory");
        motionPlanner.reset();
        motionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
        motionPlanner.setFollowingCenter(followingCenter);
        motionPlanner.setGoalHeading(goalHeading);
        trajectoryStartTime = Timer.getFPGATimestamp();
    }

    public Translation2d getLastTrajectoryVector()
    {
        return lastTrajectoryVector;
    }

    @Override
    public synchronized void update(double timestamp)
    {
        if (!motionPlanner.isDone())
        {
            Translation2d translationalInput = motionPlanner.update(timestamp, swerve.getPose()).getTranslation();
            double rotationalInput = motionPlanner.update(timestamp, swerve.getPose()).getRotation().getDegrees();
            if (!hasStartedFollowing)
            {
                hasStartedFollowing = true;
            }

            swerve.updateVectorialVelocityControl(translationalInput, rotationalInput, true, timestamp);
            lastTrajectoryVector = translationalInput;
        }
        else
        {
            if (!hasFinishedPath)
            {
                System.out.println("Path completed in: " + (timestamp - trajectoryStartTime));
                hasFinishedPath = true;
            }
        }

    }

    @Override
    public String toString()
    {
        return "Auto Path Control";
    }
}
