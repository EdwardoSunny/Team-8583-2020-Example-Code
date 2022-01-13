package frc.robot.team254.lib.trajectory;

import frc.robot.team254.lib.geometry.Pose2d;
import frc.robot.team254.lib.geometry.Twist2d;

public interface IPathFollower {
    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
