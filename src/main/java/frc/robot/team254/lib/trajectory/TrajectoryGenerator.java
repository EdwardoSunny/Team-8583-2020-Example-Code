package frc.robot.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.team8583.Constants;
import frc.robot.team8583.DriveMotionPlanner;
import frc.robot.team254.lib.geometry.Pose2d;
import frc.robot.team254.lib.geometry.Pose2dWithCurvature;
import frc.robot.team254.lib.geometry.Pose2dWithTwist;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team254.lib.geometry.Twist2d;
import frc.robot.team254.lib.trajectory.timing.TimedState;
import frc.robot.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;

public class TrajectoryGenerator
{
    private static final double kMaxVelocity = Constants.Swerve.TrajectoryGenerator.kMaxVelocity;
    private static final double kMaxAccel = Constants.Swerve.TrajectoryGenerator.kMaxAccel;
    private static final double kMaxDecel = Constants.Swerve.TrajectoryGenerator.kMaxDecel;
    private static final double kMaxVoltage = Constants.Swerve.TrajectoryGenerator.kMaxVoltage;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance()
    {
        return mInstance;
    }

    private TrajectoryGenerator()
    {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories()
    {
        if (mTrajectorySet == null)
        {
            double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println(
                    "Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public TrajectorySet getTrajectorySet()
    {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints, double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_decel, double max_voltage, double default_vel, int slowdown_chunks)
    {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel,
                max_voltage, default_vel, slowdown_chunks);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel, // inches/s
            double end_vel, // inches/s
            double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_decel, double max_voltage, double default_vel, int slowdown_chunks)
    {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel,
                max_accel, max_decel, max_voltage, default_vel, slowdown_chunks);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the right.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON LEFT! (mirrored about +x axis for RIGHT)

    static final Pose2d autoStartingPose = Constants.Field.STARTING_POSE_RIGHT;
    static final Pose2d trenchWayPoint1 = Constants.Field.RIGHT_TRENCH_WAT_POINT_1;
    static final Pose2d trenchWayPoint2 = Constants.Field.RIGHT_TRENCH_WAT_POINT_2;
    static final Pose2d trenchWayPoint3 = Constants.Field.RIGHT_TRENCH_WAT_POINT_3;
    static final Pose2d trenchEndPoint = Constants.Field.RIGHT_TRENCH_END_POINT;

    static final Pose2d pick3BallStartPoint = Constants.Field.RIGHT_CENTRAL_3_BALL_START_POINT;
    static final Pose2d pick3BallWayPoint1 = Constants.Field.RIGHT_CENTRAL_3_BALL_WAY_POINT_1;
    static final Pose2d pick3BallWayPoint2 = Constants.Field.RIGHT_CENTRAL_3_BALL_WAY_POINT_2;
    static final Pose2d pick3BallWayPoint3 = Constants.Field.RIGHT_CENTRAL_3_BALL_WAY_POINT_3;
    static final Pose2d pick3BallWayPoint4 = Constants.Field.RIGHT_CENTRAL_3_BALL_WAY_POINT_4;
    static final Pose2d pick3BallWayPoint5 = Constants.Field.RIGHT_CENTRAL_3_BALL_WAY_POINT_5;
    static final Pose2d pick3BallWayPoint6 = Constants.Field.RIGHT_CENTRAL_3_BALL_WAY_POINT_6;
    static final Pose2d pick3BallWayPoint7 = Constants.Field.RIGHT_CENTRAL_3_BALL_WAY_POINT_7;
    static final Pose2d pick3BallWayPoint8 = Constants.Field.RIGHT_CENTRAL_3_BALL_WAY_POINT_8;
    static final Pose2d pick3BallWayPoint9 = Constants.Field.RIGHT_CENTRAL_3_BALL_WAY_POINT_9;
    //static final Pose2d pick3BallStartPoint1 = Constants.Field.RIGHT_CENTRAL_3_BALL_START_POINT_1;
    static final Pose2d pick3BallEndPoint = Constants.Field.RIGHT_CENTRAL_3_BALL_END_POINT;

    static final Pose2d ready2BallWayPoint1 = Constants.Field.RIGHT_CENTRAL_RETREAT_WAY_POINT_1;
    static final Pose2d ready2BallWayPoint2 = Constants.Field.RIGHT_CENTRAL_RETREAT_WAY_POINT_2;
    static final Pose2d ready2BallWayPoint3 = Constants.Field.RIGHT_CENTRAL_RETREAT_WAY_POINT_3;
    static final Pose2d pick2BallStartPoint = Constants.Field.RIGHT_CENTRAL_2_BALL_START_POINT;
    //static final Pose2d pick2BallWayPoint = Constants.Field.RIGHT_CENTRAL_2_BALL_WAY_POINT;
    //static final Pose2d pick2BallEndPoint = Constants.Field.RIGHT_CENTRAL_2_BALL_END_POINT;
    //static final Pose2d shootingWayPoint = Constants.Field.RIGHT_SHOOT_WAY_POINT;
    static final Pose2d shootingWayPoint1 = Constants.Field.RIGHT_SHOOT_WAY_POINT_1;
    static final Pose2d shootingWayPoint2 = Constants.Field.RIGHT_SHOOT_WAY_POINT_2;
    static final Pose2d shootingWayPoint3 = Constants.Field.RIGHT_SHOOT_WAY_POINT_3;
    static final Pose2d shootingWayPoint4 = Constants.Field.RIGHT_SHOOT_WAY_POINT_4;
    static final Pose2d shootingWayPoint5 = Constants.Field.RIGHT_SHOOT_WAY_POINT_5;
    static final Pose2d shootingWayPoint6 = Constants.Field.RIGHT_SHOOT_WAY_POINT_6;
    static final Pose2d shootingWayPoint7 = Constants.Field.RIGHT_SHOOT_WAY_POINT_7;
    static final Pose2d shootingWayPoint8 = Constants.Field.RIGHT_SHOOT_WAY_POINT_8;
    static final Pose2d shootingWayPoint9 = Constants.Field.RIGHT_SHOOT_WAY_POINT_9;
    static final Pose2d shootingWayPoint10 = Constants.Field.RIGHT_SHOOT_WAY_POINT_10;
    static final Pose2d shootingWayPoint11 = Constants.Field.RIGHT_SHOOT_WAY_POINT_11;
    static final Pose2d shootingWayPoint12 = Constants.Field.RIGHT_SHOOT_WAY_POINT_12;
    static final Pose2d shootingWayPoint13 = Constants.Field.RIGHT_SHOOT_WAY_POINT_13;
    static final Pose2d shootingWayPoint14 = Constants.Field.RIGHT_SHOOT_WAY_POINT_14;
    static final Pose2d shootingWayPoint15 = Constants.Field.RIGHT_SHOOT_WAY_POINT_15;
    static final Pose2d shootingWayPoint16 = Constants.Field.RIGHT_SHOOT_WAY_POINT_16;
    static final Pose2d shootingWayPoint17 = Constants.Field.RIGHT_SHOOT_WAY_POINT_17;
    static final Pose2d shootingPoint = Constants.Field.RIGHT_SHOOT_END_POINT;

    public class TrajectorySet
    {
        public class MirroredTrajectory
        {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> left)
            {
                this.left = left;
                this.right = TrajectoryUtil.mirrorTimed(left, left.defaultVelocity());
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left)
            {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        //Test Paths
        public final Trajectory<TimedState<Pose2dWithCurvature>> straightPath;

        public final Trajectory<TimedState<Pose2dWithCurvature>> startToTrenchPath;
        //public final Trajectory<TimedState<Pose2dWithCurvature>> trenchtoCentralPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> centralPick3BallPath;
        //public final Trajectory<TimedState<Pose2dWithCurvature>> readyCentral2BallPath;
        //public final Trajectory<TimedState<Pose2dWithCurvature>> centralPick2BallPath;
        //public final Trajectory<TimedState<Pose2dWithCurvature>> centralPick5BallPath;
        public final Trajectory<TimedState<Pose2dWithCurvature>> centralShootingPath;

        private TrajectorySet()
        {
            //Test Paths
            straightPath = getStraightPath();

            //Right auto mode trajectory
            startToTrenchPath = getStartToTrenchPath();
            //trenchtoCentralPath = getTrenchToCentralPath();
            centralPick3BallPath = getCentralPick3BallPath();
            //readyCentral2BallPath = getReadyCentralPick2BallPath();
            //centralPick2BallPath = getCentralPick2BallPath();
            //centralPick5BallPath = getCentralPick5BallPath();
            centralShootingPath = getCentralShootingPath();

        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStraightPath()
        {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(autoStartingPose);
            System.out.println("waypoint1:" + autoStartingPose.toString());

            waypoints.add(autoStartingPose.transformBy(new Pose2d(new Translation2d(0.0, 50.0), new Rotation2d(0.0))));
            System.out.println("waypoint2:" + autoStartingPose
                    .transformBy(new Pose2d(new Translation2d(0.0, 50.0), new Rotation2d(0.0))).toString());

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel,
                    kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToTrenchPath()
        {
            List<Pose2d> waypoints = new ArrayList<>();

            waypoints.add(autoStartingPose);
            waypoints.add(trenchWayPoint1);
            waypoints.add(trenchWayPoint2);
            waypoints.add(trenchWayPoint3);
            waypoints.add(trenchEndPoint);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel,
                    kMaxVoltage, 20.0, 1);
        }

        /*private Trajectory<TimedState<Pose2dWithCurvature>> getTrenchToCentralPath()
        {
            List<Pose2d> waypoints = new ArrayList<>();
        
            waypoints.add(trenchEndPoint);
            waypoints.add(pick3BallStartPoint1);
        
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel,
                    kMaxVoltage, 20.0, 1);
        }*/

        private Trajectory<TimedState<Pose2dWithCurvature>> getCentralPick3BallPath()
        {
            List<Pose2d> waypoints = new ArrayList<>();

            waypoints.add(pick3BallStartPoint);
            waypoints.add(pick3BallWayPoint1);
            waypoints.add(pick3BallWayPoint2);
            waypoints.add(pick3BallWayPoint3);
            waypoints.add(pick3BallWayPoint4);
            waypoints.add(pick3BallWayPoint5);
            waypoints.add(pick3BallWayPoint6);
            waypoints.add(pick3BallWayPoint7);
            waypoints.add(pick3BallWayPoint8);
            waypoints.add(pick3BallWayPoint9);
            waypoints.add(pick3BallEndPoint);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel,
                    kMaxVoltage, 20.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getReadyCentralPick2BallPath()
        {
            List<Pose2d> waypoints = new ArrayList<>();

            waypoints.add(pick3BallEndPoint);
            waypoints.add(ready2BallWayPoint1);
            waypoints.add(ready2BallWayPoint2);
            waypoints.add(ready2BallWayPoint3);
            waypoints.add(pick2BallStartPoint);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel,
                    kMaxVoltage, 20.0, 1);
        }

        /*private Trajectory<TimedState<Pose2dWithCurvature>> getCentralPick2BallPath()
        {
            List<Pose2d> waypoints = new ArrayList<>();
        
            waypoints.add(pick2BallStartPoint);
            waypoints.add(pick2BallWayPoint);
            waypoints.add(pick2BallEndPoint);
        
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel,
                    kMaxVoltage, 20.0, 1);
        }
        
        private Trajectory<TimedState<Pose2dWithCurvature>> getCentralPick5BallPath()
        {
            List<Pose2d> waypoints = new ArrayList<>();
        
            waypoints.add(pick3BallStartPoint2);
            waypoints.add(pick3BallEndPoint);
            waypoints.add(pick2BallStartPoint);
            waypoints.add(pick2BallWayPoint);
            waypoints.add(pick2BallEndPoint);
        
            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel,
                    kMaxVoltage, 20.0, 1);
        }*/

        private Trajectory<TimedState<Pose2dWithCurvature>> getCentralShootingPath()
        {
            List<Pose2d> waypoints = new ArrayList<>();

            waypoints.add(pick3BallEndPoint);
            waypoints.add(shootingWayPoint1);
            waypoints.add(shootingWayPoint2);
            waypoints.add(shootingWayPoint3);
            waypoints.add(shootingWayPoint4);
            waypoints.add(shootingWayPoint5);
            waypoints.add(shootingWayPoint6);
            waypoints.add(shootingWayPoint7);
            waypoints.add(shootingWayPoint8);
            waypoints.add(shootingWayPoint9);
            waypoints.add(shootingWayPoint10);
            waypoints.add(shootingWayPoint11);
            waypoints.add(shootingWayPoint12);
            waypoints.add(shootingWayPoint13);
            waypoints.add(shootingWayPoint14);
            waypoints.add(shootingWayPoint15);
            waypoints.add(shootingWayPoint16);
            waypoints.add(shootingWayPoint17);
            waypoints.add(shootingPoint);

            return generateTrajectory(false, waypoints, Arrays.asList(), kMaxVelocity, kMaxAccel, kMaxDecel,
                    kMaxVoltage, 20.0, 1);
        }
    }
}
