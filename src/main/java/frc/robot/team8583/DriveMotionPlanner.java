package frc.robot.team8583;

import java.util.ArrayList;
import java.util.List;

import frc.robot.team254.lib.geometry.Pose2d;
import frc.robot.team254.lib.geometry.Pose2dWithCurvature;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team254.lib.trajectory.DistanceView;
import frc.robot.team254.lib.trajectory.Trajectory;
import frc.robot.team254.lib.trajectory.TrajectoryIterator;
import frc.robot.team254.lib.trajectory.TrajectorySamplePoint;
import frc.robot.team254.lib.trajectory.TrajectoryUtil;
import frc.robot.team254.lib.trajectory.timing.TimedState;
import frc.robot.team254.lib.trajectory.timing.TimingConstraint;
import frc.robot.team254.lib.trajectory.timing.TimingUtil;
import frc.robot.team254.lib.util.CSVWritable;
import frc.robot.team254.lib.util.Util;
import frc.robot.team8583.lib.util.HeadingController;
import frc.robot.team8583.lib.util.TranslationAxisController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionPlanner implements CSVWritable
{
    private static final double kMaxDx = Constants.Swerve.TrajectoryGenerator.kMaxDx;
    private static final double kMaxDy = Constants.Swerve.TrajectoryGenerator.kMaxDy;
    private static final double kMaxDTheta = Constants.Swerve.TrajectoryGenerator.kMaxDTheta;

    private static final double kMinTranslationV = Constants.Swerve.DriveMotionPlanner.kMinTranslationVelocity;
    private static final double kRotationFactor = Constants.Swerve.DriveMotionPlanner.kRotationFactor;

    private double defaultCook = 0.5;
    private boolean useDefaultCook = true;
    private boolean headingOntarget = false;

    private Translation2d followingCenter = Translation2d.identity();
    private double goalHeading = 0.0;

    private TranslationAxisController xController = new TranslationAxisController(
            Constants.Swerve.TranslationController.X.KP, Constants.Swerve.TranslationController.X.KI,
            Constants.Swerve.TranslationController.X.KD,
            Constants.Swerve.TranslationController.X.CONTROL_ERROR_TOLERANCE);

    private TranslationAxisController yController = new TranslationAxisController(
            Constants.Swerve.TranslationController.Y.KP, Constants.Swerve.TranslationController.Y.KI,
            Constants.Swerve.TranslationController.Y.KD,
            Constants.Swerve.TranslationController.Y.CONTROL_ERROR_TOLERANCE);

    private HeadingController thetaController = new HeadingController(Constants.Swerve.HeadingController.Auto.KP,
            Constants.Swerve.HeadingController.Auto.KI, Constants.Swerve.HeadingController.Auto.KD,
            Constants.Swerve.HeadingController.Auto.CONTROL_ERROR_TOLERANCE);

    public enum FollowerType
    {
        PURE_PURSUIT, PID_CONTROL
    }

    FollowerType mFollowerType = FollowerType.PID_CONTROL;

    public void setFollowerType(FollowerType type)
    {
        mFollowerType = type;
    }

    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;

    public Trajectory<TimedState<Pose2dWithCurvature>> getTrajectory()
    {
        return mCurrentTrajectory.trajectory();
    }

    public double getRemainingProgress()
    {
        if (mCurrentTrajectory != null)
        {
            return mCurrentTrajectory.getRemainingProgress();
        }
        return 0.0;
    }

    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();
    Pose2d mOutput = Pose2d.identity();
    double currentTrajectoryLength = 0.0;

    double mDt = 0.0;

    /**
     * getMaxRotationSpeed: calculate max speed according to the progress of current trajactory
     * @return 
     */
    public double getMaxRotationSpeed()
    {
        final double kStartPoint = 0.2;
        final double kPivotPoint = 0.5;
        final double kEndPoint = 0.8;
        final double kMaxSpeed = 1.0;
        double normalizedProgress = mCurrentTrajectory.getProgress() / currentTrajectoryLength;
        double scalar = 0.0;
        if (kStartPoint <= normalizedProgress && normalizedProgress <= kEndPoint)
        {
            if (normalizedProgress <= kPivotPoint)
            {
                scalar = (normalizedProgress - kStartPoint) / (kPivotPoint - kStartPoint);
            }
            else
            {
                scalar = 1.0 - ((normalizedProgress - kPivotPoint) / (kEndPoint - kPivotPoint));
            }
        }

        return kMaxSpeed * scalar;
    }

    public DriveMotionPlanner()
    {
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory)
    {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        defaultCook = trajectory.trajectory().defaultVelocity();
        currentTrajectoryLength = trajectory.trajectory().getLastState().t();
        for (int i = 0; i < trajectory.trajectory().length(); ++i)
        {
            if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon)
            {
                mIsReversed = false;
                break;
            }
            else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon)
            {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset()
    {
        mError = Pose2d.identity();
        mOutput = Pose2d.identity();
        mLastTime = Double.POSITIVE_INFINITY;
        useDefaultCook = true;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints, double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_decel, double max_voltage, double default_vel, int slowdown_chunks)
    {
        return generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_decel,
                max_voltage, default_vel, slowdown_chunks);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
            final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel, double end_vel, double max_vel, // inches/s
            double max_accel, // inches/s^2
            double max_decel, double max_voltage, double default_vel, int slowdown_chunks)
    {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
        // TODO re-architect the spline generator to support reverse.
        if (reversed)
        {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            for (int i = 0; i < waypoints.size(); ++i)
            {
                waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
            }
        }

        // Create a trajectory from splines.
        //Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil
        //        .trajectoryFromSplineWaypoints(waypoints_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        //now create a trajectory directly from waypoints
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromWaypoints(waypoints_maybe_flipped);

        if (reversed)
        {
            List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i)
            {
                flipped.add(new Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip),
                        -trajectory.getState(i).getCurvature(), trajectory.getState(i).getDCurvatureDs()));
            }
            trajectory = new Trajectory<>(flipped);
        }

        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        //final CurvatureVelocityConstraint velocity_constraints = new CurvatureVelocityConstraint();
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        //all_constraints.add(velocity_constraints);
        if (constraints != null)
        {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory(reversed,
                new DistanceView<>(trajectory), kMaxDx, all_constraints, start_vel, end_vel, max_vel, max_accel,
                max_decel, slowdown_chunks);

        /*for (int i = 0; i < timed_trajectory.length(); i++)
        {
            System.out.println("timed_trajectory point " + i + ":" + timed_trajectory.getPoint(i).state().toString());
        }*/
        System.out.println("timed_trajectory point " + (timed_trajectory.length() - 1) + ":"
                + timed_trajectory.getPoint(timed_trajectory.length() - 1).state().toString());

        //timed_trajectory.setDefaultVelocity(
        //        default_vel / Constants.Swerve.DriveMotionPlanner.kSwerveMaxDriveSpeedInchesPerSecond);
        return timed_trajectory;
    }

    /**
     * @param followingCenter the followingCenter to set (relative to the robot's center)
     */
    public void setFollowingCenter(Translation2d followingCenter)
    {
        this.followingCenter = followingCenter;
    }

    public void setGoalHeading(double goalHeading)
    {
        this.goalHeading = goalHeading;
    }

    @Override
    public String toCSV()
    {
        return mOutput.toCSV();
    }

    protected Pose2d updatePidPositionControl(double timestamp, Pose2d current_state)
    {
        //set target point and calculate output from controller
        if (!xController.isEnabled())
        {
            xController.enable();
        }

        if (!yController.isEnabled())
        {
            yController.enable();
        }

        if (!thetaController.isEnabled())
        {
            thetaController.enable();
        }

        //look ahead search target point coordinate from trajectory
        double lookahead_time = Constants.Swerve.DriveMotionPlanner.kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;
        TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();

        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());

        while (actual_lookahead_distance < Constants.Swerve.DriveMotionPlanner.kPathMinLookaheadDistance
                && mCurrentTrajectory.getRemainingProgress() > lookahead_time)
        {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }

        System.out.println("Lookahead state:" + lookahead_state.state() + "\n Setpoint state:" + mSetpoint.state()
                + "\n Current state:" + current_state.toString());

        Rotation2d finalHeading = new Rotation2d(goalHeading);
        xController.setTarget(lookahead_state.state().getTranslation().x());
        yController.setTarget(lookahead_state.state().getTranslation().y());
        //thetaController.setTargetHeading(lookahead_state.state().getRotation());
        thetaController.setTargetHeading(finalHeading);

        double vx = xController.calculate(current_state.getTranslation().x(), timestamp) / lookahead_time;
        double vy = yController.calculate(current_state.getTranslation().y(), timestamp) / lookahead_time;
        double vtheta = thetaController.calculate(current_state.getRotation(), timestamp) * kRotationFactor;

        Translation2d translationVector;

        translationVector = new Translation2d(vx, vy);
        final Rotation2d rotationDegree = new Rotation2d(vtheta);

        final Pose2d controlVector = new Pose2d(translationVector, rotationDegree);
        System.out.println("\nPid updated, vector is: " + translationVector.toString() + "\n rotation:"
                + rotationDegree.getDegrees());
        return controlVector;
    }

    protected Pose2d updatePurePursuit(Pose2d current_state)
    {
        double lookahead_time = Constants.Swerve.DriveMotionPlanner.kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;
        TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());

        while (actual_lookahead_distance < Constants.Swerve.DriveMotionPlanner.kPathMinLookaheadDistance
                && mCurrentTrajectory.getRemainingProgress() > lookahead_time)
        {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }

        if (actual_lookahead_distance < Constants.Swerve.DriveMotionPlanner.kPathMinLookaheadDistance)
        {
            lookahead_state = new TimedState<>(
                    new Pose2dWithCurvature(lookahead_state.state().getPose()
                            .transformBy(Pose2d.fromTranslation(new Translation2d((mIsReversed ? -1.0 : 1.0)
                                    * (Constants.Swerve.DriveMotionPlanner.kPathMinLookaheadDistance
                                            - actual_lookahead_distance),
                                    0.0))),
                            0.0),
                    lookahead_state.t(), lookahead_state.velocity(), lookahead_state.acceleration());
            System.out.println("\n Lookahead pose:" + lookahead_state.state().getPose().toString()
                    + "\n Lookahead velocity:" + lookahead_state.velocity() + "\n lookahead accleration:"
                    + lookahead_state.acceleration());
        }

        SmartDashboard.putNumber("Path X", lookahead_state.state().getTranslation().x());
        SmartDashboard.putNumber("Path Y", lookahead_state.state().getTranslation().y());
        SmartDashboard.putNumber("Path Velocity",
                lookahead_state.velocity() / Constants.Swerve.DriveMotionPlanner.kSwerveMaxDriveSpeedInchesPerSecond);

        Translation2d lookaheadTranslation = new Translation2d(current_state.getTranslation(),
                lookahead_state.state().getTranslation());

        Rotation2d steeringDirection = lookaheadTranslation.direction();
        //double normalizedSpeed = Math.abs(mSetpoint.velocity())
        //       / Constants.Swerve.DriveMotionPlanner.kSwerveMaxDriveSpeedInchesPerSecond;
        double normalizedSpeed = lookahead_state.velocity();

        System.out.println("Lookahead point: " + lookahead_state.state().getTranslation().toString()
                + " Current State: " + current_state.getTranslation().toString() + " Lookahad translation: "
                + lookaheadTranslation.toString());

        System.out.println("Speed: " + normalizedSpeed + " DefaultCook: " + defaultCook + " setpoint t:" + mSetpoint.t()
                + " Length: " + currentTrajectoryLength);

        /*if (normalizedSpeed > defaultCook || mSetpoint.t() > (currentTrajectoryLength / 2.0))
        {
            useDefaultCook = false;
        }
        if (useDefaultCook)
        {
            normalizedSpeed = defaultCook;
        }*/

        final Translation2d steeringVector = Translation2d.fromPolar(steeringDirection, normalizedSpeed);
        final Rotation2d rotationDegree = lookahead_state.state().getRotation()
                .rotateBy(current_state.getRotation().inverse());

        final Pose2d controlVector = new Pose2d(steeringVector, rotationDegree);

        /*System.out.println("\nPure pursuit updated, vector is: " + steeringVector.toString() + "\n rotation:"
                + rotationDegree.getDegrees());*/
        return controlVector;
    }

    public Pose2d update(double timestamp, Pose2d current_state)
    {
        if (mCurrentTrajectory == null)
        {
            //System.out.println("Trajectory is null, returning zero trajectory");
            return Pose2d.identity();
        }
        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime))
        {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;

        current_state = current_state.transformBy(Pose2d.fromTranslation(followingCenter));

        double searchStepSize = 1.0;
        double previewQuantity = 0.0;
        double searchDirection = 1.0;
        double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
        double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
        searchDirection = Math.signum(reverseDistance - forwardDistance);
        while (searchStepSize > 0.001)
        {
            if (Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.001))
                break;
            while (/* next point is closer than current point */ distance(current_state,
                    previewQuantity + searchStepSize * searchDirection) < distance(current_state, previewQuantity))
            {
                /* move to next point */
                previewQuantity += searchStepSize * searchDirection;
            }
            searchStepSize /= 10.0;
            searchDirection *= -1;
        }

        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory
                .advance(previewQuantity);
        mSetpoint = sample_point.state();
        SmartDashboard.putNumber("Path X", mSetpoint.state().getTranslation().x());
        SmartDashboard.putNumber("Path Y", mSetpoint.state().getTranslation().y());
        SmartDashboard.putNumber("Path Velocity", mSetpoint.velocity());

        if (!mCurrentTrajectory.isDone())
        {
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());
            System.out.println("Error:" + mError.toString());
            System.out.println("remainingProgress:" + mCurrentTrajectory.getRemainingProgress());
            if (mCurrentTrajectory.getRemainingProgress() < Constants.Swerve.DriveMotionPlanner.kPathLookaheadTime)
            {
                sample_point = mCurrentTrajectory.advance(Constants.Swerve.DriveMotionPlanner.kPathLookaheadTime);
                mSetpoint = sample_point.state();
            }

            if (mFollowerType == FollowerType.PURE_PURSUIT)
            {
                mOutput = updatePurePursuit(current_state);
            }
            else if (mFollowerType == FollowerType.PID_CONTROL)
            {
                mOutput = updatePidPositionControl(timestamp, current_state);
            }
        }
        else
        {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = Pose2d.identity();
            // System.out.println("Motion planner done, returning zero trajectory");
        }
        return mOutput;
    }

    private double distance(Pose2d current_state, double additional_progress)
    {
        return mCurrentTrajectory.preview(additional_progress).state().state().getPose().distance(current_state);
    }

    public boolean isDone()
    {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public Pose2d error()
    {
        return mError;
    }

    public TimedState<Pose2dWithCurvature> setpoint()
    {
        return mSetpoint;
    }
}
