package frc.robot.team8583.subsystems.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team254.lib.geometry.Pose2d;
import frc.robot.team254.lib.geometry.Pose2dWithCurvature;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team254.lib.trajectory.Trajectory;
import frc.robot.team254.lib.trajectory.timing.TimedState;
import frc.robot.team8583.Constants;
import frc.robot.team8583.Ports;
import frc.robot.team8583.devices.Ahrs;
import frc.robot.team8583.lib.util.HeadingController;
import frc.robot.team8583.lib.util.SwerveInverseKinematics;
import frc.robot.team8583.lib.util.SwerveKinematics;
import frc.robot.team8583.lib.util.Util;
import frc.robot.team8583.loops.ILooper;
import frc.robot.team8583.loops.Loop;
import frc.robot.team8583.subsystems.Subsystem;
import frc.robot.team8583.subsystems.swerve.states.SwerveTrajectory;
import frc.robot.team8583.subsystems.swerve.states.SwerveDisabled;
import frc.robot.team8583.subsystems.swerve.states.SwerveManual;
import frc.robot.team8583.subsystems.swerve.states.SwerveNeutral;
import frc.robot.team8583.subsystems.swerve.states.SwerveState;

public class Swerve extends Subsystem
{
    private static Swerve instance = null;

    public static synchronized Swerve getInstance()
    {
        if (instance == null)
        {
            instance = new Swerve();
        }
        return instance;
    }

    private SwerveState state;
    private Pose2d startingPose = new Pose2d();
    private final PeriodicInput periodicInput = new PeriodicInput();
    private final SwerveInverseKinematics inverseKinematics = new SwerveInverseKinematics(Constants.Swerve.MODULE_COUNT,
            Constants.Swerve.DriveModule.POSITIONS_RELATIVE_TO_DRIVE_CENTER);
    private final SwerveKinematics kinematics = new SwerveKinematics(
            Constants.Swerve.Odometer.DISTANCE_DEVIANCE_THRESHOLD);
    private final HeadingController headingController = new HeadingController(
            Constants.Swerve.HeadingController.Manual.KP, Constants.Swerve.HeadingController.Manual.KI,
            Constants.Swerve.HeadingController.Manual.KD,
            Math.toRadians(Constants.Swerve.HeadingController.Manual.CONTROL_ERROR_TOLERANCE));
    private final SwerveDriveModule frontRightModule, frontLeftModule, rearLeftModule, rearRightModule;
    private final List<SwerveDriveModule> modules;
    private final List<SwerveDriveModule> odometerModules;
    private final Ahrs ahrs = Ahrs.getInstance();

    private class PeriodicInput
    {
        public Rotation2d ahrsHeading = new Rotation2d();
        public Rotation2d ahrsAngularVelocity = new Rotation2d();
    }

    private Swerve()
    {
        frontLeftModule = new SwerveDriveModule(0, Ports.Can.FRONT_LEFT_DRIVE_MOTOR,
                Ports.Can.FRONT_LEFT_ROTATION_MOTOR, Ports.Can.FRONT_LEFT_ROTATION_SENSOR,
                Constants.Swerve.DriveModule.Rotation.FRONT_LEFT_CALIBRATION_OFFSET,
                Constants.Swerve.DriveModule.FRONT_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);
        rearLeftModule = new SwerveDriveModule(1, Ports.Can.REAR_LEFT_DRIVE_MOTOR, Ports.Can.REAR_LEFT_ROTATION_MOTOR,
                Ports.Can.REAR_LEFT_ROTATION_SENSOR, Constants.Swerve.DriveModule.Rotation.REAR_LEFT_CALIBRATION_OFFSET,
                Constants.Swerve.DriveModule.REAR_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);
        rearRightModule = new SwerveDriveModule(2, Ports.Can.REAR_RIGHT_DRIVE_MOTOR,
                Ports.Can.REAR_RIGHT_ROTATION_MOTOR, Ports.Can.REAR_RIGHT_ROTATION_SENSOR,
                Constants.Swerve.DriveModule.Rotation.REAR_RIGHT_CALIBRATION_OFFSET,
                Constants.Swerve.DriveModule.REAR_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);
        frontRightModule = new SwerveDriveModule(3, Ports.Can.FRONT_RIGHT_DRIVE_MOTOR,
                Ports.Can.FRONT_RIGHT_ROTATION_MOTOR, Ports.Can.FRONT_RIGHT_ROTATION_SENSOR,
                Constants.Swerve.DriveModule.Rotation.FRONT_RIGHT_CALIBRATION_OFFSET,
                Constants.Swerve.DriveModule.FRONT_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);
        modules = Arrays.asList(frontLeftModule, rearLeftModule, rearRightModule, frontRightModule);
        odometerModules = modules;
        configModules();
        resetSensors();
        setState(new SwerveDisabled(this));
    }

    private synchronized void configModules()
    {
        frontRightModule.setInvertTranslationMotor(true);
        rearRightModule.setInvertTranslationMotor(true);
    }

    public synchronized void setState(SwerveState state)
    {
        this.state = state;
    }

    public SwerveState getState()
    {
        return state;
    }

    public synchronized void update(double timestamp)
    {
        updateOdometer(timestamp);
        state.update(timestamp);
    }

    public void enableHeadingController()
    {
        headingController.enable();
    }

    public void disableHeadingController()
    {
        headingController.disable();
    }

    public boolean headingControllerIsEnabled()
    {
        return headingController.isEnabled();
    }

    public void setTargetHeading(Rotation2d targetHeadingDegrees)
    {
        headingController.setTargetHeading(targetHeadingDegrees);
    }

    public void setTargetHeadingToCurrentHeading()
    {
        setTargetHeading(periodicInput.ahrsHeading);
    }

    public Rotation2d getTargetHeading()
    {
        return headingController.getTargetHeading();
    }

    public boolean headingOnTarget()
    {
        return headingController.onTarget();
    }

    public synchronized void updateVectorialVelocityControl(Translation2d translationVector, double rotationMagnitude,
            boolean isFieldCentric, double timestamp)
    {
        setModuleVelocityTargets(inverseKinematics.calculateModuleVelocities(translationVector,
                rotationMagnitude + headingController.calculate(periodicInput.ahrsHeading, timestamp), getPose(),
                isFieldCentric));
    }

    public synchronized void updateNormalizedVectorialVelocityControl(Translation2d translationVector,
            double rotationMagnitude, boolean isFieldCentric, boolean enableClosedLoopControl, double timestamp)
    {
        setNormalizedModuleVelocityTargets(inverseKinematics.calculateNormalizedModuleVelocities(translationVector,
                normalizeRotationMagnitude(
                        rotationMagnitude + headingController.calculate(periodicInput.ahrsHeading, timestamp)),
                getPose(), isFieldCentric), enableClosedLoopControl);
    }

    private double normalizeRotationMagnitude(double rotationMagnitude)
    {
        return rotationMagnitude / Constants.Swerve.WHEELBASE_HALF_DIAGONAL;
    }

    public synchronized void alignModules(Rotation2d heading, boolean isFieldCentric)
    {
        List<Rotation2d> headings = new ArrayList<>(modules.size());
        for (int i = 0; i < modules.size(); i++)
        {
            headings.add(isFieldCentric ? heading.rotateBy(periodicInput.ahrsHeading.inverse()) : heading);
        }
        setModuleHeadingTargets(headings);
    }

    public synchronized void alignModulesToOShape()
    {
        setModuleHeadingTargets(inverseKinematics.calculateModuleRotationHeadingsAlignedToRotationVectors());
    }

    public synchronized void alignModulesToXShape()
    {
        setModuleHeadingTargets(inverseKinematics
                .calculateModuleRotationHeadingsAlignedToRotationVectors(Rotation2d.fromDegrees(90.0)));
    }

    private synchronized void setModuleVelocityTargets(List<Translation2d> moduleVelocities)
    {
        for (int i = 0; i < modules.size(); i++)
        {
            SwerveDriveModule module = modules.get(i);
            Translation2d moduleVelocity = moduleVelocities.get(i);
            if (Util.shouldReverseRotation(moduleVelocities.get(i).direction().getDegrees(),
                    modules.get(i).getRobotCentricRotationHeading().getDegrees()))
            {
                if (!Util.epsilonEquals(moduleVelocity.norm(), 0.0))
                {
                    module.setRotationHeadingTarget(moduleVelocity.direction().rotateBy(Rotation2d.fromDegrees(180.0)));
                }
                module.setTranslationVelocityTarget(-moduleVelocity.norm());
            }
            else
            {
                if (!Util.epsilonEquals(moduleVelocity.norm(), 0.0))
                {
                    module.setRotationHeadingTarget(moduleVelocity.direction());
                }
                module.setTranslationVelocityTarget(moduleVelocity.norm());
            }
        }
    }

    private synchronized void setNormalizedModuleVelocityTargets(List<Translation2d> moduleVelocities,
            boolean enableClosedLoopControl)
    {
        for (int i = 0; i < modules.size(); i++)
        {
            SwerveDriveModule module = modules.get(i);
            Translation2d moduleVelocity = moduleVelocities.get(i);
            if (Util.shouldReverseRotation(moduleVelocities.get(i).direction().getDegrees(),
                    modules.get(i).getRobotCentricRotationHeading().getDegrees()))
            {
                if (!Util.epsilonEquals(moduleVelocity.norm(), 0.0))
                {
                    module.setRotationHeadingTarget(moduleVelocity.direction().rotateBy(Rotation2d.fromDegrees(180.0)));
                }

                if (enableClosedLoopControl)
                {
                    module.setNormalizedTranslationVelocityTarget(-moduleVelocity.norm());
                }
                else
                {
                    module.setTranslationOpenLoop(-moduleVelocity.norm());
                }
            }
            else
            {
                if (!Util.epsilonEquals(moduleVelocity.norm(), 0.0))
                {
                    module.setRotationHeadingTarget(moduleVelocity.direction());
                }

                if (enableClosedLoopControl)
                {
                    module.setNormalizedTranslationVelocityTarget(moduleVelocity.norm());
                }
                else
                {
                    module.setTranslationOpenLoop(moduleVelocity.norm());
                }
            }
        }
    }

    public boolean moduleVelocitiesOnTarget()
    {
        for (SwerveDriveModule module : modules)
        {
            if (!module.translationVelocityOnTarget())
            {
                return false;
            }
        }
        return true;
    }

    private synchronized void setModuleHeadingTargets(List<Rotation2d> moduleRotationHeadings)
    {
        for (int i = 0; i < modules.size(); i++)
        {
            if (Util.shouldReverseRotation(moduleRotationHeadings.get(i).getDegrees(),
                    modules.get(i).getRobotCentricRotationHeading().getDegrees()))
            {
                modules.get(i).setRotationHeadingTarget(
                        moduleRotationHeadings.get(i).rotateBy(Rotation2d.fromDegrees(180.0)));
            }
            else
            {
                modules.get(i).setRotationHeadingTarget(moduleRotationHeadings.get(i));
            }
        }
    }

    public boolean moduleHeadingsOnTarget()
    {
        for (SwerveDriveModule module : modules)
        {
            if (!module.rotationHeadingOnTarget())
            {
                return false;
            }
        }
        return true;
    }

    public synchronized void stopModules()
    {
        modules.forEach((module) -> module.stop());
    }

    public synchronized void disableModules()
    {
        modules.forEach((module) -> module.disable());
    }

    public synchronized void updateOdometer(double timestamp)
    {
        List<Pose2d> moduleEstimatedRobotPoses = new ArrayList<>();
        for (SwerveDriveModule module : odometerModules)
        {
            module.updateOdometer(periodicInput.ahrsHeading);
            moduleEstimatedRobotPoses.add(module.getEstimatedRobotPose());
        }
        kinematics.update(moduleEstimatedRobotPoses, periodicInput.ahrsHeading, timestamp);
        odometerModules.forEach((module) -> module.setModulePositionFromRobotPose(kinematics.getPose()));
    }

    public Pose2d getPose()
    {
        return kinematics.getPose();
    }

    public synchronized void setPose(Pose2d pose)
    {
        kinematics.setPose(pose);
        modules.forEach((module) -> module.setModulePositionFromRobotPose(pose));
    }

    public synchronized void resetPose()
    {
        setPose(startingPose);
        kinematics.resetTotalDistance();
    }

    public void setStartingPose(Pose2d pose)
    {
        startingPose = pose;
    }

    public Translation2d getVelocity()
    {
        return kinematics.getVelocity();
    }

    public Rotation2d getAngularVelocity()
    {
        return periodicInput.ahrsAngularVelocity;
    }

    public double getTotalDistance()
    {
        return kinematics.getTotalDistance();
    }

    public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
            Translation2d followingCenter, double goalHeading)
    {
        System.out.println("setTrajectory enter real class");
        Swerve thisInstance = this;
        SwerveTrajectory swerveTrajectory = new SwerveTrajectory(thisInstance);
        setState(swerveTrajectory);
        System.out.println("state:" + getState().toString());
        swerveTrajectory.setTrajectory(trajectory, followingCenter, goalHeading);
    }

    public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goalHeading)
    {
        System.out.println("setTrajectory enter swerve class");
        setTrajectory(trajectory, Translation2d.identity(), goalHeading);
    }

    @Override
    public synchronized void stop()
    {
        setState(new SwerveNeutral(this));
    }

    @Override
    public synchronized void disable()
    {
        setState(new SwerveDisabled(this));
    }

    @Override
    public synchronized void readPeriodicInputs()
    {
        periodicInput.ahrsHeading = ahrs.getRobotHeading();
        periodicInput.ahrsAngularVelocity = ahrs.getRobotAngularVelocity();
        modules.forEach((module) -> module.readPeriodicInputs());
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        modules.forEach((module) -> module.writePeriodicOutputs());
    }

    @Override
    public synchronized void resetSensors()
    {
        resetPose();
    }

    @Override
    public void logToSmartDashboard()
    {
        SmartDashboard.putString("Swerve State", getState().toString());
        SmartDashboard.putString("Robot Pose", getPose().toString());
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putString("Robot Velocity", getVelocity().toString());
            SmartDashboard.putNumber("Robot Velocity Magnitude", getVelocity().norm());
            SmartDashboard.putNumber("Total Distance Travelled", getTotalDistance());
            SmartDashboard.putNumber("Target Heading", getTargetHeading().getDegrees());
            SmartDashboard.putNumber("IMU Fused Heading", periodicInput.ahrsHeading.getDegrees());
        }

        modules.forEach((module) -> module.logToSmartDashboard());
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        Swerve thisInstance = this;
        Loop loop = new Loop()
        {
            @Override
            public synchronized void onStart(double timestamp)
            {
                setState(new SwerveManual(thisInstance));
            }

            @Override
            public synchronized void onLoop(double timestamp)
            {
                update(timestamp);
            }

            @Override
            public synchronized void onStop(double timestamp)
            {
                stop();
            }
        };
        enabledLooper.register(loop);
    }

    @Override
    public boolean selfTest()
    {
        boolean passesTest = true;
        for (SwerveDriveModule module : modules)
        {
            passesTest &= module.selfTest();
        }
        return passesTest;
    }
}
