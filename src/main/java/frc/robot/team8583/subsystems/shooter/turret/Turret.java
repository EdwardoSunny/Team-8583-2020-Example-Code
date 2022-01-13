package frc.robot.team8583.subsystems.shooter.turret;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team8583.Constants;
import frc.robot.team8583.devices.Ahrs;
import frc.robot.team8583.devices.DriverInterface;
import frc.robot.team8583.drivers.Vision;
import frc.robot.team8583.drivers.StatefulXboxController;
import frc.robot.team8583.loops.ILooper;
import frc.robot.team8583.loops.Loop;
import frc.robot.team8583.subsystems.Subsystem;
import frc.robot.team8583.subsystems.shooter.turret.states.TurretDisabled;
import frc.robot.team8583.subsystems.shooter.turret.states.TurretInactive;
import frc.robot.team8583.subsystems.shooter.turret.states.TurretInitializing;
import frc.robot.team8583.subsystems.shooter.turret.states.TurretState;

public class Turret extends Subsystem
{
    private class TimestampedRotationState
    {
        public Rotation2d heading;
        public Rotation2d angularVelocity;
        public double timestamp;

        public TimestampedRotationState(Rotation2d heading, Rotation2d angularVelocity, double timestamp)
        {
            this.heading = heading;
            this.angularVelocity = angularVelocity;
            this.timestamp = timestamp;
        }
    }

    private class VisualTarget
    {
        public boolean isVisible = false;
        public Translation2d orientation = new Translation2d();
        public double latency = 0.0;
        public double timestamp = Double.NaN;
    }

    private class PeriodicInput
    {
        public Rotation2d ahrsHeading = new Rotation2d();
        public Rotation2d ahrsAngularVelocity = new Rotation2d();
        public LinkedList<TimestampedRotationState> ahrsYawRotationStateHistory = new LinkedList<TimestampedRotationState>();
        public VisualTarget visualTarget = new VisualTarget();
    }

    private TurretState state;
    private final PeriodicInput periodicInput = new PeriodicInput();
    private final Vision vision = new Vision();
    private final YawGimbal yawGimbal = new YawGimbal();
    private final PitchGimbal pitchGimbal = new PitchGimbal();
    private final Ahrs ahrs = Ahrs.getInstance();

    public Turret()
    {
        setState(new TurretDisabled(this));
    }

    public synchronized void setState(TurretState state)
    {
        this.state = state;
    }

    public TurretState getState()
    {
        return state;
    }

    public synchronized void update(double timestamp)
    {
        state.update();
    }

    public synchronized void activate()
    {
        setState(new TurretInitializing(this));
    }

    public synchronized void activate(Rotation2d initialHeading, boolean isFieldCentric)
    {
        setState(new TurretInitializing(this, initialHeading, isFieldCentric));
    }

    public synchronized void enableVision()
    {
        vision.enable();
    }

    public synchronized void reenableVision()
    {
        clearAhrsYawRotationStateHistory();
        clearVisualTarget();
        enableVision();
    }

    public synchronized void disableVision()
    {
        vision.disable();
    }

    public synchronized void stopYawGimbal()
    {
        yawGimbal.stop();
    }

    public synchronized void stopPitchGimbal()
    {
        pitchGimbal.stop();
    }

    private synchronized void updateAhrsYawRotationStateHistory(double timestamp)
    {
        periodicInput.ahrsYawRotationStateHistory
                .addLast(new TimestampedRotationState(yawGimbal.getFieldCentricHeading(periodicInput.ahrsHeading),
                        yawGimbal.getFieldCentricAngularVelocity(periodicInput.ahrsAngularVelocity), timestamp));
        while (periodicInput.ahrsYawRotationStateHistory
                .size() > Constants.Shooter.Turret.ROTATION_STATE_HISTORY_MAX_LENGTH)
        {
            periodicInput.ahrsYawRotationStateHistory.removeFirst();
        }
    }

    private synchronized void clearAhrsYawRotationStateHistory()
    {
        periodicInput.ahrsYawRotationStateHistory = new LinkedList<TimestampedRotationState>();
    }

    public synchronized void clearVisualTarget()
    {
        periodicInput.visualTarget = new VisualTarget();
    }

    public boolean hasVisualTarget()
    {
        return periodicInput.visualTarget.isVisible;
    }

    public boolean lostVisualTarget()
    {
        return !hasVisualTarget() && (periodicInput.visualTarget.timestamp == Double.NaN || Timer.getFPGATimestamp()
                - periodicInput.visualTarget.timestamp >= Constants.Shooter.Turret.LOSE_TARGET_TIME_THRESHOLD);
    }

    private Translation2d compensatedVisualTargetOrientationForDepth(Translation2d orientation)
    {
        return orientation
                .translateBy(Translation2d.fromPolar(orientation.direction(), Constants.Field.VISUAL_TARGET_DEPTH));
    }

    private synchronized void updateVisualTarget()
    {
        periodicInput.visualTarget.isVisible = vision.hasTarget();
        if (periodicInput.visualTarget.isVisible)
        {
            double newLatency = vision.getLatency();
            Translation2d newOrientation = vision.getTargetOrientation(
                    Rotation2d.fromDegrees(Constants.Shooter.Turret.Vision.CAMERA_ANGLE_OF_ELEVATION),
                    Constants.Shooter.Turret.Vision.CAMERA_HEIGHT, Constants.Field.VISUAL_TARGET_VISUAL_CENTER_HEIGHT)
                    .rotateBy(Rotation2d.fromDegrees(Constants.Shooter.Turret.Vision.CAMERA_HORIZONTAL_ANGULAR_OFFSET));
            if (Constants.Shooter.Turret.APPLY_DEPTH_COMPENSATION)
            {
                newOrientation = compensatedVisualTargetOrientationForDepth(newOrientation);
            }
            if (newLatency != periodicInput.visualTarget.latency
                    || !newOrientation.equals(periodicInput.visualTarget.orientation)
                    || periodicInput.visualTarget.timestamp == Double.NaN)
            {
                periodicInput.visualTarget.timestamp = Timer.getFPGATimestamp();
                periodicInput.visualTarget.latency = newLatency;
                periodicInput.visualTarget.orientation = newOrientation;
            }
        }
    }

    private Rotation2d linearInterpolateRotation(double targetTimestamp,
            TimestampedRotationState olderTimestampedRotationState,
            TimestampedRotationState newerTimestampedRotationState)
    {
        double normalizedTime = (targetTimestamp - olderTimestampedRotationState.timestamp)
                / (newerTimestampedRotationState.timestamp - olderTimestampedRotationState.timestamp);
        return olderTimestampedRotationState.heading.interpolate(newerTimestampedRotationState.heading, normalizedTime);
    }

    private Rotation2d cubicInterpolateRotationCubic(double targetTimestamp,
            TimestampedRotationState olderTimestampedRotationState,
            TimestampedRotationState newerTimestampedRotationState)
    {
        double period = newerTimestampedRotationState.timestamp - olderTimestampedRotationState.timestamp;
        double normalizedTime = (targetTimestamp - olderTimestampedRotationState.timestamp) / period;
        double pos0 = olderTimestampedRotationState.heading.getRadians();
        double pos1 = newerTimestampedRotationState.heading.getRadians();
        double vel0 = olderTimestampedRotationState.angularVelocity.getRadians() * period;
        double vel1 = newerTimestampedRotationState.angularVelocity.getRadians() * period;
        double[] coefs = new double[4];
        coefs[0] = pos0;
        coefs[1] = vel0;
        coefs[2] = -3.0 * pos0 + 3.0 * pos1 - 2.0 * vel0 - vel1;
        coefs[3] = 2.0 * pos0 - 2.0 * pos1 + vel0 + vel1;
        double sum = 0.0;
        double pow = 1.0;
        for (int i = 0; i < 4; i++)
        {
            sum += coefs[i] * pow;
            pow *= normalizedTime;
        }
        return Rotation2d.fromRadians(sum);
    }

    private Rotation2d getDelayCompensatedFieldCentricVisualTargetHeading()
    {
        Rotation2d compensatedFieldCentricYawHeading = yawGimbal.getFieldCentricHeading(periodicInput.ahrsHeading);
        double targetTimestamp = periodicInput.visualTarget.timestamp - periodicInput.visualTarget.latency
                - Constants.Looper.DELTA_TIME / 2.0 - Constants.Shooter.Turret.Vision.LATENCY_OFFSET;
        TimestampedRotationState olderTimestampedYawState = null;
        TimestampedRotationState newerTimestampedYawState = null;
        while (!periodicInput.ahrsYawRotationStateHistory.isEmpty())
        {
            olderTimestampedYawState = newerTimestampedYawState;
            newerTimestampedYawState = periodicInput.ahrsYawRotationStateHistory.removeFirst();
            if (newerTimestampedYawState.timestamp >= targetTimestamp)
            {
                if (olderTimestampedYawState == null)
                {
                    periodicInput.ahrsYawRotationStateHistory.addFirst(newerTimestampedYawState);
                    compensatedFieldCentricYawHeading = newerTimestampedYawState.heading;
                }
                else
                {
                    periodicInput.ahrsYawRotationStateHistory.addFirst(newerTimestampedYawState);
                    periodicInput.ahrsYawRotationStateHistory.addFirst(olderTimestampedYawState);
                    if (Constants.Shooter.Turret.APPLY_CUBIC_INTERPOLATION)
                    {
                        compensatedFieldCentricYawHeading = cubicInterpolateRotationCubic(targetTimestamp,
                                olderTimestampedYawState, newerTimestampedYawState);
                    }
                    else
                    {
                        compensatedFieldCentricYawHeading = linearInterpolateRotation(targetTimestamp,
                                olderTimestampedYawState, newerTimestampedYawState);
                    }
                }
                break;
            }
        }
        return compensatedFieldCentricYawHeading.rotateBy(periodicInput.visualTarget.orientation.direction());
    }

    public Translation2d getLaunchVector()
    {
        double xAccel = Constants.Shooter.Turret.AVERAGE_DRAG_INDUCED_X_ACCELERATION;
        double yAccel = Constants.Physics.G + Constants.Shooter.Turret.AVERAGE_DRAG_INDUCED_Y_ACCELERATION;
        double distance = periodicInput.visualTarget.orientation.norm();
        double heightDifference = Constants.Field.VISUAL_TARGET_CENTER_HEIGHT
                - Constants.Shooter.Turret.Vision.CAMERA_HEIGHT;
        double yVel = Math.sqrt(2.0 * yAccel * heightDifference);
        double xVel = (heightDifference * xAccel + distance * yAccel) / yVel;
        return new Translation2d(xVel, yVel);
    }

    //Try to compensate the influence of translation velocity
    public Translation2d getLaunchVectorWithDistanceCompensation(double velocity)
    {
        double xAccel = Constants.Shooter.Turret.AVERAGE_DRAG_INDUCED_X_ACCELERATION;
        double yAccel = Constants.Physics.G + Constants.Shooter.Turret.AVERAGE_DRAG_INDUCED_Y_ACCELERATION;
        double distance = periodicInput.visualTarget.orientation.norm();
        xAccel = xAccel + distance * 0.1;
        yAccel = yAccel + distance * 0.1;

        double heightDifference = Constants.Field.VISUAL_TARGET_CENTER_HEIGHT
                - Constants.Shooter.Turret.Vision.CAMERA_HEIGHT;
        double yVel = Math.sqrt(2.0 * yAccel * heightDifference);
        double xVel = (heightDifference * xAccel + distance * yAccel) / yVel;
        return new Translation2d(xVel, yVel);
    }

    public Translation2d getLaunchVectorDimension1()
    {
        double kFraction = 0.5;
        double distance = periodicInput.visualTarget.orientation.norm();
        double heightDifference = Constants.Field.VISUAL_TARGET_CENTER_HEIGHT
                - Constants.Shooter.Turret.Vision.CAMERA_HEIGHT;
        double xVel = (-kFraction * distance) * (-(Math.pow(kFraction, 2) * heightDifference / Constants.Physics.G)
                + (Math.pow(kFraction, 4) * Math.pow(heightDifference, 2) / (2 * Math.pow(Constants.Physics.G, 2))));
        double yVel = (Constants.Physics.G / kFraction)
                * (1 - (Math.pow(kFraction, 2) * heightDifference / Constants.Physics.G)
                        - (Math.pow(kFraction, 4) * Math.pow(heightDifference, 2) / (2 * Constants.Physics.G)));
        return new Translation2d(xVel, yVel);
    }

    public synchronized void setRobotCentricYawTarget(Rotation2d heading)
    {
        yawGimbal.setRobotCentricHeadingTarget(heading);
    }

    public synchronized void setFieldCentricYawTarget(Rotation2d heading)
    {
        yawGimbal.setFieldCentricHeadingTarget(heading, periodicInput.ahrsHeading);
    }

    public synchronized void setPitchTarget(Rotation2d heading)
    {
        pitchGimbal.setHeadingTarget(heading);
    }

    public synchronized void followVisualTarget()
    {
        if (hasVisualTarget())
        {
            setFieldCentricYawTarget(getDelayCompensatedFieldCentricVisualTargetHeading());
            setPitchTarget(getLaunchVector().direction());
        }
        else
        {
            yawGimbal.stop();
            pitchGimbal.stop();
        }
    }

    public boolean yawOnTarget()
    {
        return yawGimbal.headingOnTarget();
    }

    public boolean pitchOnTarget()
    {
        return pitchGimbal.headingOnTarget();
    }

    public double getPitchPosition()
    {
        return pitchGimbal.getHeading().getDegrees();
    }

    public boolean readyToLaunch()
    {
        return state.readyToLaunch();
    }

    @Override
    public synchronized void stop()
    {
        vision.disable();
        yawGimbal.stop();
        pitchGimbal.stop();
    }

    @Override
    public synchronized void disable()
    {
        yawGimbal.disable();
        pitchGimbal.disable();
    }

    @Override
    public synchronized void readPeriodicInputs()
    {
        double timestamp = Timer.getFPGATimestamp();

        yawGimbal.readPeriodicInputs();
        pitchGimbal.readPeriodicInputs();

        periodicInput.ahrsHeading = ahrs.getRobotHeading();
        periodicInput.ahrsAngularVelocity = ahrs.getRobotAngularVelocity();
        updateAhrsYawRotationStateHistory(timestamp);
        updateVisualTarget();
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        yawGimbal.writePeriodicOutputs();
        pitchGimbal.writePeriodicOutputs();
    }

    @Override
    public synchronized void resetSensors()
    {
        yawGimbal.resetSensors();
        pitchGimbal.resetSensors();

        clearAhrsYawRotationStateHistory();
        clearVisualTarget();
    }

    @Override
    public void logToSmartDashboard()
    {
        SmartDashboard.putString("Shooter Turret State", getState().toString());
        SmartDashboard.putBoolean("Shooter Turret Has Visual Target", hasVisualTarget());
        SmartDashboard.putString("Shooter Turret Visual Target Orientation",
                periodicInput.visualTarget.orientation.toString());
        SmartDashboard.putBoolean("Shooter Turret Heading On Target", yawOnTarget() && pitchOnTarget());

        yawGimbal.logToSmartDashboard();
        pitchGimbal.logToSmartDashboard();
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        Turret thisInstance = this;
        Loop loop = new Loop()
        {
            @Override
            public void onStart(double timestamp)
            {
                setState(new TurretInactive(thisInstance));
            }

            @Override
            public void onLoop(double timestamp)
            {
                update(timestamp);
            }

            @Override
            public void onStop(double timestamp)
            {
                stop();
            }
        };
        enabledLooper.register(loop);
    }

    @Override
    public boolean selfTest()
    {
        boolean passTest = true;
        passTest &= yawGimbal.selfTest();
        passTest &= pitchGimbal.selfTest();
        return passTest;
    }

    // ---------------- FOR TESTING ----------------

    // private final StatefulXboxController operatorController = DriverInterface.getCodriverController(); // for testing

    public void test()
    {
        // yawGimbal.setRobotCentricHeadingTarget((Timer.getFPGATimestamp() / 8.0 % 1.0 * 2.0 - 1.0) * (Timer.getFPGATimestamp() / 8.0 % 2.0 < 1.0 ? 1.0 : -1.0) * 90);

        /*
        if (operatorController.getButton(ButtonId.BUTTON_Y).isBeingPressed())
        {
            pitchGimbal.setHeadingTarget(new Rotation2d(Constants.Shooter.Turret.PitchGimbal.LAUNCH_ANGLE_MIN));
        }
        else if (operatorController.getButton(ButtonId.BUTTON_B).isBeingPressed())
        {
            pitchGimbal.setHeadingTarget(new Rotation2d(Constants.Shooter.Turret.PitchGimbal.LAUNCH_ANGLE_MAX));
        }
        else
        {
            pitchGimbal.stop();
        }
        */

        /*
        pitchGimbal.setHeadingTarget(new Rotation2d((Constants.Shooter.Turret.PitchGimbal.LAUNCH_ANGLE_MAX
                - Constants.Shooter.Turret.PitchGimbal.LAUNCH_ANGLE_MIN) / 2 * operatorController.getLeftY()
                + ((Constants.Shooter.Turret.PitchGimbal.LAUNCH_ANGLE_MAX
                        + Constants.Shooter.Turret.PitchGimbal.LAUNCH_ANGLE_MIN) / 2)));
                        */

        /*vision.enableLighting();
        if (vision.hasTarget())
        {
            Rotation2d test = vision.getTargetOrientation(
                    Rotation2d.fromDegrees(Constants.Shooter.Turret.Vision.CAMERA_ANGLE_OF_ELEVATION),
                    Constants.Shooter.Turret.Vision.CAMERA_HEIGHT, Constants.Field.VISUAL_TARGET_VISUAL_CENTER_HEIGHT)
                    .direction();
            System.out.println("test" + test.getDegrees());
            followVisualTarget();
            //yawGimbal.setRobotCentricHeadingTarget(yawGimbal.getRobotCentricHeading().rotateBy(test));
        }
        */

        //double tgt2 = -operatorController.getLeftX() * 0.5;
        //System.out.println("tgt2:" + tgt2);
        //yawGimbal.setOpenLoop(tgt);

        //yawGimbal.setRobotCentricHeadingTarget(180 - operatorController.getRightX() * 90.0);

        /*
        yawGimbal.setFieldCentricHeadingTarget(Rotation2d.fromDegrees(180 - operatorController.getRightX() * 90.0),
                ahrs.getRobotHeading());
        */

        /*
        yawGimbal.setRobotCentricHeadingTarget(
                yawGimbal.getRobotCentricHeading().rotateBy(vision.getTargetOrientation()));
        */

        /*
        yawGimbal.setFieldCentricHeadingTarget(
                yawGimbal.getFieldCentricHeading(periodicInput.robotHeading).rotateBy(vision.getTargetOrientation()), periodicInput.robotHeading);
        */

        /*
        yawGimbal.setFieldCentricHeadingTarget(getCompensatedFieldCentricVisualTargetOrientation(),
                periodicInput.robotHeading);
        */

        // followVisualTarget();

        SmartDashboard.putNumber("Shooter Turret Field Centric Yaw",
                yawGimbal.getFieldCentricAngularVelocity(periodicInput.ahrsHeading).getDegrees());
    }
}
