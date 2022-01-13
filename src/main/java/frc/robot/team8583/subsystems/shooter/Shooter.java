package frc.robot.team8583.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team8583.devices.PneumaticCompressor;
import frc.robot.team8583.loops.ILooper;
import frc.robot.team8583.loops.Loop;
import frc.robot.team8583.subsystems.Subsystem;
import frc.robot.team8583.subsystems.shooter.spindex.Spindex;
import frc.robot.team8583.subsystems.shooter.states.ShooterAuto;
import frc.robot.team8583.subsystems.shooter.states.ShooterDisabled;
import frc.robot.team8583.subsystems.shooter.states.ShooterManual;
import frc.robot.team8583.subsystems.shooter.states.ShooterState;
import frc.robot.team8583.subsystems.shooter.turret.Turret;

public class Shooter extends Subsystem
{
    private static Shooter instance = null;

    public static Shooter getInstance()
    {
        if (instance == null)
        {
            instance = new Shooter();
        }
        return instance;
    }

    private ShooterState state;
    private Translation2d launchVector = null;
    private boolean shouldIgnoreFlywheelSurfaceSpeed = false;
    private final Turret turret = new Turret();
    private final BallDrive ballDrive = new BallDrive();
    private final BallFeeder ballFeeder = new BallFeeder();
    private final Spindex spindex = new Spindex();
    private final PneumaticCompressor compressor = PneumaticCompressor.getInstance();

    private Shooter()
    {
        setState(new ShooterDisabled(this));
    }

    public synchronized void setState(ShooterState state)
    {
        this.state = state;
    }

    public ShooterState getState()
    {
        return state;
    }

    public synchronized void update(double timestamp)
    {
        state.update();
    }

    public synchronized void stopTurret()
    {
        turret.stop();
    }

    public synchronized void activateTurret()
    {
        turret.activate();
        launchVector = null;
    }

    public synchronized void activateTurret(Rotation2d initialHeading, boolean isFieldCentric)
    {
        turret.activate(initialHeading, isFieldCentric);
        launchVector = null;
    }

    public synchronized void setBallDriveReverse()
    {
        ballDrive.setFlywheelReverse();
    }

    public synchronized void stopBallDrive()
    {
        ballDrive.stop();
    }

    public synchronized void setBallFeederInject()
    {
        ballFeeder.setInject();
    }

    public synchronized void setBallFeederEject()
    {
        ballFeeder.setEject();
    }

    public synchronized void stopBallFeeder()
    {
        ballFeeder.stop();
    }

    public synchronized void startSpindex()
    {
        spindex.startSpin();
    }

    public synchronized void toggleSpindex()
    {
        spindex.toggleSpin();
    }

    public synchronized void enableSpindexAntiJam()
    {
        spindex.enableAntiJam();
    }

    public synchronized void disableSpindexAntiJam()
    {
        spindex.disableAntiJam();
    }

    public synchronized void stopSpindex()
    {
        spindex.stop();
    }

    public synchronized void setAutoLaunch(boolean launchState)
    {
        ShooterAuto shooterAutoState = new ShooterAuto(this);
        setState(shooterAutoState);
        shooterAutoState.setLaunchState(launchState);
    }

    public synchronized void setLaunch(boolean triggerState)
    {
        if (turret.readyToLaunch())
        {
            spindex.startSpin();
            spindex.accelerateSpin();
            if (triggerState)
            {
                turret.stopYawGimbal();
                turret.stopPitchGimbal();
                compressor.disable();
                if (launchVector == null)
                {
                    launchVector = turret.getLaunchVector();
                }
                ballDrive.setFlywheelSurfaceSpeedTargetByProjectileSpeed(launchVector.norm());
                if (ballDrive.flywheelSurfaceSpeedOnTarget() || shouldIgnoreFlywheelSurfaceSpeed)
                {
                    shouldIgnoreFlywheelSurfaceSpeed = true;
                    setBallFeederInject();
                }
                else
                {
                    stopBallFeeder();
                }
            }
            else
            {
                stopBallDrive();
                stopBallFeeder();
                compressor.enable();
                launchVector = turret.getLaunchVector();
                shouldIgnoreFlywheelSurfaceSpeed = false;
            }
        }
        else
        {
            stopBallDrive();
            stopBallFeeder();
            spindex.decelerateSpin();
            compressor.enable();
            shouldIgnoreFlywheelSurfaceSpeed = false;
        }
    }

    @Override
    public synchronized void stop()
    {
        setState(new ShooterDisabled(this));
    }

    @Override
    public synchronized void readPeriodicInputs()
    {
        turret.readPeriodicInputs();
        ballDrive.readPeriodicInputs();
        ballFeeder.readPeriodicInputs();
        spindex.readPeriodicInputs();
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        turret.writePeriodicOutputs();
        ballDrive.writePeriodicOutputs();
        ballFeeder.writePeriodicOutputs();
        spindex.writePeriodicOutputs();
    }

    @Override
    public synchronized void resetSensors()
    {
        turret.resetSensors();
    }

    @Override
    public void logToSmartDashboard()
    {
        SmartDashboard.putString("Shooter State", getState().toString());

        turret.logToSmartDashboard();
        ballDrive.logToSmartDashboard();
        ballFeeder.logToSmartDashboard();
        spindex.logToSmartDashboard();
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        Shooter thisInstance = this;
        Loop loop = new Loop()
        {
            @Override
            public void onStart(double timestamp)
            {
                setState(new ShooterManual(thisInstance));
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
        turret.registerEnabledLoops(enabledLooper);
        spindex.registerEnabledLoops(enabledLooper);
    }

    @Override
    public boolean selfTest()
    {
        boolean passesTest = true;
        passesTest &= turret.selfTest();
        passesTest &= ballDrive.selfTest();
        passesTest &= ballFeeder.selfTest();
        passesTest &= spindex.selfTest();
        return passesTest;
    }
}
