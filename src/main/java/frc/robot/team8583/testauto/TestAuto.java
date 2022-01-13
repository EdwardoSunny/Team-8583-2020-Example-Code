package frc.robot.team8583.testauto;

import frc.robot.team8583.subsystems.shooter.Shooter;
import frc.robot.team8583.subsystems.shooter.states.ShooterDisabled;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.team254.lib.geometry.Rotation2d;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team8583.loops.ILooper;
import frc.robot.team8583.loops.Loop;
import frc.robot.team8583.subsystems.collector.Collector;
import frc.robot.team8583.subsystems.collector.states.CollectorDisabled;
import frc.robot.team8583.subsystems.swerve.Swerve;
import frc.robot.team8583.subsystems.swerve.states.SwerveInitializing;

public class TestAuto
{
    private int stage = 0;
    private double prevStateTime = 0.0;
    private Swerve swerve = Swerve.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Collector collector = Collector.getInstance();

    private void init()
    {
        swerve.setState(new SwerveInitializing(swerve, Rotation2d.fromDegrees(180.0), true));
        shooter.setState(new ShooterDisabled(shooter));
        collector.setState(new CollectorDisabled(collector));
        prevStateTime = Timer.getFPGATimestamp();
    }

    private void run()
    {
        switch (stage)
        {
        case 0:
        {
            swerve.disableHeadingController();
            // swerve.setTargetHeadingToCurrentHeading();
            shooter.activateTurret();
            stage++;
            break;
        }

        case 1:
        {
            nextStageAfter(0.6);
            break;
        }

        case 2:
        {
            shooter.setLaunch(true);
            nextStageAfter(3.0);
            break;
        }

        case 3:
        {
            shooter.stop();
            collector.setElevate(false);
            collector.setInject();
            setSwerveFieldCentricVectorialMove(-75.0, 0);
            nextStageAfter(2.5);
            break;
        }

        case 4:
        {
            swerve.stop();
            nextStageAfter(0.1);
            break;
        }

        case 5:
        {
            swerve.disableHeadingController();
            // swerve.setTargetHeadingToCurrentHeading();
            stage++;
            break;
        }

        case 6:
        {
            setSwerveFieldCentricVectorialMove(100.0, 0);
            nextStageAfter(1.8);
            break;
        }

        case 7:
        {
            swerve.stop();
            shooter.activateTurret();
            stage++;
            break;
        }

        case 8:
        {
            nextStageAfter(0.6);
            break;
        }

        case 9:
        {
            shooter.setLaunch(true);
            nextStageAfter(4.0);
            break;
        }

        case 10:
        {
            collector.stop();
            collector.setElevate(true);
            nextStageAfter(2.0);
            break;
        }

        default:
        {
            stop();
            break;
        }
        }
    }

    private void stop()
    {
        swerve.stop();
        shooter.stop();
        collector.stop();
    }

    /*
    private void run()
    {
        switch (stage)
        {
        case 0:
        {
            swerve.enableHeadingController();
            swerve.setTargetHeadingToCurrentHeading();
            shooter.activateTurret();
            stage++;
            break;
        }
    
        case 1:
        {
            nextStageAfter(0.6);
            break;
        }
    
        case 2:
        {
            shooter.setLaunch(true);
            nextStageAfter(3.0);
            break;
        }
    
        case 3:
        {
            shooter.stop();
            collector.setElevate(false);
            collector.setInject();
            setSwerveFieldCentricVectorialMove(-87.5, 0);
            nextStageAfter(3.0);
            break;
        }
    
        case 4:
        {
            swerve.stop();
            nextStageAfter(0.1);
            break;
        }
    
        case 5:
        {
            swerve.enableHeadingController();
            swerve.setTargetHeadingToCurrentHeading();
            stage++;
            break;
        }
    
        case 6:
        {
            setSwerveFieldCentricVectorialMove(150.0, 0);
            nextStageAfter(1.3);
            break;
        }
    
        case 7:
        {
            swerve.stop();
            shooter.activateTurret();
            stage++;
            break;
        }
    
        case 8:
        {
            nextStageAfter(0.5);
            break;
        }
    
        case 9:
        {
            shooter.setLaunch(true);
            nextStageAfter(4.0);
            break;
        }
    
        case 10:
        {
            collector.stop();
            collector.setElevate(true);
            nextStageAfter(2.0);
            break;
        }
    
        default:
        {
            stop();
            break;
        }
        }
    }
    
    private void stop()
    {
        swerve.stop();
        shooter.stop();
        collector.stop();
    }
    */

    private void nextStageAfter(double duration)
    {
        double time = Timer.getFPGATimestamp();
        if (time - prevStateTime > duration)
        {
            prevStateTime = time;
            stage++;
        }
    }

    private void setSwerveFieldCentricVectorialMove(double x, double y)
    {
        swerve.updateVectorialVelocityControl(new Translation2d(x, y), 0.0, true, Timer.getFPGATimestamp());
    }

    public void registerAutoLoop(ILooper autoLoop)
    {
        Loop loop = new Loop()
        {
            @Override
            public synchronized void onStart(double timestamp)
            {
                init();
            }

            @Override
            public synchronized void onLoop(double timestamp)
            {
                run();
            }

            @Override
            public synchronized void onStop(double timestamp)
            {
                stop();
            }
        };
        autoLoop.register(loop);
    }
}
