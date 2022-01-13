/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.team254.lib.trajectory.TrajectoryGenerator;
import frc.robot.team8583.auto.AutoModeExecuter;
import frc.robot.team8583.auto.SmartDashboardInteractions;
import frc.robot.team8583.devices.Ahrs;
import frc.robot.team8583.devices.DriverInterface;
import frc.robot.team8583.devices.PneumaticCompressor;
import frc.robot.team8583.devices.VideoFeed;
import frc.robot.team8583.lib.util.CrashTracker;
import frc.robot.team8583.lib.util.Logger;
import frc.robot.team8583.loops.Looper;
import frc.robot.team8583.subsystems.SubsystemGroup;
import frc.robot.team8583.subsystems.climber.Climber;
import frc.robot.team8583.subsystems.collector.Collector;
import frc.robot.team8583.subsystems.collector.states.CollectorManual;
import frc.robot.team8583.subsystems.shooter.Shooter;
import frc.robot.team8583.subsystems.shooter.states.ShooterDisabled;
import frc.robot.team8583.subsystems.swerve.Swerve;
import frc.robot.team8583.subsystems.swerve.states.SwerveManual;
import frc.robot.team8583.testauto.TestAuto;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
    private final Looper enabledLooper = new Looper();
    private final Looper disabledLooper = new Looper();
    private final DriverStation driverStation = DriverStation.getInstance();

    private SmartDashboardInteractions smartDashboardInteractions = new SmartDashboardInteractions();
    private AutoModeExecuter autoModeExecuter = null;

    private Swerve swerve;
    private Shooter shooter;
    private Collector collector;
    private Climber climber;
    private SubsystemGroup subsystems;

    private TrajectoryGenerator trajGenerator;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit()
    {
        Logger.clearLog();
        LiveWindow.disableAllTelemetry();

        try
        {
            initDevices();
            initSubsystems();
            initAuto();
            smartDashboardInteractions.initWithDefaults();
        } catch (Throwable t)
        {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    private void initDevices()
    {
        Ahrs.getInstance();
        PneumaticCompressor.getInstance().enable();
        // VideoFeed.getInstance().enable();
    }

    private void initSubsystems()
    {
        swerve = Swerve.getInstance();
        shooter = Shooter.getInstance();
        collector = Collector.getInstance();
        climber = Climber.getInstance();
        subsystems = new SubsystemGroup(Arrays.asList(swerve, collector, climber, shooter));//, shooter, collector, climber));
        subsystems.registerEnabledLoops(enabledLooper);
        subsystems.registerDisabledLoops(disabledLooper);
        if (subsystems.selfTest()) // Temporary implementation, should use unified console print interface
        {
            System.out.println("Subsystem self-test PASSED");
        }
        else
        {
            System.out.println("Subsystem self-test FAILED");
        }
    }

    private void initAuto()
    {
        trajGenerator = TrajectoryGenerator.getInstance();
        trajGenerator.generateTrajectories();
    }

    public void allPeriodic()
    {
        logToSmartDashboard();
        subsystems.logToSmartDashboard();
        enabledLooper.logToSmartDashboard();
    }

    public void logToSmartDashboard()
    {
        if (Constants.Log.ENABLE_DEBUG_OUTPUT)
        {
            SmartDashboard.putBoolean("Enabled", driverStation.isEnabled());
            SmartDashboard.putNumber("Match Time", driverStation.getMatchTime());
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic()
    {
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit()
    {
        try
        {
            disabledLooper.stop();
            enabledLooper.start();

            SmartDashboard.putBoolean("Auto", true);

            autoModeExecuter = new AutoModeExecuter();
            autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode());
            autoModeExecuter.start();

        } catch (Throwable t)
        {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic()
    {
        try
        {
            allPeriodic();
        } catch (Throwable t)
        {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit()
    {
        try
        {
            if (autoModeExecuter != null)
            {
                autoModeExecuter.stop();
            }
            disabledLooper.stop();
            enabledLooper.restart();
            swerve.setState(new SwerveManual(swerve));
        } catch (Throwable t)
        {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic()
    {
        try
        {
            DriverInterface.update();
            allPeriodic();
        } catch (Throwable t)
        {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit()
    {
        teleopInit();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic()
    {
        teleopPeriodic();
    }
}
