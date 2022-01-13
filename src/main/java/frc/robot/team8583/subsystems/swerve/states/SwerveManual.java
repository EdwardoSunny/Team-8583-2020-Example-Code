package frc.robot.team8583.subsystems.swerve.states;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.team254.lib.geometry.Translation2d;
import frc.robot.team8583.Constants;
import frc.robot.team8583.devices.DriverInterface;
import frc.robot.team8583.drivers.StatefulXboxController;
import frc.robot.team8583.drivers.StatefulXboxController.ButtonId;
import frc.robot.team8583.lib.util.Util;
import frc.robot.team8583.subsystems.swerve.Swerve;

public class SwerveManual extends SwerveState
{
    private final StatefulXboxController driverController = DriverInterface.getDriverController();

    public SwerveManual(Swerve swerve)
    {
        super(swerve);
    }

    @Override
    public synchronized void update(double timestamp)
    {
        Translation2d translationalInput = getXboxControllerTranslationalInput();
        double rotationalInput = getXboxControllerRotationalInput();
        if (Constants.Swerve.ENABLE_HEADING_STABILIZATION && Util.epsilonEquals(rotationalInput, 0.0))
        {
            if (!swerve.headingControllerIsEnabled() && Math.abs(swerve.getAngularVelocity()
                    .getUnboundedDegrees()) <= Constants.Swerve.HEADING_STABILIZATION_ANGULAR_VELOCITY_THRESHOLD)
            {
                swerve.setTargetHeadingToCurrentHeading();
                swerve.enableHeadingController();
            }
        }
        else
        {
            swerve.disableHeadingController();
        }
        swerve.updateNormalizedVectorialVelocityControl(translationalInput, rotationalInput, true, true, timestamp);
    }

    private Translation2d getXboxControllerTranslationalInput()
    {
        Translation2d input = Util
                .applyRemappedCircularDeadband(
                        new Translation2d(-driverController.getY(Hand.kRight), -driverController.getX(Hand.kRight)),
                        Constants.Swerve.TRANSLATION_INPUT_CIRCULAR_DEADBAND)
                .scale(Constants.Swerve.TRANSLATIONAL_SPEED_FACTOR);
        if (driverController.getButton(ButtonId.TRIGGER_LEFT).isBeingPressed())
        {
            input = input.scale(1.0 - Constants.Swerve.SLOW_MODE_TRANSLATIONAL_SPEED_ATTENUATION_FACTOR);
        }
        return input;
    }

    private double getXboxControllerRotationalInput()
    {
        double input = Util.applyRemappedDeadband(-driverController.getX(Hand.kLeft),
                Constants.Swerve.ROTATION_INPUT_DEADBAND) * Constants.Swerve.ROTATIONAL_SPEED_FACTOR;
        if (driverController.getButton(ButtonId.TRIGGER_LEFT).isBeingPressed())
        {
            input *= (1.0 - Constants.Swerve.SLOW_MODE_ROTATIONAL_SPEED_ATTENUATION_FACTOR);
        }
        return input;
    }

    @Override
    public String toString()
    {
        return "Manual Control";
    }
}
