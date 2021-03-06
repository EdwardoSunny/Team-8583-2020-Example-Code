/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.team8583.lib.util;

import frc.robot.team254.lib.geometry.Translation2d;

/**
 * Robot position controller for translation
 */
public class TranslationAxisController
{

    private boolean isEnabled = false;
    private double targetOupt;
    private double previousUpdateTimestamp = Double.NaN;
    private final SynchronousPIDF pidController;

    public TranslationAxisController(double kp, double ki, double kd, double errorTolerance)
    {
        pidController = new SynchronousPIDF(kp, ki, kd);
        pidController.setInputRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        pidController.setOutputRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        pidController.setDeadband(errorTolerance);
    }

    public synchronized void enable()
    {
        if (!isEnabled)
        {
            isEnabled = true;
            pidController.reset();
            previousUpdateTimestamp = Double.NaN;
        }
    }

    public synchronized void disable()
    {
        isEnabled = false;
    }

    public boolean isEnabled()
    {
        return isEnabled;
    }

    public synchronized void setInvert(boolean invert)
    {
        pidController.setInvert(invert);
    }

    public boolean isInverted()
    {
        return pidController.isInverted();
    }

    public void setInputRange(double minInput, double maxInput)
    {
        pidController.setInputRange(minInput, maxInput);
    }

    public void setOutputRange(double minOutput, double maxOutput)
    {
        pidController.setOutputRange(minOutput, maxOutput);
    }

    public synchronized void setTarget(double targetOupt)
    {
        this.targetOupt = targetOupt;
        pidController.setSetpoint(this.targetOupt);
    }

    public double getTarget()
    {
        return pidController.getSetpoint();
    }

    public synchronized double calculate(double input, double timestamp)
    {
        double output = 0.0;
        //System.out.println("input:" + input);
        if (isEnabled)
        {
            //One axis controller
            output = pidController.calculate(input, timestamp - previousUpdateTimestamp,
                    Double.isNaN(previousUpdateTimestamp));

        }
        return output;
    }
}
