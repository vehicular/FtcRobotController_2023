package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;

/**
 * This is the PID class to use for the robot.
 */
public class PID
{
    private long timeStamp, prevTimeStamp;
    private double kP, kI, kD, kF;
    private double error, previousError;
    private double deadband = .05;
    private double min = -1, max = 1;
    private double setpoint = 0;

    public PID(double kP, double kI, double kD, double kF)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        previousError = 0;
    }

    /**
     * Calculates the next output for the pid, then returns the output.
     *
     * @return
     */
    public double update(double input)
    {
        error = setpoint - input;
        double output = calculateP() - calculateD();
        error = previousError;
        return Util.trim(output, min, max);
    }

    /**
     * @return the error multiplied by the kP term.
     */
    public double calculateP()
    {
        return kP * error;
    }

    /**
     * The rate of change for the error with respect to time.
     *
     * @return
     */
    public double calculateD()
    {
        return kD * ((error - previousError) / (double) (timeStamp - prevTimeStamp));
    }


    public double getSetpoint()
    {
        return setpoint;
    }


    //Getters and setters for variables.
    public double getP()
    {
        return kP;
    }

    public double getI()
    {
        return kI;
    }

    public double getD()
    {
        return kD;
    }

    public double getF()
    {
        return kF;
    }

    public double getMax()
    {
        return max;
    }

    public double getMin()
    {
        return min;
    }

    public void setP(double p)
    {
        kP = p;
    }

    public void setI(double i)
    {
        kI = i;
    }

    public void setD(double d)
    {
        kD = d;
    }

    public void setF(double f)
    {
        kF = f;
    }

    public void setMax(double max)
    {
        this.max = max;
    }

    public void setMin(double min)
    {
        this.min = min;
    }

    public void setSetpoint(double target)
    {
        setpoint = target;
    }
}
