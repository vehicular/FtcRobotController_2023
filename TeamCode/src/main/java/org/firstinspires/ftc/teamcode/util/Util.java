package org.firstinspires.ftc.teamcode.util;

public class Util
{
    /**
     * Forces a number to be between a min and max by trimming it to the max or min if the number
     * is to great in magnitude.
     *
     * @param num
     * @param min
     * @param max
     * @return
     */
    public static double trim(double num, double min, double max)
    {
        if (num < min) return min;
        if (num > max) return max;
        return num;
    }
    
    public static boolean inRange(double myValue, double min, double max)
    {
        if(myValue >= min && myValue <= max)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /**
     * Returns 0 if the number is within a threshold of 0.
     *
     * @param num
     * @param deadband
     * @return
     */
    public static double applyDeadband(double num, double deadband)
    {
        if (num < deadband && num > -deadband)
            return 0;
        return num;
    }
}