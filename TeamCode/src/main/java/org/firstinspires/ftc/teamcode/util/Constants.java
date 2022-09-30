package org.firstinspires.ftc.teamcode.util;

/**
 * Here, store the names for each of the motors in the hardware map,
 * store the pid constants for each of the loops,
 * and store any other constants that are used in the code.
 */
public class Constants
{
    public static String elevatorMotor = "liftmotor";
    public static String intakeFlipper = "intakeflipper";
    public static String liftFlipper = "liftflipper";
    public static String intakeMotor = "intakemotor";
    public static String leftDrive = "LFMotor";
    public static String rightDrive = "RFMotor";
    public static String climber = "climber";
    public static double TICKS_TO_INCHES = 1;
    public static int topClimber = -25000;
    public static int bottomClimber= 25000;
    public static int landingNumber = 5000;
    public static int middleClimber = 0;

    /**
     * To wirelessly connect to phone,
     * plug in via USB]
     * In terminal use commands
     * adb devices
     * adb tcpip 5555
     * adb connect <IP address of phone>
     * adb disconnect <IP adress of phone>
     */
}
