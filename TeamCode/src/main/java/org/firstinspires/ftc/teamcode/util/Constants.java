package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Here, store the names for each of the motors in the hardware map,
 * store the pid constants for each of the loops,
 * and store any other constants that are used in the code.
 */
public class Constants
{
    public static String fingerServo = "fingerservo3";
    public static String knuckleServo = "knuckleservo2";
    public static String palmServo = "palmservo1";
    public static String wristServo = "wristservo0";

    public static String leftbackMotor = "lbmotor1";
    public static String rightbackMotor = "rbmotor3";
    public static String leftfrontMotor = "lfmotor0";
    public static String rightfrontMotor = "rfmotor2";

    public static String lifterMotor = "liftermotor0";
    public static String rotatorMotor = "rotatormotor1";
    public static String armMotor = "armmotor2";

    public static String imu = "imu";
    public static String frontColorSensor = "leftcolor0";
    public static String frontWebcamera = "Webcam 1";

    public static double TICKS_TO_INCHES = 1;
    public static int topClimber = -25000;
    public static int bottomClimber= 25000;
    public static int landingNumber = 5000;



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
