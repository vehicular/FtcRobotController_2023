package org.firstinspires.ftc.teamcode.util;

public class MotorPosition {

    public int LifterMotor;
    public int RotatorMotor;
    public int ArmMotor;
    public double WristServo;
    public double PalmServo;
    public double KnuckleServo;
    public double FingerServo;

    public final static String LifterMotorStr = "LifterMotor";
    public final static String  RotatorMotorStr = "RotatorMotor";
    public final static String  ArmMotorStr = "ArmMotor";
    public final static String  WristServoStr = "WristServo";
    public final static String  PalmServoStr = "PalmServo";
    public final static String  KnuckleServoStr = "KnuckleServo";
    public final static String  FingerServoStr = "FingerServo";

    public MotorPosition(int LifterMotor,
            int RotatorMotor,
            int ArmMotor,
            double WristServo,
            double PalmServo,
            double KnuckleServo,
            double FingerServo)
    {
        this.LifterMotor = LifterMotor;
        this.RotatorMotor = RotatorMotor;
        this.ArmMotor = ArmMotor;
        this.WristServo = WristServo;
        this.PalmServo = PalmServo;
        this.KnuckleServo = KnuckleServo;
        this.FingerServo = FingerServo;
    }


}
