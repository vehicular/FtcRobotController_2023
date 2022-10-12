package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class MotorPositionCal
{
    public final static String LifterMotorStr = "LifterMotor";
    public final static String RotatorMotorStr = "RotatorMotor";
    public final static String ArmMotorStr = "ArmMotor";
    public final static String WristServoStr = "WristServo";
    public final static String PalmServoStr = "PalmServo";
    public final static String KnuckleServoStr = "KnuckleServo";
    public final static String FingerServoStr = "FingerServo";

    public class StepPosition {
        public int LifterMotor;
        public int RotatorMotor;
        public int ArmMotor;
        public double WristServo;
        public double PalmServo;
        public double KnuckleServo;
        public double FingerServo;

        public StepPosition()
        {}
        public String toString()
        {
            return String.format(Locale.getDefault(), "%7d : %7d : %7d : %2f : %2f : %2f : %2f",
                LifterMotor,
                RotatorMotor,
                ArmMotor,
                WristServo,
                PalmServo,
                KnuckleServo,
                FingerServo
            );
        }

        public void SetValue(String[] data)
        {
            LifterMotor =
                    Integer.parseInt(data[0]);
            RotatorMotor =
                    Integer.parseInt(data[1]);
            ArmMotor =
                    Integer.parseInt(data[2]);
            WristServo =
                    Double.parseDouble(data[3]);
            PalmServo =
                    Double.parseDouble(data[4]);
            KnuckleServo =
                    Double.parseDouble(data[5]);
            FingerServo =
                    Double.parseDouble(data[6]);
        }
    }

    public StepPosition InitPosition = new StepPosition();

    public StepPosition Pickup_up = new StepPosition();
    public StepPosition Pickup_down = new StepPosition();
    public StepPosition Pickup_left = new StepPosition();
    public StepPosition Pickup_right = new StepPosition();

    public StepPosition Drop_A = new StepPosition();
    public StepPosition Drop_B = new StepPosition();
    public StepPosition Drop_X = new StepPosition();
    public StepPosition Drop_y = new StepPosition();

    public MotorPositionCal()
    {}

}
