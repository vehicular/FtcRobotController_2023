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

    public final static int LifterMotorInt = 0;
    public final static int RotatorMotorInt = 1;
    public final static int ArmMotorInt = 2;
    public final static int WristServoInt = 3;
    public final static int PalmServoInt = 4;
    public final static int KnuckleServoInt = 5;
    public final static int FingerServoInt = 6;
    public final static int maxNumOfMotors = 7; // must be the last one

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
                    Integer.parseInt(data[LifterMotorInt]);
            RotatorMotor =
                    Integer.parseInt(data[RotatorMotorInt]);
            ArmMotor =
                    Integer.parseInt(data[ArmMotorInt]);
            WristServo =
                    Double.parseDouble(data[WristServoInt]);
            PalmServo =
                    Double.parseDouble(data[PalmServoInt]);
            KnuckleServo =
                    Double.parseDouble(data[KnuckleServoInt]);
            FingerServo =
                    Double.parseDouble(data[FingerServoInt]);
        }
    }

    public StepPosition InitPosition = new StepPosition();

    public StepPosition Pickup_up = new StepPosition();
    public StepPosition Pickup_down = new StepPosition();
    public StepPosition Pickup_left = new StepPosition();
    public StepPosition Pickup_right = new StepPosition();

    public StepPosition Drop_A_1 = new StepPosition(); // 0-10 inches
    public StepPosition Drop_B_2 = new StepPosition(); // 10-20 inches
    public StepPosition Drop_X_3 = new StepPosition(); // 20-30 inches
    public StepPosition Drop_Y_4 = new StepPosition(); // 30-40 inches

    public MotorPositionCal()
    {}

}
