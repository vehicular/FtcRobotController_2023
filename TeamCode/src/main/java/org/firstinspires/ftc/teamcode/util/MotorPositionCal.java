package org.firstinspires.ftc.teamcode.util;

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

    public class SubsystemPosition {
        public int LifterMotor;
        public int RotatorMotor;
        public int ArmMotor;
        public double WristServo;
        public double PalmServo;
        public double KnuckleServo;
        public double FingerServo;

        public SubsystemPosition()
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

    public SubsystemPosition InitPosition = new SubsystemPosition();

    public SubsystemPosition Pickup_up = new SubsystemPosition();
    public SubsystemPosition Pickup_down = new SubsystemPosition();
    public SubsystemPosition Pickup_left = new SubsystemPosition();
    public SubsystemPosition Pickup_right = new SubsystemPosition();

    public SubsystemPosition Drop_A_1 = new SubsystemPosition(); // 0-10 inches
    public SubsystemPosition Drop_B_2 = new SubsystemPosition(); // 10-20 inches
    public SubsystemPosition Drop_X_3 = new SubsystemPosition(); // 20-30 inches
    public SubsystemPosition Drop_Y_4 = new SubsystemPosition(); // 30-40 inches

    public MotorPositionCal()
    {}

}
