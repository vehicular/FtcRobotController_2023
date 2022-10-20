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


    public static final int LIFTER_POSITION_LOWEST = 0;
    public static final int LIFTER_POSITION_HIGHEST = 1000;

    public static final int ROTATOR_POSITION_CENTER = 0;
    public static final int ROTATOR_POSITION_MOST_LEFT = 500;
    public static final int ROTATOR_POSITION_MOST_RIGHT = -500;

    public static final int ARM_POSITION_HIGHEST = 1200;  // max position
    public static final int ARM_POSITION_P_45D = 250;     // 45 degree point up
    public static final int ARM_POSITION_HORIZONTAL = 0;  // horizontal, init
    public static final int ARM_POSITION_N_45D = -250;    // 45 degree point down
    public static final int ARM_POSITION_LOWEST = -330;   // min position

    public static final double WRIST_POSITION_MAX = 1.0;
    public static final double WRIST_POSITION_ARM_P45D = 0.8;  // angle up 45 degree with arm
    public static final double WRIST_POSITION_WITH_ARM = 0.7;  // flat with arm
    public static final double WRIST_POSITION_WITH_N45D = 0.6; // angle down 45 degree with arm
    public static final double WRIST_POSITION_MIN = 0;

    public static final double PALM_POSITION_MAX = 1.0;
    public static final double PALM_POSITION_WITH_WRIST = 0.7; // flat with wrist
    public static final double PALM_POSITION_MIN = 0;

    public static final double KNUCKLE_POSITION_LEFT_MAX = 0;
    public static final double KNUCKLE_POSITION_LEFT = 0.2;
    public static final double KNUCKLE_POSITION_CENTER = 0.4;
    public static final double KNUCKLE_POSITION_RIGHT = 0.6;
    public static final double KNUCKLE_POSITION_RIGHT_MAX = 0.8;

    public static final double FINGER_POSITION_CLOSE = 1.0;
    public static final double FINGER_POSITION_OPEN = 0;


}
