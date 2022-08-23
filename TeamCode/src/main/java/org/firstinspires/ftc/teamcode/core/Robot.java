package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.core.subsystems.Climber;
import org.firstinspires.ftc.teamcode.core.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.core.subsystems.IntakeFlipper;
import org.firstinspires.ftc.teamcode.core.subsystems.IntakeMotor;
import org.firstinspires.ftc.teamcode.core.subsystems.LiftFlipper;
import org.firstinspires.ftc.teamcode.core.subsystems.Subsystem;

import java.util.ArrayList;

/**
 * This class will store static instances of each of the modules.
 */
public class Robot
{
    public static boolean initialized = false;

    public static Chassis chassis;
    public static Elevator elevator;
    public static IntakeFlipper intakeFlipper;
    public static LiftFlipper liftFlipper;
    public static IntakeMotor intakeMotor;
    public static Climber climber;

    public static ArrayList<Subsystem> modules = new ArrayList<Subsystem>();

    /**
     * Initialize subsystems and add them to the modules list.
     *
     * @param hwMap
     */
    public Robot(HardwareMap hwMap)
    {
        chassis = new Chassis(hwMap);
        elevator = new Elevator(hwMap);
        intakeFlipper = new IntakeFlipper(hwMap);
        liftFlipper = new LiftFlipper(hwMap);
        intakeMotor = new IntakeMotor(hwMap);
        climber = new Climber(hwMap);


        modules.add(chassis);
        modules.add(elevator);
        modules.add(intakeFlipper);
        modules.add(liftFlipper);
        modules.add(intakeMotor);
        modules.add(climber);

        initialized = true;
    }
}
