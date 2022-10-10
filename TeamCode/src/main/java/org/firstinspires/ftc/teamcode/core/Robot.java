package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.subsystems.ChassisTank;
import org.firstinspires.ftc.teamcode.core.subsystems.Hand;

import java.util.ArrayList;

/**
 * This class will store static instances of each of the modules.
 */
public class Robot
{
    public boolean initialized = false;

    public ChassisTank chassis;
    public Hand hand;

    public ArrayList<Subsystem> modules = new ArrayList<Subsystem>();

    /**
     * Initialize subsystems and add them to the modules list.
     *
     * @param hwMap
     */
    public Robot(HardwareMap hwMap)
    {
        chassis = new ChassisTank(hwMap);
        hand = new Hand(hwMap);

        //Robot.chassis.startAccelerationIntegration();
        //Robot.chassis.teleopInit();
        //Robot.hand.teleopInit();

        modules.add(chassis);
        modules.add(hand);

        initialized = true;
    }
}
