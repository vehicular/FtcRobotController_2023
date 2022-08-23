package org.firstinspires.ftc.teamcode.core.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Constants;

public class LiftFlipper extends Subsystem
{
    //private double pos = flipper.setPosition(0);
    private Servo flipper;
    private double pos;
    private boolean altControl;

    public LiftFlipper(HardwareMap map)
    {
        super(map);
        flipper = hardwaremap.servo.get(Constants.liftFlipper);
        pos = 0;
        altControl = false;
    }

    /**
     * Runs repeatedly during teleop. The right bumper toggles between control modes.
     * In the first control mode, 3 buttons move the flipper between 3 positions.
     * In the second control mode, the 2 buttons move theh flipper up and down.
     *
     * @param gamepad1
     * @param gamepad2
     */
    @Override
    public void teleopControls(Gamepad gamepad1, Gamepad gamepad2)
    {
        if (gamepad2.left_bumper) altControl = !altControl;

        if (altControl)
        {
            if (gamepad2.dpad_up) pos += 0.05;
            else if (gamepad2.dpad_down) pos -= 0.05;
            flipper.setPosition(pos);
        } else
        {
            if (gamepad2.dpad_down) flipper.setPosition(0);
            else if (gamepad2.dpad_left) flipper.setPosition(0.5);
            else if (gamepad2.dpad_up) flipper.setPosition(1);
        }
    }

    @Override
    public String addTelemetry()
    {
        String s = "Lift flipper \n\t" + flipper.getPosition();
        s += "\n " + altControl;
        return s;
    }

    @Override
    public void stop()
    {

    }

    public void autoInit()
    {

    }

    public void setTargetPosition(int position)
    {
        flipper.setPosition(position);
    }
}
