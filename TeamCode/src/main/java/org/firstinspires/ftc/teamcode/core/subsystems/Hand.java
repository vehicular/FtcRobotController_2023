package org.firstinspires.ftc.teamcode.core.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Subsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Hand extends Subsystem
{
    private double pos = 0;

    private DcMotor lifterMotor;
    private DcMotor rotatorMotor;
    private DcMotor armMotor;
    private Servo wristServo;
    private Servo palmServo;
    private Servo knukcleSevo;
    private Servo finger;

    private boolean altControl;

    public Hand(HardwareMap map)
    {
        super(map);
        finger = map.servo.get(Constants.fingerServo);
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
        if (gamepad1.right_bumper) altControl = !altControl;
        if (altControl)
        {
            if (gamepad1.a) pos += 0.05;
            else if (gamepad1.y) pos -= 0.05;
            finger.setPosition(pos);
        } else
        {
            if (gamepad1.a) finger.setPosition(0);
            else if (gamepad1.b) finger.setPosition(0.5);
            else if (gamepad1.y) finger.setPosition(1);
        }
    }

    @Override
    public String addTelemetry()
    {
        String s = "Finger Position: \t" + finger.getPosition();
        return s;
    }

    @Override
    public void stop()
    {

    }
    @Override
    public void autoInit()
    {

    }

    public void setTargetPosition(int position)
    {
        finger.setPosition(position);
    }
@Override
    public void teleopInit(){}
}
