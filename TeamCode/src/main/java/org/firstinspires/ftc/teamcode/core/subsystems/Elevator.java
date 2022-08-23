package org.firstinspires.ftc.teamcode.core.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Elevator extends Subsystem
{
    private DcMotor upAndDown;

    public Elevator(HardwareMap map)
    {
        super(map);
        upAndDown = map.dcMotor.get(Constants.elevatorMotor);
        upAndDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void teleopControls(Gamepad gamepad1, Gamepad gamepad2)
    {
        upAndDown.setPower(-gamepad2.right_stick_y);
    }

    @Override
    public void stop()
    {

    }

    @Override
    public String addTelemetry()
    {
        return "";
    }
}
