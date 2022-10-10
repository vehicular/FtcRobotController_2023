package org.firstinspires.ftc.teamcode.core.subsystems.compound;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RotatorMotor //extends Subsystem
{
    private DcMotor motor;

    public RotatorMotor(HardwareMap map)
    {
        //super(map);
        //motor = hardwaremap.dcMotor.get(Constants.intakeMotor);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //@Override
    public void teleopControls(Gamepad gamepad1, Gamepad gamepad2)
    {
        motor.setPower(7 * gamepad2.left_stick_y);
    }

    //@Override
    public String addTelemetry()
    {
        return "";
    }

    //@Override
    public void stop()
    {

    }
}
