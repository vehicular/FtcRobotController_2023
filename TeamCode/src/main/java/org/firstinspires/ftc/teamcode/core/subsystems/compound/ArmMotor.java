package org.firstinspires.ftc.teamcode.core.subsystems.compound;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.teamcode.core.commands.RaiseLift;


public class ArmMotor
{
    public DcMotor motor;

    public ArmMotor(HardwareMap map)
    {
        //super(map);
        //this.motor = hardwaremap.dcMotor.get(Constants.climber);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //@Override
    public void teleopControls(Gamepad gamepad1, Gamepad gamepad2)
    {
        if (gamepad1.left_trigger > 0)
        {
            motor.setPower(-gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0)
        {
            motor.setPower(gamepad1.right_trigger);
        } else
        {
            motor.setPower(0);
        }

        if (gamepad1.a)
        {
            //RaiseLift(Constants.bottomClimber);
        }
    }

    //@Override
    public String addTelemetry()
    {
        return "Climber: " +
                "\n Motor Mode: " + motor.getMode()
                + "\ncurrent power: " + motor.getPower()
                + "\nSetpoint: " + motor.getTargetPosition()
                + "\n Is Finished: " + this.reachedTargetPosition();
    }

    //@Override
    public void stop()
    {
        motor.setPower(0);
    }

    public void setPower(double power)
    {
        motor.setPower(power);
    }

    /**
     * Sets the runmode to position.
     */
    //@Override
    public void autoInit()
    {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //@Override
    public void teleopInit()
    {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Sets the position for the climber to move to.
     * You can use Constants.liftPosition** to move to top, middle, or bottom.
     * Another option is to set the power in ticks.
     *
     * @param position
     * @warning This method is useless unless you call runToPosition repeatedly afterwards
     */
    public void setTargetPosition(int position)
    {
        motor.setTargetPosition(position + motor.getCurrentPosition());
    }

    public void runToPosition()
    {
        motor.setPower(-1);
    }

    public boolean reachedTargetPosition()
    {
        return Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()) < 7;
    }

    public int getCurrentPosition()
    {
        return motor.getCurrentPosition();
    }

}
