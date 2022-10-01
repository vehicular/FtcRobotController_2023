package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.subsystems.Subsystem;

@TeleOp(name="teleop",group="")
@Disabled
public class Teleop extends OpMode
{
    /**
     * Initialize any subsystems that need initializing before the game starts.
     */
    @Override
    public void init()
    {
        new Robot(hardwareMap);
        Robot.chassis.startAccelerationIntegration();
        Robot.chassis.teleopInit();
        //Robot.climber.teleopInit();
    }

    /**
     * Calls the teleop control method in all subsystems.
     */
    @Override
    public void loop()
    {
        for(Subsystem system: Robot.modules)
        {
            system.teleopControls(gamepad1, gamepad2);

            telemetry.addData(system.toString(), system.addTelemetry() + "\n\n");

            telemetry.addData("left_stick_y" ,gamepad1.left_stick_y);
            telemetry.addData("right_stick_y" ,gamepad1.right_stick_y);
        }
    }

    /**
     * Stops all subsystems.
     */
    @Override
    public void stop()
    {
        for(Subsystem system: Robot.modules)
        {
            system.stop();
        }
    }
}
