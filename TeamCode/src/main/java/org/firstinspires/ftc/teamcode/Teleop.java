package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Robot;
import org.firstinspires.ftc.teamcode.core.Subsystem;
import org.firstinspires.ftc.teamcode.util.GameState;

@TeleOp(name="JD Robot",group="")
//@Disabled
public class Teleop extends OpMode
{
    private int StateMachine = GameState.RUN;

    private Robot myRobot;
    /**
     * Initialize any subsystems that need initializing before the game starts.
     */
    @Override
    public void init()
    {
        myRobot = new Robot(hardwareMap);
        for (Subsystem system : myRobot.modules) {
            system.teleopInit();
        }
    }

    /**
     * Calls the teleop control method in all subsystems.
     */
    @Override
    public void loop()
    {
        if( StateMachine == GameState.RUN) {
            for (Subsystem system : myRobot.modules) {
                system.teleopControls(gamepad1, gamepad2);

                telemetry.addData(system.toString(), system.addTelemetry() + "\n\n");

                telemetry.addData("left_stick_y", gamepad1.left_stick_y);
                telemetry.addData("right_stick_y", gamepad1.right_stick_y);
            }
        }
    }

    /**
     * Stops all subsystems.
     */
    @Override
    public void stop()
    {
        for(Subsystem system: myRobot.modules)
        {
            system.stop();
        }
    }
}
