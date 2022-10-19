package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.core.Subsystem;
import org.firstinspires.ftc.teamcode.core.subsystems.ChassisMecanum;
import org.firstinspires.ftc.teamcode.core.subsystems.Hand;

@TeleOp(name = "Game Robot", group = "Run")
public class RobotMain extends OpMode
{
    private Subsystem chassis;
    private Subsystem hand;

    boolean isTankDrive = false; // 2 wheels Tank drive, or 4 wheels Mecanum drive

    /**
     * Initialize any subsystems that need initializing before the game starts.
     */
    @Override
    public void init()
    {
        chassis = new ChassisMecanum(hardwareMap, isTankDrive);
        hand = new Hand(hardwareMap);

        chassis.teleopInit(hand);
        hand.teleopInit(chassis);
    }

    /**
     * Calls the teleop control method in all subsystems.
     */
    @Override
    public void loop() {
        chassis.teleopControls(gamepad1, gamepad2);
        telemetry.addData("  ", chassis.addTelemetry() + "\n");

        hand.teleopControls(gamepad1, gamepad2);
        telemetry.addData("  ", hand.addTelemetry() + "\n");

        chassis.CrossSubsystemCheck();
        hand.CrossSubsystemCheck();

        //telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        //telemetry.addData("right_stick_y", gamepad1.right_stick_y);
    }

    /**
     * Stop all subsystems.
     */
    @Override
    public void stop() {
        chassis.stop();
        hand.stop();
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
