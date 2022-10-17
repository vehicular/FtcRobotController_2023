package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.core.Subsystem;
import org.firstinspires.ftc.teamcode.core.subsystems.ChassisMecanum;
import org.firstinspires.ftc.teamcode.core.subsystems.Hand;
import org.firstinspires.ftc.teamcode.util.GameState;

@TeleOp(name="JD Robot",group="Run")
//@Disabled
public class Teleop extends OpMode
{
    private int StateMachine = GameState.RUN;

    private ChassisMecanum chassis;
    private Hand hand;

    /**
     * Initialize any subsystems that need initializing before the game starts.
     */
    @Override
    public void init()
    {
        chassis = new ChassisMecanum(hardwareMap, false);
        hand = new Hand(hardwareMap);

        chassis.teleopInit(hand);
        hand.teleopInit(chassis);
    }

    /**
     * Calls the teleop control method in all subsystems.
     */
    @Override
    public void loop()
    {
        /**
         * As an illustration, the first line on our telemetry display will display the battery voltage.
         * The idea here is that it's expensive to compute the voltage (at least for purposes of illustration)
         * so you don't want to do it unless the data is <em>actually</em> going to make it to the
         * driver station (recall that telemetry transmission is throttled to reduce bandwidth use.
         * Note that getBatteryVoltage() below returns 'Infinity' if there's no voltage sensor attached.
         *
         * @see Telemetry#getMsTransmissionInterval()
         */
        /*telemetry.addData("voltage", "%.1f volts", new Func<Double>() {
            @Override public Double value() {
                return getBatteryVoltage();
            }
        });*/

        if( StateMachine == GameState.RUN) {

            chassis.teleopControls(gamepad1, gamepad2);
            telemetry.addData(chassis.toString(), chassis.addTelemetry() + "\n\n");

            hand.teleopControls(gamepad1, gamepad2);
            telemetry.addData(hand.toString(), hand.addTelemetry() + "\n\n");

            chassis.CrossSubsystemCheck();
            hand.CrossSubsystemCheck();

            //telemetry.addData("left_stick_y", gamepad1.left_stick_y);
            //telemetry.addData("right_stick_y", gamepad1.right_stick_y);
        }
    }

    /**
     * Stops all subsystems.
     */
    @Override
    public void stop()
    {
        //for(Subsystem system: myRobot.modules)
        {
            chassis.stop();
            hand.stop();
        }
    }

    // Computes the current battery voltage
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
