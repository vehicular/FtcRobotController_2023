package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareMecanum;

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
//@Disabled
public class MecanumTeleOp extends LinearOpMode {

    HardwareMecanum robot = new HardwareMecanum();

    @Override
    public void runOpMode() throws InterruptedException {

        double x1; // left/right
        double y1; // front/back

        double fortyFiveInRads = -Math.PI/4;
        double cosine45 = Math.cos(fortyFiveInRads);
        double sine45 = Math.sin(fortyFiveInRads);

        double x2;
        double y2;

        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            y1 = -gamepad1.left_stick_y;
            x1 = gamepad1.left_stick_x;

            // need to rotate 45 degrees
            y2 = y1 * cosine45 + x1 * sine45;
            x2 = x1 * cosine45 - y1 * sine45;

            robot.LFMotor.setPower(x2);
            robot.RBMotor.setPower(x2);

            robot.RFMotor.setPower(y2);
            robot.LBMotor.setPower(y2);

            telemetry.addData("left", "%.2f", x1);
            telemetry.addData("right", "%.2f", y1);
            telemetry.update();

            sleep(50);
        }
    }

}
