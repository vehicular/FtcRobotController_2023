package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwareMecanum;

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
@Disabled
public class MecanumTeleOp extends LinearOpMode {

    HardwareMecanum robot = new HardwareMecanum();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.LFMotor.setPower(frontLeftPower);
            robot.RBMotor.setPower(backRightPower);

            robot.RFMotor.setPower(frontRightPower);
            robot.LBMotor.setPower(backLeftPower);

            telemetry.addData("left", "%.2f", frontLeftPower);
            telemetry.addData("right", "%.2f", frontRightPower);
            telemetry.update();

            sleep(50);
        }
    }

}
