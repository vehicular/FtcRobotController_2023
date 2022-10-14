package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Constants;


@Disabled
@TeleOp(name = "JD Manual Drive", group = "sample")

public class MecanumRobotTeleop extends OpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;


    double LFMotorMultiplier = 0.5;
    double RFMotorMultiplier = 0.5;
    double LBMotorMultiplier = 0.5;
    double RBMotorMultiplier = 0.5;

    //public MecanumDrivetrain drivetrain;

    BNO055IMU imu;

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, Constants.leftfrontMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.rightfrontMotor);
        backLeft = hardwareMap.get(DcMotor.class, Constants.leftbackMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.rightbackMotor);


        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //drivetrain = new MecanumDrivetrain(new DcMotor[]{frontLeft, frontRight, backLeft, backRight});

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, Constants.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        /*double course = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - Math.PI/2;
        double velocity = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double rotation = -gamepad1.left_stick_x;

        drivetrain.setCourse(course);
        drivetrain.setVelocity(velocity);
        drivetrain.setRotation(rotation);*/

        double x = -gamepad1.left_stick_x; // Remember, this is reversed!
        double y = gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

        //the translation joystick values need to be rotated by the robot heading
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower*LFMotorMultiplier);
        backLeft.setPower(backLeftPower*LBMotorMultiplier);
        frontRight.setPower(frontRightPower*RFMotorMultiplier);
        backRight.setPower(backRightPower*RBMotorMultiplier);

        telemetry.addData("left_stick_x" ,gamepad1.left_stick_x);
        telemetry.addData("left_stick_y" ,gamepad1.left_stick_y);
        telemetry.addData("right_stick_x" ,gamepad1.right_stick_x);

        /*telemetry.addData("course", course);
        telemetry.addData("velocity", velocity);
        telemetry.addData("rotation", rotation);*/
        telemetry.update();
    }
}

