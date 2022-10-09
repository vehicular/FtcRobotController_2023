/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * {@link SensorEncoderReader} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "Sensor: Encoder Reader", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class SensorEncoderReader extends LinearOpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    DcMotor lifter;
    DcMotor rotator;
    DcMotor arm;
    DcMotor slider;

    int rotatorPosition;
    int armPosition;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

        final double TRIGGER_THRESHOLD  = 0.75;     // Squeeze more than 3/4 to get rumble.

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        //frontLeft = hardwareMap.get(DcMotor.class, "LFMotor");
        //frontRight = hardwareMap.get(DcMotor.class, "RFMotor");
        backLeft = hardwareMap.get(DcMotor.class, "LBMotor");
        backRight = hardwareMap.get(DcMotor.class, "RBMotor");

        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifter = hardwareMap.get(DcMotor.class, "liftermotor");
        rotator = hardwareMap.get(DcMotor.class, "rotatormotor");
        arm = hardwareMap.get(DcMotor.class, "armmotor");//""armM");
        //slider = hardwareMap.get(DcMotor.class, "sliderM");

        //lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        //slider.setDirection(DcMotorSimple.Direction.REVERSE);

        //lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotatorPosition = 0;
        armPosition = 0;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget = arm.getCurrentPosition();

        boolean highLevel = false;
        int lifterZero = lifter.getCurrentPosition();

        // Loop and update the dashboard
        while (opModeIsActive()) {

            double armPower = 0.35;
            if(gamepad1.right_stick_y > 0.1  || gamepad1.right_stick_y < -0.1)
            {
                newLeftTarget += (int)(gamepad1.right_stick_y*50);
                armPower *= Math.abs(gamepad1.right_stick_y);
                //if(gamepad1.right_stick_y > 0.1 )
                if(gamepad1.right_stick_y < -0.1) // move arm down (need to check after 180 degrees)
                    armPower = 0.1*Math.abs(gamepad1.right_stick_y);
            }
            arm.setTargetPosition(newLeftTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            arm.setPower(armPower);//Math.max(0.2,gamepad1.right_stick_y));
            while (opModeIsActive() &&
                    (runtime.seconds() < 1) &&
                    (arm.isBusy() )) {

            }

            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (!highLevel) {
                    highLevel = true;  // Hold off any more triggers
                    lifter.setTargetPosition(lifter.getCurrentPosition()+100);
                    lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    lifter.setPower(0.2);
                    while (opModeIsActive() &&
                            (runtime.seconds() < 1) &&
                            (lifter.isBusy() )) {

                    }
                    gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
                }
            } else {
                highLevel = false;  // We can trigger again now.
            }

            if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                lifter.setTargetPosition(lifter.getCurrentPosition()-100);
                lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lifter.setPower(0.2);
                while (opModeIsActive() &&
                        (runtime.seconds() < 1) &&
                        (lifter.isBusy() )) {

                }
            }


            backLeft.setPower(gamepad1.left_stick_x*0.5);
            backRight.setPower(gamepad1.right_stick_x*.5);

            rotator.setPower(gamepad1.left_stick_y*0.2);


            telemetry.addLine().addData("Lifter Position at ",  "%7d",
                    lifter.getCurrentPosition());

            telemetry.addLine().addData("Rotator Currently at ",  "%7d",
                    rotatorPosition);

            telemetry.addLine().addData("Arm Position at ",  "%7d : %7d",
                    newLeftTarget,
                    armPosition);

            telemetry.addLine().addData("Back Drive Pos L:R ",  "%7d : %7d",
                    backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());

            telemetry.update();

            //sleep(250);   // optional pause after each move.
        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
                {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity  = imu.getGravity();
                    rotatorPosition = rotator.getCurrentPosition();
                    armPosition = arm.getCurrentPosition();
                }
            });

        telemetry.addLine()
            .addData("status", new Func<String>() {
                @Override public String value() {
                    return imu.getSystemStatus().toShortString();
                    }
                })
            .addData("calib", new Func<String>() {
                @Override public String value() {
                    return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
            .addData("roll", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
            .addData("pitch", new Func<String>() {
                @Override public String value() {
                    return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
            .addData("grvty", new Func<String>() {
                @Override public String value() {
                    return gravity.toString();
                    }
                })
            .addData("mag", new Func<String>() {
                @Override public String value() {
                    return String.format(Locale.getDefault(), "%.3f",
                            Math.sqrt(gravity.xAccel*gravity.xAccel
                                    + gravity.yAccel*gravity.yAccel
                                    + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
