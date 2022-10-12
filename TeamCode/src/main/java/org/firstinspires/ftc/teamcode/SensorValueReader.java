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

import android.os.Environment;

import com.google.gson.JsonObject;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.core.subsystems.Hand;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotorPosition;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Locale;

/**
 * {@link SensorValueReader} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@TeleOp(name = "JD Sensor Reader", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list
public class SensorValueReader extends LinearOpMode
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    DcMotor lifterMotor;
    DcMotor rotatorMotor;
    DcMotor armMotor;
    private Servo wristServo;
    private Servo palmServo;
    private Servo knukcleServo;
    private Servo fingerServo;

    int rotatorPosition;
    int armPosition;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

        final double TRIGGER_THRESHOLD  = 0.75;     // Squeeze more than 3/4 to get rumble.

        Hand hand;

    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override public void runOpMode() {

        //frontLeft = hardwareMap.get(DcMotor.class, "LFMotor");
        //frontRight = hardwareMap.get(DcMotor.class, "RFMotor");
        backLeft = hardwareMap.get(DcMotor.class, Constants.leftbackMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.rightbackMotor);

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

        lifterMotor = hardwareMap.get(DcMotor.class, Constants.lifterMotor);
        rotatorMotor = hardwareMap.get(DcMotor.class, Constants.rotatorMotor);
        armMotor = hardwareMap.get(DcMotor.class, Constants.armMotor);

        //lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        rotatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //slider.setDirection(DcMotorSimple.Direction.REVERSE);

        //lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //rotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotatorPosition = 0;
        armPosition = 0;

        fingerServo = hardwareMap.servo.get(Constants.fingerServo);
        wristServo = hardwareMap.servo.get(Constants.wristServo);
        palmServo = hardwareMap.servo.get(Constants.palmServo);
        knukcleServo = hardwareMap.servo.get(Constants.knuckleServo);

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
        imu = hardwareMap.get(BNO055IMU.class, Constants.imu);
        imu.initialize(parameters);

        //hand = new Hand(hardwareMap);

        // Set up our telemetry dashboard
        composeTelemetry();

        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/MOTORS";

        /*
        JSONObject InitData = new JSONObject();
        try {
            InitData.put(MotorPosition.LifterMotorStr, lifterMotor.getCurrentPosition());
            InitData.put(MotorPosition.RotatorMotorStr, rotatorMotor.getCurrentPosition());
            InitData.put(MotorPosition.ArmMotorStr, armMotor.getCurrentPosition());
            InitData.put(MotorPosition.WristServoStr, wristServo.getPosition());
            InitData.put(MotorPosition.PalmServoStr, palmServo.getPosition());
            InitData.put(MotorPosition.KnuckleServoStr, knukcleServo.getPosition());
            InitData.put(MotorPosition.FingerServoStr, fingerServo.getPosition());

            // Convert JsonObject to String Format
            String userString = InitData.toString();
            //telemetry.addLine(userString);
            // Define the File Path and its Name
            File directory = new File(directoryPath);
            directory.mkdir();
            FileWriter fileWriter = new FileWriter(
                    directoryPath+"/"+"InitMotorsPosition.json");

            fileWriter.write(userString);
            fileWriter.close();
        }catch (Exception e)
        {
            telemetry.addLine("Save Init Motor Position Exception..."+e.toString());
        }
*/

        try {
            FileReader fileReader = new FileReader(
                    directoryPath+"/"+"InitMotorsPosition.json");
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            StringBuilder stringBuilder = new StringBuilder();
            String line = bufferedReader.readLine();
            while (line != null) {
                stringBuilder.append(line).append("\n");
                line = bufferedReader.readLine();
            }
            bufferedReader.close();
            // This responce will have Json Format String
            String responce = stringBuilder.toString();
            //telemetry.addLine(responce);


            JSONObject jsonObject = new JSONObject(responce);
            int a = Integer.parseInt(jsonObject.get(MotorPosition.LifterMotorStr).toString());
            int b = Integer.parseInt(jsonObject.get(MotorPosition.RotatorMotorStr).toString());
            int c = Integer.parseInt(jsonObject.get(MotorPosition.ArmMotorStr).toString());
            double d = Double.parseDouble(jsonObject.get(MotorPosition.WristServoStr).toString());
            double e = Double.parseDouble(jsonObject.get(MotorPosition.PalmServoStr).toString());
            double f = Double.parseDouble(jsonObject.get(MotorPosition.KnuckleServoStr).toString());
            double g = Double.parseDouble(jsonObject.get(MotorPosition.FingerServoStr).toString());

            //Java Object
            MotorPosition javaObject = new MotorPosition(a,b,c,d,e,f,g);

            telemetry.addLine().addData("Init Position at: ",
                    "%7d:%7d:%7d:%7f:%7f:%7f:%7f",
                    a,b,c,d,e,f,g);
        }
        catch (Exception e)
        {
            telemetry.addLine("Read Init Motor Position Exception..."+e.toString());
        }

        telemetry.update();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget = armMotor.getCurrentPosition();

        boolean highLevel = false;
        int lifterZero = lifterMotor.getCurrentPosition();

        // Loop and update the dashboard
        while (opModeIsActive()) {
/* disable all controls

            double armPower = 0.35;
            if(gamepad1.right_stick_y > 0.1  || gamepad1.right_stick_y < -0.1)
            {
                newLeftTarget += (int)(gamepad1.right_stick_y*50);
                armPower *= Math.abs(gamepad1.right_stick_y);
                //if(gamepad1.right_stick_y > 0.1 )
                if(gamepad1.right_stick_y < -0.1) // move arm down (need to check after 180 degrees)
                    armPower = 0.1*Math.abs(gamepad1.right_stick_y);
            }
            armMotor.setTargetPosition(newLeftTarget);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            armMotor.setPower(armPower);//Math.max(0.2,gamepad1.right_stick_y));
            while (opModeIsActive() &&
                    (runtime.seconds() < 1) &&
                    (armMotor.isBusy() )) {

            }

            if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (!highLevel) {
                    highLevel = true;  // Hold off any more triggers
                    lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition()+100);
                    lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    lifterMotor.setPower(0.2);
                    while (opModeIsActive() &&
                            (runtime.seconds() < 1) &&
                            (lifterMotor.isBusy() )) {

                    }
                    gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
                }
            } else {
                highLevel = false;  // We can trigger again now.
            }

            if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition()-100);
                lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lifterMotor.setPower(0.2);
                while (opModeIsActive() &&
                        (runtime.seconds() < 1) &&
                        (lifterMotor.isBusy() )) {

                }
            }


            backLeft.setPower(gamepad1.left_stick_x*0.5);
            backRight.setPower(gamepad1.right_stick_x*.5);

            rotatorMotor.setPower(gamepad1.left_stick_y*0.2);
*/
            //hand.teleopControls(gamepad1, gamepad2);


            telemetry.addLine().addData("Lifter Position at ",  "%7d",
                    lifterMotor.getCurrentPosition());

            telemetry.addLine().addData("Rotator Currently at ",  "%7d",
                    rotatorMotor.getCurrentPosition());

            telemetry.addLine().addData("Arm Position at ",  "%7d",
                    armMotor.getCurrentPosition());

            telemetry.addLine().addData("wristServo Currently at ",  "%7f",
                    wristServo.getPosition());

            telemetry.addLine().addData("palmServo Currently at ",  "%7f",
                    palmServo.getPosition());

            telemetry.addLine().addData("knukcleServo Currently at ",  "%7f",
                    knukcleServo.getPosition());

            telemetry.addLine().addData("fingerServo Currently at ",  "%7f",
                    fingerServo.getPosition());

            //telemetry.addLine(hand.addTelemetry());

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
                    //rotatorPosition = rotatorMotor.getCurrentPosition();
                    //armPosition = armMotor.getCurrentPosition();
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
