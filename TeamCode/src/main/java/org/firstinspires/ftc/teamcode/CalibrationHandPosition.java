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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.HandMotorsPosition;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Locale;

/**
 * {@link CalibrationHandPosition} Set Motors' Predefined Position
 * <p>
 */
@TeleOp(name = "Calibration Motors' Position ", group = "Calibration")
//@Disabled
public class CalibrationHandPosition extends LinearOpMode
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------
    
    //public DcMotor frontLeft;
    //public DcMotor frontRight;
    //public DcMotor backLeft;
    //public DcMotor backRight;
    
    DcMotor lifterMotor;
    DcMotor rotatorMotor;
    DcMotor armMotor;
    private Servo wristServo;
    private Servo palmServo;
    private Servo knukcleServo;
    private Servo fingerServo;
    
    // The IMU sensor object
    //BNO055IMU imu;
    
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    int armPosition;
    
    HandMotorsPosition PredefinedPosition = new HandMotorsPosition();
    
    String directoryPath = Environment.getExternalStorageDirectory().getPath() + "/MOTORS";
    
    private void SavePositonsToFile(HandMotorsPosition.SubsystemPosition positions)
    {
        String fileName = positions.GetPositionName();
        JSONObject InitData = new JSONObject();
        try
        {
            InitData.put(HandMotorsPosition.LifterMotorStr, positions.LifterMotor);
            InitData.put(HandMotorsPosition.RotatorMotorStr, positions.RotatorMotor);
            InitData.put(HandMotorsPosition.ArmMotorStr, positions.ArmMotor);
            InitData.put(HandMotorsPosition.WristServoStr, positions.WristServo);
            InitData.put(HandMotorsPosition.PalmServoStr, positions.PalmServo);
            InitData.put(HandMotorsPosition.KnuckleServoStr, positions.KnuckleServo);
            InitData.put(HandMotorsPosition.FingerServoStr, positions.FingerServo);
            
            // Convert JsonObject to String Format
            String userString = InitData.toString();
            //telemetry.addLine(userString);
            // Define the File Path and its Name
            File directory = new File(directoryPath);
            directory.mkdir();
            FileWriter fileWriter = new FileWriter(
                    directoryPath + "/" + fileName);
            FileWriter bkWriter = new FileWriter(
                    directoryPath + "/bk_" + fileName);
            
            fileWriter.write(userString);
            fileWriter.close();
            bkWriter.write(userString);
            bkWriter.close();
        } catch (Exception e)
        {
            telemetry.addLine("Save " + fileName + " Error..." + e.toString());
        }
    }
    
    private String[] ReadPositionFromFile(String fileName)
    {
        String[] data = new String[HandMotorsPosition.maxNumOfMotors];
        try
        {
            FileReader fileReader = new FileReader(
                    directoryPath + "/" + fileName);
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            StringBuilder stringBuilder = new StringBuilder();
            String line = bufferedReader.readLine();
            while (line != null)
            {
                stringBuilder.append(line).append("\n");
                line = bufferedReader.readLine();
            }
            bufferedReader.close();
            fileReader.close();
            // This responce will have Json Format String
            String responce = stringBuilder.toString();
            
            JSONObject jsonObject = new JSONObject(responce);
            data[HandMotorsPosition.LifterMotorInt] = ((jsonObject.get(HandMotorsPosition.LifterMotorStr).toString()));
            data[HandMotorsPosition.RotatorMotorInt] = ((jsonObject.get(HandMotorsPosition.RotatorMotorStr).toString()));
            data[HandMotorsPosition.ArmMotorInt] = ((jsonObject.get(HandMotorsPosition.ArmMotorStr).toString()));
            data[HandMotorsPosition.WristServoInt] = ((jsonObject.get(HandMotorsPosition.WristServoStr).toString()));
            data[HandMotorsPosition.PalmServoInt] = ((jsonObject.get(HandMotorsPosition.PalmServoStr).toString()));
            data[HandMotorsPosition.KnuckleServoInt] = ((jsonObject.get(HandMotorsPosition.KnuckleServoStr).toString()));
            data[HandMotorsPosition.FingerServoInt] = ((jsonObject.get(HandMotorsPosition.FingerServoStr).toString()));
            
        } catch (Exception e)
        {
            telemetry.addLine("Read " + fileName + " Error..." + e.toString());
        }
        return data;
    }
    
    private void SetMotorsPosition(HandMotorsPosition.SubsystemPosition positions)
    {
        /*wristServo.setPosition(positions.WristServo);
        palmServo.setPosition(positions.PalmServo);
        knukcleServo.setPosition(positions.KnuckleServo);
        fingerServo.setPosition(positions.FingerServo);*/
    }
    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------
    
    
    @Override
    public void runOpMode()
    {
        /*//frontLeft = hardwareMap.get(DcMotor.class, "LFMotor");
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
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        
        lifterMotor = hardwareMap.get(DcMotor.class, Constants.lifterMotor);
        rotatorMotor = hardwareMap.get(DcMotor.class, Constants.rotatorMotor);
        armMotor = hardwareMap.get(DcMotor.class, Constants.armMotor);
        
        //lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        rotatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //slider.setDirection(DcMotorSimple.Direction.REVERSE);
        
        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        fingerServo = hardwareMap.servo.get(Constants.fingerServo);
        wristServo = hardwareMap.servo.get(Constants.wristServo);
        palmServo = hardwareMap.servo.get(Constants.palmServo);
        knukcleServo = hardwareMap.servo.get(Constants.knuckleServo);
        
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled = true;
        //parameters.loggingTag = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        //imu = hardwareMap.get(BNO055IMU.class, Constants.imu);
        //imu.initialize(parameters);
        
        //hand = new Hand(hardwareMap);
        
        // Set up our telemetry dashboard
        //composeTelemetry();


// Start to Save files
        
        // palm: max 0.7, min 0.05
        // wrist max, 0.72, min 0
        
        PredefinedPosition.PowerOnHold.LifterMotor = 0;
        PredefinedPosition.PowerOnHold.RotatorMotor = 0;
        PredefinedPosition.PowerOnHold.ArmMotor = 0;
        PredefinedPosition.PowerOnHold.WristServo = 0.74;
        PredefinedPosition.PowerOnHold.PalmServo = 0.7;
        PredefinedPosition.PowerOnHold.KnuckleServo = 0.33;
        PredefinedPosition.PowerOnHold.FingerServo = 0.1;
    
        PredefinedPosition.EyeLowPole.LifterMotor = 0;
        PredefinedPosition.EyeLowPole.RotatorMotor = 0;
        PredefinedPosition.EyeLowPole.ArmMotor = 202;
        PredefinedPosition.EyeLowPole.WristServo = 0.5;
        PredefinedPosition.EyeLowPole.PalmServo = 0.52;
        PredefinedPosition.EyeLowPole.KnuckleServo = 0.34;
        PredefinedPosition.EyeLowPole.FingerServo = 0.1;
        
        PredefinedPosition.InitPosition.LifterMotor = 0;
        PredefinedPosition.InitPosition.RotatorMotor = 0;
        PredefinedPosition.InitPosition.ArmMotor = 0;
        PredefinedPosition.InitPosition.WristServo = 0.92;
        PredefinedPosition.InitPosition.PalmServo = 0.95;
        PredefinedPosition.InitPosition.KnuckleServo = 0.33;
        PredefinedPosition.InitPosition.FingerServo = 0.9;
        
        PredefinedPosition.Pickup_up.LifterMotor = 0;
        PredefinedPosition.Pickup_up.RotatorMotor = 0;
        PredefinedPosition.Pickup_up.ArmMotor = -445;
        PredefinedPosition.Pickup_up.WristServo = 0.82;
        PredefinedPosition.Pickup_up.PalmServo = 0.28;
        PredefinedPosition.Pickup_up.KnuckleServo = 0.33;
        PredefinedPosition.Pickup_up.FingerServo = 0.9;
        
        PredefinedPosition.Pickup_down.LifterMotor = 0;
        PredefinedPosition.Pickup_down.RotatorMotor = 0;
        PredefinedPosition.Pickup_down.ArmMotor = -210;//-310 is lowest, add a bit
        PredefinedPosition.Pickup_down.WristServo = 0.91;
        PredefinedPosition.Pickup_down.PalmServo = 0.95;
        PredefinedPosition.Pickup_down.KnuckleServo = 0.33;
        PredefinedPosition.Pickup_down.FingerServo = 1;
        
        PredefinedPosition.Pickup_left.LifterMotor = 0;
        PredefinedPosition.Pickup_left.RotatorMotor = 0;
        PredefinedPosition.Pickup_left.ArmMotor = -210;//-310 is lowest, add a bit
        PredefinedPosition.Pickup_left.WristServo = 0.91;
        PredefinedPosition.Pickup_left.PalmServo = 0.95;
        PredefinedPosition.Pickup_left.KnuckleServo = 0.0;
        PredefinedPosition.Pickup_left.FingerServo = 0.9;
        
        PredefinedPosition.Pickup_right.LifterMotor = 0;
        PredefinedPosition.Pickup_right.RotatorMotor = 0;
        PredefinedPosition.Pickup_right.ArmMotor = -210;//-310 is lowest, add a bit
        PredefinedPosition.Pickup_right.WristServo = 0.91;
        PredefinedPosition.Pickup_right.PalmServo = 0.95;
        PredefinedPosition.Pickup_right.KnuckleServo = 0.7;
        PredefinedPosition.Pickup_right.FingerServo = 0.9;
        
        PredefinedPosition.Drop_A_1.LifterMotor = 0;
        PredefinedPosition.Drop_A_1.RotatorMotor = 0;
        PredefinedPosition.Drop_A_1.ArmMotor = -442;
        PredefinedPosition.Drop_A_1.WristServo = 0.64;
        PredefinedPosition.Drop_A_1.PalmServo = 0.48;
        PredefinedPosition.Drop_A_1.KnuckleServo = 0.33;
        PredefinedPosition.Drop_A_1.FingerServo = 0.1;
        
        PredefinedPosition.Drop_B_2.LifterMotor = 0;
        PredefinedPosition.Drop_B_2.RotatorMotor = 0;
        PredefinedPosition.Drop_B_2.ArmMotor = 172;
        PredefinedPosition.Drop_B_2.WristServo = 0.68;
        PredefinedPosition.Drop_B_2.PalmServo = 0.57;
        PredefinedPosition.Drop_B_2.KnuckleServo = 0.34;
        PredefinedPosition.Drop_B_2.FingerServo = 0.1;
        
        PredefinedPosition.Drop_X_3.LifterMotor = 0;
        PredefinedPosition.Drop_X_3.RotatorMotor = 0;
        PredefinedPosition.Drop_X_3.ArmMotor = 725;
        PredefinedPosition.Drop_X_3.WristServo = 0.76;
        PredefinedPosition.Drop_X_3.PalmServo = 0.61;
        PredefinedPosition.Drop_X_3.KnuckleServo = 0.34;
        PredefinedPosition.Drop_X_3.FingerServo = 0.1;
        
        PredefinedPosition.Drop_Y_4.LifterMotor = 300;
        PredefinedPosition.Drop_Y_4.RotatorMotor = 0;
        PredefinedPosition.Drop_Y_4.ArmMotor = 1150;
        PredefinedPosition.Drop_Y_4.WristServo = 0.91;
        PredefinedPosition.Drop_Y_4.PalmServo = 0.96;
        PredefinedPosition.Drop_Y_4.KnuckleServo = 0.34;
        PredefinedPosition.Drop_Y_4.FingerServo = 0.1;
        
        SavePositonsToFile(PredefinedPosition.PowerOnHold);
        SavePositonsToFile(PredefinedPosition.EyeLowPole);
        SavePositonsToFile(PredefinedPosition.InitPosition);
        
        SavePositonsToFile(PredefinedPosition.Pickup_up);
        SavePositonsToFile(PredefinedPosition.Pickup_down);
        SavePositonsToFile(PredefinedPosition.Pickup_left);
        SavePositonsToFile(PredefinedPosition.Pickup_right);
        
        SavePositonsToFile(PredefinedPosition.Drop_A_1);
        SavePositonsToFile(PredefinedPosition.Drop_B_2);
        SavePositonsToFile(PredefinedPosition.Drop_X_3);
        SavePositonsToFile(PredefinedPosition.Drop_Y_4);
// End of Saving
    
        PredefinedPosition.PowerOnHold.SetValue(
                ReadPositionFromFile(PredefinedPosition.PowerOnHold.GetPositionName()));
        PredefinedPosition.EyeLowPole.SetValue(
                ReadPositionFromFile(PredefinedPosition.EyeLowPole.GetPositionName()));
        PredefinedPosition.InitPosition.SetValue(
                ReadPositionFromFile(PredefinedPosition.InitPosition.GetPositionName()));
        PredefinedPosition.Pickup_up.SetValue(
                ReadPositionFromFile(PredefinedPosition.Pickup_up.GetPositionName()));
        PredefinedPosition.Pickup_down.SetValue(
                ReadPositionFromFile(PredefinedPosition.Pickup_down.GetPositionName()));
        PredefinedPosition.Pickup_left.SetValue(
                ReadPositionFromFile(PredefinedPosition.Pickup_left.GetPositionName()));
        PredefinedPosition.Pickup_right.SetValue(
                ReadPositionFromFile(PredefinedPosition.Pickup_right.GetPositionName()));
        PredefinedPosition.Drop_A_1.SetValue(
                ReadPositionFromFile(PredefinedPosition.Drop_A_1.GetPositionName()));
        PredefinedPosition.Drop_B_2.SetValue(
                ReadPositionFromFile(PredefinedPosition.Drop_B_2.GetPositionName()));
        PredefinedPosition.Drop_X_3.SetValue(
                ReadPositionFromFile(PredefinedPosition.Drop_X_3.GetPositionName()));
        PredefinedPosition.Drop_Y_4.SetValue(
                ReadPositionFromFile(PredefinedPosition.Drop_Y_4.GetPositionName()));
        
        
        telemetry.addLine("Init Position at: " +
                PredefinedPosition.InitPosition.toString());
        
        
        telemetry.update();
        
        SetMotorsPosition(PredefinedPosition.InitPosition);
        
        // Wait until we're told to go
        waitForStart();
        
        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        // Loop and update the dashboard
        while (opModeIsActive())
        {
            /*telemetry.addLine().addData("Lifter Position at ", "%7d",
                    lifterMotor.getCurrentPosition());
            
            telemetry.addLine().addData("Rotator Currently at ", "%7d",
                    rotatorMotor.getCurrentPosition());
            
            telemetry.addLine().addData("Arm Position at ", "%7d",
                    armMotor.getCurrentPosition());
            
            telemetry.addLine().addData("wristServo Currently at ", "%7f",
                    wristServo.getPosition());
            
            telemetry.addLine().addData("palmServo Currently at ", "%7f",
                    palmServo.getPosition());
            
            telemetry.addLine().addData("knukcleServo Currently at ", "%7f",
                    knukcleServo.getPosition());
            
            telemetry.addLine().addData("fingerServo Currently at ", "%7f",
                    fingerServo.getPosition());*/

            /*telemetry.addLine().addData("Back Drive Pos L:R ", "%7d : %7d",
                    backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());*/
            
            telemetry.update();
            
            //sleep(250);   // optional pause after each move.
        }
    }
    
    
    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------
    /*
    void composeTelemetry()
    {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable()
        {
            @Override
            public void run()
            {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //gravity = imu.getGravity();
            }
        });
        
        telemetry.addLine()
                .addData("status", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        
        telemetry.addLine()
                .addData("heading", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        
        telemetry.addLine()
                .addData("grvty", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>()
                {
                    @Override
                    public String value()
                    {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel
                                        + gravity.yAccel * gravity.yAccel
                                        + gravity.zAccel * gravity.zAccel));
                    }
                });
    }
    */
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------
    
    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    
    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
