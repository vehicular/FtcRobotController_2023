/* Copyright (c) 2022 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.core.subsystems.Eye;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.Locale;

/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Game Robot Autonomous", group="Robot") // Drive
//@Disabled
public class RobotAutonomousDrive extends OpMode
{

    /* Declare OpMode members. */
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU
    
    private DcMotor lifterMotor;
    private DcMotor rotatorMotor;
    private DcMotor armMotor;
    
    private Servo wristServo;
    private Servo palmServo;
    private Servo knuckleServo;
    private Servo fingerServo;

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    Eye eye;
    
    private ElapsedTime taskRunTimeout = new ElapsedTime();
    
    /**
     * Initialize any subsystems that need initializing before the game starts.
     */
    @Override
    public void init()
    {
        //hand = new Hand(hardwareMap);
        eye = new Eye(hardwareMap);
        
        //hand.autoInit();
        eye.autoInit();
    
        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, Constants.leftfrontMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.rightfrontMotor);
        backLeft = hardwareMap.get(DcMotor.class, Constants.leftbackMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.rightbackMotor);
    
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

    
        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, Constants.imu);
        imu.initialize(parameters);
    
        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
        // Set the encoders for closed loop speed control, and reset the heading.
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        resetHeading();
    
        fingerServo = hardwareMap.servo.get(Constants.fingerServo);
        wristServo = hardwareMap.servo.get(Constants.wristServo);
        palmServo = hardwareMap.servo.get(Constants.palmServo);
        knuckleServo = hardwareMap.servo.get(Constants.knuckleServo);
    
        lifterMotor = hardwareMap.get(DcMotor.class, Constants.lifterMotor);
        rotatorMotor = hardwareMap.get(DcMotor.class, Constants.rotatorMotor);
        armMotor = hardwareMap.get(DcMotor.class, Constants.armMotor);
    
        //lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    
        // reset all motors encoder to zero. remove them since we use saved cali data
        //lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        lifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        
        // Wait for the game to start (Display Gyro value while waiting)
        telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
        telemetry.update();
        
        composeTelemetry();
    }
    
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {
        fingerServo.setPosition(0.2);
        telemetry.update(); // show composeTelemetry
    }
    
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        targetPositionLifter = lifterMotor.getCurrentPosition();
        targetPositionRotator = rotatorMotor.getCurrentPosition();
        targetPositionArm = armMotor.getCurrentPosition();
    }
    
    // SPOT is loop-able function, MOVE is onetime execution function
    private enum Mission
    {
        SPOT_A, // init position
        MOVE_A2B,
        SPOT_B, // cone stock
        MOVE_B2C,
        MOVE_B2D,
        SPOT_C, // junction
        MOVE_C2B,
        MOVE_C2D,
        SPOT_D, // parking zone, final stop
        EXIT
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
     switch (currentMission)
     {
         case SPOT_A:
             // find self init position, read parking zone picture, drop preload to low junction
             spotA();
             break;
         case SPOT_B:
             // pick up a cone
             sportB();
             break;
         case SPOT_C:
             break;
         case SPOT_D:
             break;
         case MOVE_A2B:
             moveA2B();
             break;
         case MOVE_B2C:
             break;
         case MOVE_B2D:
             break;
         case MOVE_C2B:
             break;
         case MOVE_C2D:
             break;
         default: //EXIT
             break;
     }
        rotatorMotorRunnable();
        armMotorRunnable();
    
        sendTelemetry(true);
        
        telemetry.update();
        //sleep(1000);  // Pause to display last telemetry message.
    }
    
    private int loopTaskCount = 0;
    private Mission currentMission = Mission.SPOT_A;
    private Mission previousMission = Mission.SPOT_A;
    private void setMissionTo(Mission newMission)
    {
        loopTaskCount = 0;
        previousMission = currentMission;
        currentMission = newMission;
    }
    
    private void spotA()
    {
        if(loopTaskCount == 0)
        {
            palmServo.setPosition(0.7);//PredefinedPosition.PowerOnHold.PalmServo);
            knuckleServo.setPosition(0.33);//PredefinedPosition.PowerOnHold.KnuckleServo);
            targetPositionArm = -430;
            taskRunTimeout.reset();
            loopTaskCount = 1;
        }
        else if(loopTaskCount == 1)
        {
            boolean done = Util.inRange(armMotor.getCurrentPosition(),
                    targetPositionArm-10, targetPositionArm+10);
            if(done || taskRunTimeout.seconds() >= 2)
            {
                wristServo.setPosition(0.74);//PredefinedPosition.PowerOnHold.WristServo);
                targetPositionArm = 680;
                taskRunTimeout.reset();
                loopTaskCount = 2;
            }
        }
        else if(loopTaskCount == 2)
        {
            boolean done = Util.inRange(armMotor.getCurrentPosition(),
                targetPositionArm-10, targetPositionArm+10);
            if(done || taskRunTimeout.seconds() >= 3)
            {
                wristServo.setPosition(0.6);//PredefinedPosition.EyeLowPole.WristServo);
                palmServo.setPosition(0.29);//0.52);//PredefinedPosition.EyeLowPole.PalmServo);
                //knuckleServo.setPosition(0.33);//PredefinedPosition.EyeLowPole.KnuckleServo);
                taskRunTimeout.reset();
                loopTaskCount = 3;
            }
        }
        else if(loopTaskCount == 3)
        {
            if(taskRunTimeout.milliseconds() > 300 )
            {
                targetPositionArm = 460;
                taskRunTimeout.reset();
                loopTaskCount = 4;
            }
        }
        else if(loopTaskCount == 4)
        {
            if (taskRunTimeout.seconds() < 5)
            {
                Eye.ObjectLocation isCenter = eye.CheckLowPoleOnCenter();
                if (isCenter == Eye.ObjectLocation.CENTER)
                {
                    // found the location, prepare the drop positions
                    targetPositionArm = 390;
                    wristServo.setPosition(0.72);
                    palmServo.setPosition(0.19);
                    taskRunTimeout.reset();
                    loopTaskCount = 5;
                }
                else if (isCenter == Eye.ObjectLocation.LEFT)
                {
                    targetPositionRotator += 5;
                }
                else // right, front, back
                {
                    targetPositionRotator = -5;
                }
            }
            else // timeout
            {
                //TODO check other side of robot, repeat this step
                loopTaskCount = 5; // going to drop the cone randomly, good luck
            }
        }
        else if(loopTaskCount == 5)
        {
            if(taskRunTimeout.milliseconds() > 300 )
            {
                fingerServo.setPosition(0.7); // drop it
                setMissionTo(Mission.MOVE_A2B);
                targetPositionArm = 700; // better to help robot moving around
            }
        }
    
    
        // Camera to identify parking ID
        /*Eye.ParkingZone parkingID = eye.ReadParkingID();//wait 5 secs for now
    
        // Camera to read position ID, picture is on the side wall
        hand.EyeOnSideWallSetup();
        Eye.InitPosition initSlot = Eye.InitPosition.UNKNOWN;
        runtimeout.reset();
        while (initSlot == Eye.InitPosition.UNKNOWN && runtimeout.seconds() < 5) // need timeout
        {
            hand.RotatorToAngle(rototorAngle, 1);
            initSlot = eye.ReadLeftWallSlotID(true);
            if (initSlot == Eye.InitPosition.UNKNOWN)
            {
                hand.RotatorToAngle(-1*rototorAngle, 3);
                initSlot = eye.ReadLeftWallSlotID(false);
            }
            rototorAngle = rototorAngle - 25;
        }
        */
    }
    
    private void moveA2B()
    {
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        if(loopTaskCount == 0)
        {
            driveStrafeInit(24, DRIVE_SPEED, 4, 0.0);    // strafe left 10
            taskRunTimeout.reset();
            loopTaskCount = 1;
        }
        else if(loopTaskCount == 1)
        {
            boolean done = driveStrafeLoop(24, DRIVE_SPEED, 4, 0.0);
            if(taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                driveStraightInit(DRIVE_SPEED, 48.0, 0.0);    // Drive Forward 72"
                taskRunTimeout.reset();
                loopTaskCount = 2;
            }
        }
        else if(loopTaskCount == 2)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, 48.0, 0.0);
            if(taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                turnToHeadingInit( TURN_SPEED, -90.0);
                taskRunTimeout.reset();
                loopTaskCount = 3;
            }
        }
        else if(loopTaskCount == 3)
        {
            boolean done = turnToHeadingLoop(TURN_SPEED, -90.0);
            if(taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                holdHeadingInit( TURN_SPEED, -90.0, 0.5);   // Hold -45 Deg heading for a 1/2 second
                taskRunTimeout.reset();
                loopTaskCount = 4;
            }
        }
        else if(loopTaskCount == 4)
        {
            boolean done = holdHeadingLoop( TURN_SPEED, -90.0, 0.5);
            if(taskRunTimeout.seconds() >= 1)
            {
                // timeout, bad! should not happen at all
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                driveStraightInit(DRIVE_SPEED, 36.0, -90);    // Drive Forward 36
                wristServo.setPosition(0.6); // let camera ready
                palmServo.setPosition(0.29);
                taskRunTimeout.reset();
                loopTaskCount = 5;
            }
        }
        else if(loopTaskCount == 5)
        {
            boolean done = driveStraightLoop(DRIVE_SPEED, 36.0, -90);
            if(taskRunTimeout.seconds() >= 10)
            {
                // timeout, bad! should not happen at all
                setMissionTo(Mission.EXIT);
            }
            else if( done )
            {
                targetPositionArm = 0; // prepare to pick up
                setMissionTo(Mission.SPOT_B);
                taskRunTimeout.reset();
            }
        }
    }
    
    private void sportB()
    {
        if(loopTaskCount == 0)
        {
            if (taskRunTimeout.seconds() < 5)
            {
                Eye.ObjectLocation isCenter = eye.CheckConeOnCenter();
                if (isCenter == Eye.ObjectLocation.CENTER)
                {
                    // found the location, prepare the pickup positions
                    targetPositionArm = -380;
                    wristServo.setPosition(0.72);
                    palmServo.setPosition(0.19);
                    taskRunTimeout.reset();
                    loopTaskCount = 1;
                }
                else if (isCenter == Eye.ObjectLocation.LEFT)
                {
                    // use mecanum is better, TODO
                    targetPositionRotator += 5;
                }
                else // right, front, back
                {
                    targetPositionRotator = -5;
                }
            }
            else
            {
                // nothing is found, timeout. Go parking
                setMissionTo(Mission.MOVE_B2D);
            }
        }
        else if(loopTaskCount == 1)
        {
            if (taskRunTimeout.milliseconds() >= 100)
            {
                fingerServo.setPosition(0.1);
                loopTaskCount = 2;
            }
        }
        else if( loopTaskCount == 2)
        {
            targetPositionArm = -380;
            setMissionTo(Mission.MOVE_B2C);
        }
    }
    
    int targetPositionLifter;
    
    int targetPositionArm;
    ElapsedTime armMotorTimer = new ElapsedTime();
    int armTimeoutMillsecs = 1000;
    double armMovePower = 0.2;
    void armMotorRunnable()
    {
        if (targetPositionArm > armMotor.getCurrentPosition())
        {
            armMovePower = 0.3;
        }
        else if (targetPositionArm < armMotor.getCurrentPosition())
        {
            armMovePower = 0.1;
            //if over up 45-D, need more power:TODO
        }
        else
        {
            armMovePower = 0.2;
        }
        armMotor.setTargetPosition(targetPositionArm);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //runtimeManual.reset();
        armMotor.setPower(armMovePower);
    }
    
    int targetPositionRotator;
    ElapsedTime rotatorMotorTimer = new ElapsedTime();
    int rotatorTimeoutMillsecs = 2000;
    double rotatorMovePower = 0.2;
    void rotatorMotorRunnable()
    {
        if (!rotatorMotor.isBusy())
        {
            if (rotatorMotor.getCurrentPosition() != targetPositionRotator)
            {
                rotatorMotor.setTargetPosition(targetPositionRotator);
                rotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //rotatorTimeoutMillsecs = 1000;
                rotatorMotorTimer.reset();
                rotatorMotor.setPower(rotatorMovePower);
            }
            else
            {
                rotatorMotor.setPower(0);
            }
        }
        else
        {
            if (rotatorMotorTimer.milliseconds() > rotatorTimeoutMillsecs)
            {
                rotatorMotor.setPower(0);
                targetPositionRotator = rotatorMotor.getCurrentPosition();
            }
        }
    }
    

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    public void driveStraightInit(double maxDriveSpeed,
                              double distance,
                              double heading)
    {
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH);
        leftTarget = backLeft.getCurrentPosition() + moveCounts;
        rightTarget = backRight.getCurrentPosition() + moveCounts;
    
        // Set Target FIRST, then turn on RUN_TO_POSITION
        backLeft.setTargetPosition(leftTarget);
        backRight.setTargetPosition(rightTarget);
    
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        moveRobot(maxDriveSpeed, 0);
    }
    
    public boolean driveStraightLoop(double maxDriveSpeed,
                               double distance,
                               double heading)
    {
        // keep looping while we are still active, and BOTH motors are running.
        if (backLeft.isBusy() && backRight.isBusy())
        {
        
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
        
            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;
        
            // Apply the turning correction to the current driving speed.
            moveRobot(driveSpeed, turnSpeed);
        
            // Display drive status for the driver.
            //sendTelemetry(true);
            return false;
        }
        else
        {
            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return true;
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeadingInit(double maxTurnSpeed, double heading)
    {
        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);
    }
    public boolean turnToHeadingLoop(double maxTurnSpeed, double heading)
    {
        // keep looping while we are still active, and not on heading.
        if (Math.abs(headingError) > HEADING_THRESHOLD)
        {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
            return false;
        }
        else
        {
            // Stop all motion;
            moveRobot(0, 0);
            return true;
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    ElapsedTime holdTimer = new ElapsedTime();
    public void holdHeadingInit(double maxTurnSpeed, double heading, double holdTime)
    {
        holdTimer.reset();
    }
    public boolean holdHeadingLoop(double maxTurnSpeed, double heading, double holdTime)
    {
        if (holdTimer.time() < holdTime)
        {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            //sendTelemetry(false);
            return false;
        }
        else
        {
            // Stop all motion;
            moveRobot(0, 0);
            return true;
        }
    }
    
    public void driveStrafeInit(double Inches, double maxSpeed, int timeoutInSeconds, double target)
    {
        int a, b, c, d;
        // move to right side
        /*a = frontRight.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
        b = frontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        c = backRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        d = backLeft.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);*/
        
        // move to left side
        a = frontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        b = frontLeft.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
        c = backRight.getCurrentPosition() - (int) (Inches * COUNTS_PER_INCH);
        d = backLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
        
        //frontRight.setTargetPosition(a); TODO will add it later once cable is ready
        //frontLeft.setTargetPosition(b);
        backRight.setTargetPosition(c);
        backLeft.setTargetPosition(d);
        
        
        //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        runtime.reset();
        
        // move to right
        //frontLeft.setPower(0.2);
        //backRight.setPower(0.2);
        //frontRight.setPower(-0.2);
        //backLeft.setPower(-0.2);
        // move to left
        frontLeft.setPower(-0.2);
        backRight.setPower(-0.2);
        frontRight.setPower(0.2);
        backLeft.setPower(0.2);
    }
    
    ElapsedTime runtime = new ElapsedTime();
    
    public boolean driveStrafeLoop(double Inches, double maxSpeed, int timeoutInSeconds, double target)
    {
        if ((runtime.seconds() < timeoutInSeconds)
            //&& backLeft.isBusy()  && backRight.isBusy()
            // todo && frontRight.isBusy() && frontLeft.isBusy()
        )
        {
            /*if (Inches < 0)
            {
                frontLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                backLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                backRight.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                frontRight.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
            }
            else
            {
                frontLeft.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
                backLeft.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                backRight.setPower(Range.clip(maxSpeed - (getRawHeading() - target) / 100, -1, 1));
                frontRight.setPower(Range.clip(maxSpeed + (getRawHeading() - target) / 100, -1, 1));
            }*/
            if(!backRight.isBusy() || !backLeft.isBusy())
            {
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                return true;
            }
            else
            {
                frontLeft.setPower(-0.2);
                backRight.setPower(-0.2);
                frontRight.setPower(0.2);
                backLeft.setPower(0.2);
                return false;
            }
        }
        else
        {
            frontRight.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            backLeft.setPower(0);
            return true;
        }
    }
    
    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    private double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        //robotHeading = getRawHeading() - headingOffset;
        robotHeading = currentRobotAngles.firstAngle - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    private void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);
    }
    
    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    private double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    private void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
    
    // ****************  updating telemetry **********************
    double currentWristPosition;
    double currentPalmPosition;
    double currentKnucklePosition;
    double currentFingerPosition;
    Orientation currentRobotAngles;
    private void composeTelemetry() {
        
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            currentRobotAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentWristPosition = wristServo.getPosition();
            currentPalmPosition =  palmServo.getPosition();
            currentKnucklePosition = knuckleServo.getPosition();
            currentFingerPosition = fingerServo.getPosition();
        }
        });
        
        telemetry.addLine()
                .addData("Wrist", new Func<String>() {
                    @Override public String value() {
                        return formatServoPosition(currentWristPosition);
                    }
                })
                .addData("Palm", new Func<String>() {
                    @Override public String value() {
                        return formatServoPosition(currentPalmPosition);
                    }
                })
                .addData("Knuckle", new Func<String>() {
                    @Override public String value() {
                        return formatServoPosition(currentKnucklePosition);
                    }
                })
                .addData("Finger", new Func<String>() {
                    @Override public String value() {
                        return formatServoPosition(currentFingerPosition);
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
                        return formatAngle(currentRobotAngles.angleUnit, currentRobotAngles.firstAngle);
                    }
                })
                /*.addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(currentRobotAngles.angleUnit, currentRobotAngles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(currentRobotAngles.angleUnit, currentRobotAngles.thirdAngle);
                    }
                })*/;
    }
    
    private void sendTelemetry(boolean straight)
    {
        telemetry.addData("Lifter %d", lifterMotor.getCurrentPosition());
        telemetry.addData("Rotator %d", rotatorMotor.getCurrentPosition());
        telemetry.addData("Arm %d", armMotor.getCurrentPosition());
        
        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
        } else {
            telemetry.addData("Motion", "Turning");
        }
        
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.addData("Wheel Positions L:R",  "%7d:%7d",      backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());
    }
    
    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    
    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
    private String formatServoPosition(double servoPosition)
    {
        return String.format(Locale.getDefault(), "%.2f", servoPosition);
    }
}
