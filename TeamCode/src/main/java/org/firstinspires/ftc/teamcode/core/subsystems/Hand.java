package org.firstinspires.ftc.teamcode.core.subsystems;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Subsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotorPositionCal;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class Hand extends Subsystem
{
    Subsystem crossSubsystem;
    
    enum SysState
    {
        RUN,
        RECALIBRATION,
        RECOVER,
    }
    
    /*
        enum HandStatus
        {
            NONE,
            MOVING,
            INIT,
            READY_PICKUP,
            PICKUP_DONE,
            READY_DROPOFF,
            DROPOFF_DONE,
        }
    
        enum FingerPosition
        {
            UNKNOWN,
            FLATE,
            DOWN_MID,
            DOWN_LEFT,
            DOWN_RIGHT,
        }
    */
    MotorPositionCal PredefinedPosition = new MotorPositionCal();
    MotorPositionCal.SubsystemPosition CurrentPositionBySet;
    MotorPositionCal.SubsystemPosition PreviousPositionBySet;
    String directoryPath = Environment.getExternalStorageDirectory().getPath() + "/MOTORS";
    
    private DcMotor lifterMotor;
    private DcMotor rotatorMotor;
    private DcMotor armMotor;
    private Servo wristServo;
    private Servo palmServo;
    private Servo knuckleServo;
    private Servo fingerServo;
    
    private SysState CurState = SysState.RUN;
    String FileOpTele = "";
    
    private int armPosition = 0;
    
    public Hand(HardwareMap hardwareMap)
    {
        super(hardwareMap);
        
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
        
        PredefinedPosition.InitPosition.SetValue(
                ReadPositionFromFile("InitMotorsPosition.json"));
        PredefinedPosition.Pickup_up.SetValue(
                ReadPositionFromFile("PickupUpMotorsPosition.json"));
        PredefinedPosition.Pickup_down.SetValue(
                ReadPositionFromFile("PickupDownMotorsPosition.json"));
        PredefinedPosition.Pickup_left.SetValue(
                ReadPositionFromFile("PickupLeftMotorsPosition.json"));
        PredefinedPosition.Pickup_right.SetValue(
                ReadPositionFromFile("PickupRightMotorsPosition.json"));
        PredefinedPosition.Drop_A_1.SetValue(
                ReadPositionFromFile("DropA1MotorsPosition.json"));
        PredefinedPosition.Drop_B_2.SetValue(
                ReadPositionFromFile("DropB2MotorsPosition.json"));
        PredefinedPosition.Drop_X_3.SetValue(
                ReadPositionFromFile("DropX3MotorsPosition.json"));
        PredefinedPosition.Drop_Y_4.SetValue(
                ReadPositionFromFile("DropY4MotorsPosition.json"));
    }
    
    @Override
    public String addTelemetry()
    {
        String s = "----HAND---- \n";
        s += "Lifter: " + lifterMotor.getCurrentPosition() + "\n";
        s += "Rotator: " + rotatorMotor.getCurrentPosition() + "\n";
        s += "Arm: " + armMotor.getCurrentPosition() + "\n";
        s += "wristServo: " + wristServo.getPosition() + "\n";
        s += "palmServo: " + palmServo.getPosition() + "\n";
        s += "knuckleServo: " + knuckleServo.getPosition() + "\n";
        s += "fingerServo: " + fingerServo.getPosition() + "\n";
        s += FileOpTele + "\n";
        return s;
    }
    
    @Override
    public void autoInit()
    {
    }
    
    @Override
    public void teleopInit(Subsystem otherSys)
    {
        crossSubsystem = otherSys;
        PreviousPositionBySet = CurrentPositionBySet = PredefinedPosition.InitPosition;
        SetMotorsPosition(PredefinedPosition.InitPosition);
        armPosition = armMotor.getCurrentPosition();
    }
    
    @Override
    public void crossSubsystemCheck()
    {
    }
    
    @Override
    public void stop()
    {
        // slowly move the arm and lifter down
        armMotor.setTargetPosition(-220);
        runtimeArm.reset();
        armMotor.setPower(0.1);
        while ((runtimeArm.seconds() < 2) &&
                (armMotor.isBusy()))
        {
        }
    
        if (lifterMotor.getCurrentPosition() > 100) // min low limit
        {
            lifterMotor.setTargetPosition(50);
            runtimeManual.reset();
            lifterMotor.setPower(0.2);
            while ((runtimeManual.seconds() < 1) &&
                    (lifterMotor.isBusy()))
            {
            }
        }
    }
    
    /**
     * Runs repeatedly during teleop. The right bumper toggles between control modes.
     * In the first control mode, 3 buttons move the flipper between 3 positions.
     * In the second control mode, the 2 buttons move theh flipper up and down.
     *
     * @param gamepad1
     * @param gamepad2
     */
    @Override
    public void teleopControls(Gamepad gamepad1, Gamepad gamepad2)
    {
        
        if (CurState == SysState.RUN)
        {
            if (gamepad1.start)
            {
                CurState = SysState.RECALIBRATION;
                //FileOpTele = "Enter RE-CALIBRATION ...";
            }
            else if (gamepad1.back)
            {
                CurState = SysState.RECOVER;
                //FileOpTele = "Enter RECOVER ...";
            }
        }
        
        if (CurState == SysState.RUN)
        {
            SetPredefinedHandMotors(gamepad1);
            TuningHandMotors(gamepad2);
        }
        else //if(CurState == SysState.RECALIBRATION || RECOVER)
        {
            RedoPredefinedHandMotors();
            CurState = SysState.RUN;
            //FileOpTele = "";
        }
    }
    
    boolean keylock_crossup = false;
    boolean keylock_crossdown = false;
    boolean keylock_crossleft = false;
    boolean keylock_crossright = false;
    boolean keylock_a = false;
    boolean keylock_b = false;
    boolean keylock_x = false;
    boolean keylock_y = false;
    
    private void SetPredefinedHandMotors(Gamepad gamepad)
    {
        // finger servo
        if (gamepad.left_trigger > TRIGGER_THRESHOLD)
        {
            if (!leftTriggerLock)
            {
                leftTriggerLock = true;
                if (gamepad.right_trigger < TRIGGER_THRESHOLD)
                { // pickup
                    // TODO: to be checked if fingers are down position
                    if (armMotor.getCurrentPosition() > -320) //TODO: to be calibrated
                    {
                        armMotor.setTargetPosition(-320);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        runtimeManual.reset();
                        armMotor.setPower(0.1);
                        while ((runtimeManual.seconds() < 1) &&
                                (armMotor.isBusy()))
                        {
                        }
                        // arm will automatically get back to starting position
                    }
                    fingerServo.setPosition(0);
                }
                else// if (gamepad.left_trigger && right_trigger > 0.75) { // dropoff
                {
                    armMotor.setTargetPosition(armMotor.getCurrentPosition() - 30);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtimeManual.reset();
                    armMotor.setPower(0.1);
                    while ((runtimeManual.seconds() < 1) &&
                            (armMotor.isBusy()))
                    {
                    }
                    runtimeManual.reset();
                    while (runtimeManual.milliseconds() < 200)
                    {
                    }
                    fingerServo.setPosition(1);
                    runtimeManual.reset();
                    while (runtimeManual.milliseconds() < 300)
                    {
                    }
                    // arm will automatically get back to starting position
                }
            }
        }
        else if (gamepad.left_trigger < TRIGGER_THRESHOLD &&
                gamepad.right_trigger < TRIGGER_THRESHOLD)
        {
            leftTriggerLock = false;
        }
        
        
        if (gamepad.dpad_up)
        {
            if (!keylock_crossup)
            {
                keylock_crossup = true;
                SetMotorsPosition(PredefinedPosition.Pickup_up);
            }
        }
        else
        {
            keylock_crossup = false;
        }
        
        if (gamepad.dpad_down)
        {
            if (!keylock_crossdown)
            {
                keylock_crossdown = true;
                SetMotorsPosition(PredefinedPosition.Pickup_down);
            }
        }
        else
        {
            keylock_crossdown = false;
        }
        
        if (gamepad.dpad_left)
        {
            if (!keylock_crossleft)
            {
                keylock_crossleft = true;
                SetMotorsPosition(PredefinedPosition.Pickup_left);
            }
        }
        else
        {
            keylock_crossleft = false;
        }
        
        if (gamepad.dpad_right)
        {
            if (!keylock_crossright)
            {
                keylock_crossright = true;
                SetMotorsPosition(PredefinedPosition.Pickup_right);
            }
        }
        else
        {
            keylock_crossright = false;
        }
        
        if (gamepad.a)
        {
            if (!keylock_a)
            {
                keylock_a = true;
                SetMotorsPosition(PredefinedPosition.Drop_A_1);
            }
        }
        else
        {
            keylock_a = false;
        }
        
        if (gamepad.b)
        {
            if (!keylock_b)
            {
                keylock_b = true;
                SetMotorsPosition(PredefinedPosition.Drop_B_2);
            }
        }
        else
        {
            keylock_b = false;
        }
        
        if (gamepad.x)
        {
            if (!keylock_x)
            {
                keylock_x = true;
                SetMotorsPosition(PredefinedPosition.Drop_X_3);
            }
        }
        else
        {
            keylock_x = false;
        }
        
        if (gamepad.y)
        {
            if (!keylock_y)
            {
                keylock_y = true;
                SetMotorsPosition(PredefinedPosition.Drop_Y_4);
            }
        }
        else
        {
            keylock_y = false;
        }
    }
    
    final double TRIGGER_THRESHOLD = 0.75;     // Squeeze more than 3/4 to get rumble.
    private boolean wristServoRightLock = false;
    private boolean wristServoLeftLock = false;
    private boolean palmServoRightLock = false;
    private boolean palmServoLeftLock = false;
    private boolean knuckleServoLeftLock = false;
    private boolean knuckleServoRightLock = false;
    private boolean lifterHighLock = false;
    private boolean lifterLowLock = false;
    private boolean leftTriggerLock = false;
    private boolean armPositionLock = false;
    ElapsedTime runtimeManual = new ElapsedTime();
    ElapsedTime runtimeArm = new ElapsedTime();
    
    private void TuningHandMotors(Gamepad gamepad)
    {
        //Wrist Servo
        if (gamepad.right_stick_y > 0.3)
        {
            if (!wristServoRightLock)
            {
                wristServoRightLock = true;  // Hold off any more triggers
                wristServo.setPosition(wristServo.getPosition() + 0.02);
            }
        }
        else
        {
            wristServoRightLock = false;  // We can trigger again now.
        }
        
        if (gamepad.right_stick_y < -0.3)
        {
            if (!wristServoLeftLock)
            {
                wristServoLeftLock = true;  // Hold off any more triggers
                wristServo.setPosition(wristServo.getPosition() - 0.02);
            }
        }
        else
        {
            wristServoLeftLock = false;  // We can trigger again now.
        }
        
        // palm servo
        if (gamepad.right_stick_x > 0.3)
        {
            if (!palmServoLeftLock)
            {
                palmServoLeftLock = true;
                palmServo.setPosition(palmServo.getPosition() + 0.05);
            }
        }
        else
        {
            palmServoLeftLock = false;
        }
        if (gamepad.right_stick_x < -0.3)
        {
            if (!palmServoRightLock)
            {
                palmServoRightLock = true;
                palmServo.setPosition(palmServo.getPosition() - 0.05);
            }
        }
        else
        {
            palmServoRightLock = false;
        }
        
        // Knuckle servo
        if (gamepad.b)
        {
            if (!knuckleServoLeftLock)
            {
                knuckleServoLeftLock = true;
                knuckleServo.setPosition(knuckleServo.getPosition() - 0.05);
            }
        }
        else
        {
            knuckleServoLeftLock = false;
        }
        if (gamepad.x)
        {
            if (!knuckleServoRightLock)
            {
                knuckleServoRightLock = true;
                knuckleServo.setPosition(knuckleServo.getPosition() + 0.05);
            }
        }
        else
        {
            knuckleServoRightLock = false;
        }
        
        // finger servo
        if (gamepad.left_trigger > TRIGGER_THRESHOLD)
        {
            if (!leftTriggerLock)
            {
                leftTriggerLock = true;
                if (gamepad.right_trigger < TRIGGER_THRESHOLD)
                { // pickup
                    // TODO: to be checked if fingers are down position
                    if (armMotor.getCurrentPosition() > -320) //TODO: to be calibrated
                    {
                        armMotor.setTargetPosition(-320);
                        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        runtimeManual.reset();
                        armMotor.setPower(0.1);
                        while ((runtimeManual.seconds() < 1) &&
                                (armMotor.isBusy()))
                        {
                        }
                        // arm will automatically get back to starting position
                    }
                    fingerServo.setPosition(0);
                }
                else// if (gamepad.left_trigger & right_trigger > 0.75) { // dropoff
                {
                    armMotor.setTargetPosition(armMotor.getCurrentPosition() - 30);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtimeManual.reset();
                    armMotor.setPower(0.1);
                    while ((runtimeManual.seconds() < 1) &&
                            (armMotor.isBusy()))
                    {
                    }
                    runtimeManual.reset();
                    while (runtimeManual.milliseconds() < 200)
                    {
                    }
                    fingerServo.setPosition(1);
                    runtimeManual.reset();
                    while (runtimeManual.milliseconds() < 300)
                    {
                    }
                    // arm will automatically get back to starting position
                }
            }
        }
        else if (gamepad.left_trigger < TRIGGER_THRESHOLD &&
                gamepad.right_trigger < TRIGGER_THRESHOLD)
        {
            leftTriggerLock = false;
        }
        if (gamepad.y)
        {
            fingerServo.setPosition(1);//open
        }
        if (gamepad.a)
        {
            fingerServo.setPosition(0);// close
        }
        
        
        // arm motor
        double y = -gamepad.left_stick_y;

            /*double armPower = 0.35;
            if (gamepad.dpad_up) {
                armPosition += 25;
                armPower = 0.35;
            } else if (gamepad.dpad_down) {
                armPosition -= 25;
                armPower = 0.1;
            }
            else // stay
            {
                armPower = 0.2;
            }*/
        if (y > 0.1)
        {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setPower(y * 0.4);
            armPositionLock = false;
        }
        else if (y < -0.1)
        {
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (armMotor.getCurrentPosition() > 900)// TODO: COS/SIN calculation
            {
                armMotor.setPower(y * 0.1);
            }
            else
            {
                armMotor.setPower(y * 0.02);
            }
            armPositionLock = false;
        }
        else
        {
            if (armPositionLock == false)
            {
                armMotor.setPower(0);
                armPositionLock = true;
                armPosition = armMotor.getCurrentPosition();
            }
            
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setTargetPosition(armPosition);
            runtimeArm.reset();
            armMotor.setPower(0.1);
            while ((runtimeArm.seconds() < 2) &&
                    (armMotor.isBusy()))
            {
            }
        }
        
        
        // rotator motor
        double x = -gamepad.left_stick_x;
        rotatorMotor.setPower(0.15 * x);
            /*if (gamepad.dpad_left) {
                rotatorMotor.setPower(0.2);
            } else if (gamepad.dpad_right) {
                rotatorMotor.setPower(-0.2);
            } else {
                rotatorMotor.setPower(0);
            }*/
        
        //lifter motor
        if (gamepad.left_bumper)
        {//up
            if (!lifterHighLock)
            {
                lifterHighLock = true;  // Hold off any more triggers
                if (lifterMotor.getCurrentPosition() < 2000)
                { // max high limit
                    lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition() + 100);
                    lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtimeManual.reset();
                    lifterMotor.setPower(0.2);
                    while ((runtimeManual.seconds() < 1) &&
                            (lifterMotor.isBusy()))
                    {
                    }
                }
                //gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
            }
        }
        else
        {
            lifterHighLock = false;  // We can trigger again now.
        }
        
        if (gamepad.right_bumper)
        {//down
            if (!lifterLowLock)
            {
                lifterLowLock = true;
                if (lifterMotor.getCurrentPosition() > 100) // min low limit
                {
                    lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition() - 100);
                    lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtimeManual.reset();
                    lifterMotor.setPower(0.2);
                    while ((runtimeManual.seconds() < 1) &&
                            (lifterMotor.isBusy()))
                    {
                    }
                }
            }
        }
        else
        {
            lifterLowLock = false;
        }
    }
    
    private void RedoPredefinedHandMotors()
    {
        if (CurState == SysState.RECALIBRATION)
        {
            if (CurrentPositionBySet == PredefinedPosition.Pickup_up)
            {
                GetMotorsPosition(PredefinedPosition.Pickup_up);
                SavePositonsToFile("PickupUpMotorsPosition.json",
                        PredefinedPosition.Pickup_up);
            }
            else if (CurrentPositionBySet == PredefinedPosition.Pickup_down)
            {
                GetMotorsPosition(PredefinedPosition.Pickup_down);
                SavePositonsToFile("PickupDownMotorsPosition.json",
                        PredefinedPosition.Pickup_down);
            }
            else if (CurrentPositionBySet == PredefinedPosition.Pickup_left)
            {
                GetMotorsPosition(PredefinedPosition.Pickup_left);
                SavePositonsToFile("PickupLeftMotorsPosition.json",
                        PredefinedPosition.Pickup_left);
            }
            else if (CurrentPositionBySet == PredefinedPosition.Pickup_right)
            {
                GetMotorsPosition(PredefinedPosition.Pickup_right);
                SavePositonsToFile("PickupRightMotorsPosition.json",
                        PredefinedPosition.Pickup_right);
            }
            else if (CurrentPositionBySet == PredefinedPosition.Drop_A_1)
            {
                GetMotorsPosition(PredefinedPosition.Drop_A_1);
                SavePositonsToFile("DropA1MotorsPosition.json",
                        PredefinedPosition.Drop_A_1);
            }
            else if (CurrentPositionBySet == PredefinedPosition.Drop_B_2)
            {
                GetMotorsPosition(PredefinedPosition.Drop_B_2);
                SavePositonsToFile("DropB2MotorsPosition.json",
                        PredefinedPosition.Drop_B_2);
            }
            else if (CurrentPositionBySet == PredefinedPosition.Drop_X_3)
            {
                GetMotorsPosition(PredefinedPosition.Drop_X_3);
                SavePositonsToFile("DropX3MotorsPosition.json",
                        PredefinedPosition.Drop_X_3);
            }
            else if (CurrentPositionBySet == PredefinedPosition.Drop_Y_4)
            {
                GetMotorsPosition(PredefinedPosition.Drop_Y_4);
                SavePositonsToFile("DropY4MotorsPosition.json",
                        PredefinedPosition.Drop_Y_4);
            }
        }
        else if (CurState == SysState.RECOVER)
        {
            if (CurrentPositionBySet == PredefinedPosition.Pickup_up)
            {
                RecoverPositionFile("PickupUpMotorsPosition.json");
                PredefinedPosition.Pickup_up.SetValue(
                        ReadPositionFromFile("PickupUpMotorsPosition.json"));
            }
            else if (CurrentPositionBySet == PredefinedPosition.Pickup_down)
            {
                RecoverPositionFile("PickupDownMotorsPosition.json");
                PredefinedPosition.Pickup_down.SetValue(
                        ReadPositionFromFile("PickupDownMotorsPosition.json"));
            }
            else if (CurrentPositionBySet == PredefinedPosition.Pickup_left)
            {
                RecoverPositionFile("PickupLeftMotorsPosition.json");
                PredefinedPosition.Pickup_left.SetValue(
                        ReadPositionFromFile("PickupLeftMotorsPosition.json"));
            }
            else if (CurrentPositionBySet == PredefinedPosition.Pickup_right)
            {
                RecoverPositionFile("PickupRightMotorsPosition.json");
                PredefinedPosition.Pickup_right.SetValue(
                        ReadPositionFromFile("PickupRightMotorsPosition.json"));
            }
            else if (CurrentPositionBySet == PredefinedPosition.Drop_A_1)
            {
                RecoverPositionFile("DropA1MotorsPosition.json");
                PredefinedPosition.Drop_A_1.SetValue(
                        ReadPositionFromFile("DropA1MotorsPosition.json"));
            }
            else if (CurrentPositionBySet == PredefinedPosition.Drop_B_2)
            {
                RecoverPositionFile("DropB2MotorsPosition.json");
                PredefinedPosition.Drop_B_2.SetValue(
                        ReadPositionFromFile("DropB2MotorsPosition.json"));
            }
            else if (CurrentPositionBySet == PredefinedPosition.Drop_X_3)
            {
                RecoverPositionFile("DropX3MotorsPosition.json");
                PredefinedPosition.Drop_X_3.SetValue(
                        ReadPositionFromFile("DropX3MotorsPosition.json"));
            }
            else if (CurrentPositionBySet == PredefinedPosition.Drop_Y_4)
            {
                RecoverPositionFile("DropY4MotorsPosition.json");
                PredefinedPosition.Drop_Y_4.SetValue(
                        ReadPositionFromFile("DropY4MotorsPosition.json"));
            }
        }
    }
    
    ////////////////////////////////////////////////////////////////////
    
    private String[] ReadPositionFromFile(String fileName)
    {
        String[] data = new String[MotorPositionCal.maxNumOfMotors];
        try
        {
            FileReader fileReader = new FileReader(
                    directoryPath + "/" + fileName);
            BufferedReader bufferedReader = new BufferedReader(fileReader);
            StringBuilder stringBuilder = new StringBuilder();
            String oneLine = bufferedReader.readLine();
            while (oneLine != null)
            {
                stringBuilder.append(oneLine).append("\n");
                oneLine = bufferedReader.readLine();
            }
            bufferedReader.close();
            fileReader.close();
            // This objStr will have Json Format String
            String objStr = stringBuilder.toString();
            
            JSONObject jsonObject = new JSONObject(objStr);
            data[MotorPositionCal.LifterMotorInt] =
                    ((jsonObject.get(MotorPositionCal.LifterMotorStr).toString()));
            data[MotorPositionCal.RotatorMotorInt] =
                    ((jsonObject.get(MotorPositionCal.RotatorMotorStr).toString()));
            data[MotorPositionCal.ArmMotorInt] =
                    ((jsonObject.get(MotorPositionCal.ArmMotorStr).toString()));
            data[MotorPositionCal.WristServoInt] =
                    ((jsonObject.get(MotorPositionCal.WristServoStr).toString()));
            data[MotorPositionCal.PalmServoInt] =
                    ((jsonObject.get(MotorPositionCal.PalmServoStr).toString()));
            data[MotorPositionCal.KnuckleServoInt] =
                    ((jsonObject.get(MotorPositionCal.KnuckleServoStr).toString()));
            data[MotorPositionCal.FingerServoInt] =
                    ((jsonObject.get(MotorPositionCal.FingerServoStr).toString()));
        } catch (Exception e)
        {
            FileOpTele = ("Read " + fileName + " Error..." + e.toString());
        }
        return data;
    }
    
    private void SavePositonsToFile(String fileName, MotorPositionCal.SubsystemPosition positions)
    {
        JSONObject InitData = new JSONObject();
        try
        {
            InitData.put(MotorPositionCal.LifterMotorStr, positions.LifterMotor);
            InitData.put(MotorPositionCal.RotatorMotorStr, positions.RotatorMotor);
            InitData.put(MotorPositionCal.ArmMotorStr, positions.ArmMotor);
            InitData.put(MotorPositionCal.WristServoStr, positions.WristServo);
            InitData.put(MotorPositionCal.PalmServoStr, positions.PalmServo);
            InitData.put(MotorPositionCal.KnuckleServoStr, positions.KnuckleServo);
            InitData.put(MotorPositionCal.FingerServoStr, positions.FingerServo);
            
            // Convert JsonObject to String Format
            String userString = InitData.toString();
            //telemetry.addLine(userString);
            // Define the File Path and its Name
            File directory = new File(directoryPath);
            directory.mkdir();
            FileWriter fileWriter = new FileWriter(
                    directoryPath + "/" + fileName);
            
            fileWriter.write(userString);
            fileWriter.close();
        } catch (Exception e)
        {
            FileOpTele = ("Write " + fileName + " Error..." + e.toString());
        }
    }
    
    private void RecoverPositionFile(String fileName)
    {
        try
        {
            FileReader fileReader = new FileReader(
                    directoryPath + "/bk_" + fileName);
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
            // This dataStr will have Json Format String
            String dataStr = stringBuilder.toString();
            
            FileWriter fileWriter = new FileWriter(
                    directoryPath + "/" + fileName);
            fileWriter.write(dataStr);
            fileWriter.close();
        } catch (Exception e)
        {
            FileOpTele = ("Copy " + fileName + " Error..." + e.toString());
        }
    }
    
    private void SetMotorsPosition(MotorPositionCal.SubsystemPosition targetPosition)
    {
        if (targetPosition != CurrentPositionBySet)
        {
            PreviousPositionBySet = CurrentPositionBySet;
            CurrentPositionBySet = targetPosition;
        }
        
        boolean doArmFirst = false;
        double armPow = 0.1;
        
        if (targetPosition.ArmMotor > armMotor.getCurrentPosition())
        {
            doArmFirst = true;
        }
        
        if (armMotor.getCurrentPosition() < -200)
        {
            if (targetPosition.PalmServo > 0.8 && PreviousPositionBySet.PalmServo < 0.8)
            {
                doArmFirst = true;
            }
            else if (targetPosition.PalmServo < 0.8 && PreviousPositionBySet.PalmServo > 0.8)
            {
                doArmFirst = false;
            }
        }
        if (targetPosition.ArmMotor > armMotor.getCurrentPosition())
        {
            armPow = 0.3;
        }
        else
        {
            if (armMotor.getCurrentPosition() > 900)// TODO: COS/SIN calculation
            {
                armPow = (0.1);
            }
            else
            {
                armPow = (0.1);
            }
        }
        
        if (!doArmFirst)
        {
            wristServo.setPosition(targetPosition.WristServo);
            palmServo.setPosition(targetPosition.PalmServo);
            knuckleServo.setPosition(targetPosition.KnuckleServo);
            fingerServo.setPosition(targetPosition.FingerServo);
        }
        
        armPosition = targetPosition.ArmMotor;
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(armPosition);
        runtimeArm.reset();
        armMotor.setPower(armPow);
        while ((runtimeArm.seconds() < 2) &&
                (armMotor.isBusy()))
        {
        }
        
        if (doArmFirst)
        {
            wristServo.setPosition(targetPosition.WristServo);
            palmServo.setPosition(targetPosition.PalmServo);
            knuckleServo.setPosition(targetPosition.KnuckleServo);
            fingerServo.setPosition(targetPosition.FingerServo);
        }
    }
    
    private void GetMotorsPosition(MotorPositionCal.SubsystemPosition position)
    {
        position.WristServo = wristServo.getPosition();
        position.PalmServo = palmServo.getPosition();
        position.KnuckleServo = knuckleServo.getPosition();
        position.FingerServo = fingerServo.getPosition();
        position.ArmMotor = armMotor.getCurrentPosition();
        position.RotatorMotor = rotatorMotor.getCurrentPosition();
        position.LifterMotor = lifterMotor.getCurrentPosition();
    }
}
