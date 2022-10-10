package org.firstinspires.ftc.teamcode.core.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Subsystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Hand extends Subsystem
{
    final double TRIGGER_THRESHOLD  = 0.75;

    private double pos = 0;

    private DcMotor lifterMotor;
    private DcMotor rotatorMotor;
    private DcMotor armMotor;
    private Servo wristServo;
    private Servo palmServo;
    private Servo knukcleServo;
    private Servo fingerServo;

    private boolean altControl;


    private int rotatorPosition;
    private int armPosition;

    ElapsedTime runtime = new ElapsedTime();
    int newArmTarget;

    boolean highLevel = false;
    int lifterZero;

    public Hand(HardwareMap hardwareMap)
    {
        super(hardwareMap);
        fingerServo = hardwareMap.servo.get(Constants.fingerServo);
        wristServo = hardwareMap.servo.get(Constants.wristServo);
        palmServo = hardwareMap.servo.get(Constants.palmServo);
        knukcleServo = hardwareMap.servo.get(Constants.knuckleServo);
        pos = 0;
        altControl = false;

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

        rotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotatorPosition = 0;
        armPosition = 0;


        newArmTarget = armMotor.getCurrentPosition();
        lifterZero = lifterMotor.getCurrentPosition();
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
        //Wrist Servo
        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            if (!highLevel) {
                highLevel = true;  // Hold off any more triggers
                wristServo.setPosition(wristServo.getPosition()+0.25);
            }
        } else {
            highLevel = false;  // We can trigger again now.
        }

        if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            wristServo.setPosition(wristServo.getPosition()-0.25);
        }

        // palm servo
        if (gamepad1.right_bumper)
        {
            double palmCurPosition = palmServo.getPosition();
            if (palmCurPosition >0.5)
            {
                palmCurPosition = 0;
            }
            else
            {
                palmCurPosition = 1;
            }
            palmServo.setPosition(palmCurPosition);
        }

        // Knuckle servo
        if (gamepad1.left_bumper)
        {
            double knuckleCurPosition = knukcleServo.getPosition();
            if (knuckleCurPosition >0.7)
            {
                knuckleCurPosition = 0;
            }
            else if(knuckleCurPosition > 0.3)
            {
                knuckleCurPosition = 1;
            }
            else
            {
                knuckleCurPosition = 0.5;
            }
            knukcleServo.setPosition(knuckleCurPosition);
        }

        // finger servo
        if (gamepad1.x) {
            fingerServo.setPosition(1);
        }
        if (gamepad1.b) {
            fingerServo.setPosition(0);
        }

        // arm motor
        double armPower = 0.35;
        if(gamepad1.dpad_up)
        {
            newArmTarget += 50;
        }else if(gamepad1.dpad_down) {
            newArmTarget -= 50;
            armPower = 0.1;
        }
        armMotor.setTargetPosition(newArmTarget);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        armMotor.setPower(armPower);
        while ((runtime.seconds() < 1) &&
                (armMotor.isBusy() )) {

        }

        // rotator motor
        if(gamepad1.dpad_left) {
            rotatorMotor.setPower(0.2);
        }
        else if(gamepad1.dpad_right)
        {
            rotatorMotor.setPower(-0.2);
        }
        else
        {
            rotatorMotor.setPower(0);
        }

        //lifter motor
        if (gamepad1.y) {
            if (!highLevel) {
                highLevel = true;  // Hold off any more triggers
                lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition()+100);
                lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                lifterMotor.setPower(0.2);
                while ((runtime.seconds() < 1) &&
                        (lifterMotor.isBusy() )) {

                }
                gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
            }
        } else {
            highLevel = false;  // We can trigger again now.
        }

        if (gamepad1.a) {
            lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition()-100);
            lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            lifterMotor.setPower(0.2);
            while ((runtime.seconds() < 1) &&
                    (lifterMotor.isBusy() )) {

            }
        }
    }

    @Override
    public String addTelemetry()
    {
        String s = "Finger Position: \t" + fingerServo.getPosition();
        return s;
    }

    @Override
    public void stop()
    {

    }
    @Override
    public void autoInit()
    {

    }

    public void setTargetPosition(int position)
    {
        fingerServo.setPosition(position);
    }

@Override
    public void teleopInit(){}
}
