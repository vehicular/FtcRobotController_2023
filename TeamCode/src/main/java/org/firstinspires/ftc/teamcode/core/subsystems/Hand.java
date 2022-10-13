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
import java.io.FileReader;

public class Hand extends Subsystem {
    MotorPositionCal PredefinedPosition = new MotorPositionCal();
    String directoryPath = Environment.getExternalStorageDirectory().getPath() + "/MOTORS";

    private DcMotor lifterMotor;
    private DcMotor rotatorMotor;
    private DcMotor armMotor;
    private Servo wristServo;
    private Servo palmServo;
    private Servo knukcleServo;
    private Servo fingerServo;


    public Hand(HardwareMap hardwareMap) {
        super(hardwareMap);
        fingerServo = hardwareMap.servo.get(Constants.fingerServo);
        wristServo = hardwareMap.servo.get(Constants.wristServo);
        palmServo = hardwareMap.servo.get(Constants.palmServo);
        knukcleServo = hardwareMap.servo.get(Constants.knuckleServo);

        lifterMotor = hardwareMap.get(DcMotor.class, Constants.lifterMotor);
        rotatorMotor = hardwareMap.get(DcMotor.class, Constants.rotatorMotor);
        armMotor = hardwareMap.get(DcMotor.class, Constants.armMotor);

        //lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rotatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PredefinedPosition.InitPosition.SetValue(ReadPositionFromFile("InitMotorsPosition.json"));
        PredefinedPosition.Pickup_up.SetValue(ReadPositionFromFile("PickupUpMotorsPosition.json"));
        PredefinedPosition.Pickup_down.SetValue(ReadPositionFromFile("PickupDownMotorsPosition.json"));
        PredefinedPosition.Pickup_left.SetValue(ReadPositionFromFile("PickupLeftMotorsPosition.json"));
        PredefinedPosition.Pickup_right.SetValue(ReadPositionFromFile("PickupRightMotorsPosition.json"));
        PredefinedPosition.Drop_A_1.SetValue(ReadPositionFromFile("DropA1MotorsPosition.json"));
        PredefinedPosition.Drop_B_2.SetValue(ReadPositionFromFile("DropB2MotorsPosition.json"));
        PredefinedPosition.Drop_X_3.SetValue(ReadPositionFromFile("DropX3MotorsPosition.json"));
        PredefinedPosition.Drop_Y_4.SetValue(ReadPositionFromFile("DropY4MotorsPosition.json"));
    }


    private String[] ReadPositionFromFile(String fileName) {
        String[] data = new String[MotorPositionCal.maxNumOfMotors];
        try {
            FileReader fileReader = new FileReader(
                    directoryPath + "/" + fileName);
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

            JSONObject jsonObject = new JSONObject(responce);
            data[MotorPositionCal.LifterMotorInt] = ((jsonObject.get(MotorPositionCal.LifterMotorStr).toString()));
            data[MotorPositionCal.RotatorMotorInt] = ((jsonObject.get(MotorPositionCal.RotatorMotorStr).toString()));
            data[MotorPositionCal.ArmMotorInt] = ((jsonObject.get(MotorPositionCal.ArmMotorStr).toString()));
            data[MotorPositionCal.WristServoInt] = ((jsonObject.get(MotorPositionCal.WristServoStr).toString()));
            data[MotorPositionCal.PalmServoInt] = ((jsonObject.get(MotorPositionCal.PalmServoStr).toString()));
            data[MotorPositionCal.KnuckleServoInt] = ((jsonObject.get(MotorPositionCal.KnuckleServoStr).toString()));
            data[MotorPositionCal.FingerServoInt] = ((jsonObject.get(MotorPositionCal.FingerServoStr).toString()));
        } catch (Exception e) {
            //telemetry.addLine("Read " + fileName + " Error..." + e.toString());
        }
        return data;
    }

    private void SetMotorsPosition(MotorPositionCal.StepPosition positions) {
        wristServo.setPosition(positions.WristServo);
        palmServo.setPosition(positions.PalmServo);
        knukcleServo.setPosition(positions.KnuckleServo);
        fingerServo.setPosition(positions.FingerServo);
        armPosition = positions.ArmMotor;
        armMotor.setTargetPosition(armPosition);
    }

    /**
     * Runs repeatedly during teleop. The right bumper toggles between control modes.
     * In the first control mode, 3 buttons move the flipper between 3 positions.
     * In the second control mode, the 2 buttons move theh flipper up and down.
     *
     * @param gamepad1
     * @param gamepad2
     */
    private int armPosition = 0;
    @Override
    public void teleopControls(Gamepad gamepad1, Gamepad gamepad2) {
        ManualAdjustHandMotors(gamepad1);
        SetPredefinedHandMotors(gamepad2);
        TuningHandMotors(gamepad2);
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
    ElapsedTime runtimeManual = new ElapsedTime();

    //private boolean rampUp[] = new boolean[]{false, false, false, false, false, false, false};
    //private double wristServoPosition = 0;

    private void ManualAdjustHandMotors(Gamepad gamepad) {

        if(gamepad.right_trigger > TRIGGER_THRESHOLD || gamepad.left_trigger > TRIGGER_THRESHOLD)
        {
            //Wrist Servo
            if (gamepad.dpad_up) {
                if (!wristServoRightLock) {
                    wristServoRightLock = true;  // Hold off any more triggers
                    wristServo.setPosition(wristServo.getPosition() + 0.02);
                }
            } else {
                wristServoRightLock = false;  // We can trigger again now.
            }

            if (gamepad.dpad_down) {
                if (!wristServoLeftLock) {
                    wristServoLeftLock = true;  // Hold off any more triggers
                    wristServo.setPosition(wristServo.getPosition() - 0.02);
                }
            } else {
                wristServoLeftLock = false;  // We can trigger again now.
            }

            // palm servo
            if (gamepad.y) {
                if (!palmServoLeftLock) {
                    palmServoLeftLock = true;
                    palmServo.setPosition(palmServo.getPosition() + 0.02);
                }
            } else {
                palmServoLeftLock = false;
            }
            if (gamepad.a) {
                if (!palmServoRightLock) {
                    palmServoRightLock = true;
                    palmServo.setPosition(palmServo.getPosition() - 0.02);
                }
            } else {
                palmServoRightLock = false;
            }

            // Knuckle servo
            if (gamepad.dpad_left) {
                if (!knuckleServoLeftLock) {
                    knuckleServoLeftLock = true;
                    knukcleServo.setPosition(knukcleServo.getPosition() + 0.05);
                }
            } else {
                knuckleServoLeftLock = false;
            }
            if (gamepad.dpad_right) {
                if (!knuckleServoRightLock) {
                    knuckleServoRightLock = true;
                    knukcleServo.setPosition(knukcleServo.getPosition() - 0.05);
                }
            } else {
                knuckleServoRightLock = false;
            }

            // finger servo
            if (gamepad.x) { //open
                fingerServo.setPosition(1);
            }
            if (gamepad.b) { // close
                fingerServo.setPosition(0);
            }
        }
        else
        //if(gamepad.left_trigger <= TRIGGER_THRESHOLD && gamepad.right_trigger <= TRIGGER_THRESHOLD)
        {
            // arm motor
            double armPower = 0.35;
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
            }
            armMotor.setTargetPosition(armPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtimeManual.reset();
            armMotor.setPower(armPower);
            while ((runtimeManual.seconds() < 1) &&
                    (armMotor.isBusy())) {
            }

            // rotator motor
            if (gamepad.dpad_left) {
                rotatorMotor.setPower(0.2);
            } else if (gamepad.dpad_right) {
                rotatorMotor.setPower(-0.2);
            } else {
                rotatorMotor.setPower(0);
            }

            //lifter motor
            if (gamepad.left_bumper) {//up
                if (!lifterHighLock) {
                    lifterHighLock = true;  // Hold off any more triggers
                    if (lifterMotor.getCurrentPosition() < 2000) { // max high limit
                        lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition() + 100);
                        lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        runtimeManual.reset();
                        lifterMotor.setPower(0.2);
                        while ((runtimeManual.seconds() < 1) &&
                                (lifterMotor.isBusy())) {
                        }
                    }
                    //gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
                }
            } else {
                lifterHighLock = false;  // We can trigger again now.
            }

            if (gamepad.right_bumper) {//down
                if (!lifterLowLock) {
                    lifterLowLock = true;
                    if (lifterMotor.getCurrentPosition() > 100) // min low limit
                    {
                        lifterMotor.setTargetPosition(lifterMotor.getCurrentPosition() - 100);
                        lifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        runtimeManual.reset();
                        lifterMotor.setPower(0.2);
                        while ((runtimeManual.seconds() < 1) &&
                                (lifterMotor.isBusy())) {
                        }
                    }
                }
            } else {
                lifterLowLock = false;
            }
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

    public void SetPredefinedHandMotors(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            if (!keylock_crossup) {
                keylock_crossup = true;
                SetMotorsPosition(PredefinedPosition.Pickup_up);
            }
        } else {
            keylock_crossup = false;
        }

        if (gamepad.dpad_down) {
            if (!keylock_crossdown) {
                keylock_crossdown = true;
                SetMotorsPosition(PredefinedPosition.Pickup_down);
            }
        } else {
            keylock_crossdown = false;
        }

        if (gamepad.dpad_left) {
            if (!keylock_crossleft) {
                keylock_crossleft = true;
                SetMotorsPosition(PredefinedPosition.Pickup_left);
            }
        } else {
            keylock_crossleft = false;
        }

        if (gamepad.dpad_right) {
            if (!keylock_crossright) {
                keylock_crossright = true;
                SetMotorsPosition(PredefinedPosition.Pickup_right);
            }
        } else {
            keylock_crossright = false;
        }

        if (gamepad.a) {
            if (!keylock_a) {
                keylock_a = true;
                SetMotorsPosition(PredefinedPosition.Drop_A_1);
            }
        } else {
            keylock_a = false;
        }

        if (gamepad.b) {
            if (!keylock_b) {
                keylock_b = true;
                SetMotorsPosition(PredefinedPosition.Drop_B_2);
            }
        } else {
            keylock_b = false;
        }

        if (gamepad.x) {
            if (!keylock_x) {
                keylock_x = true;
                SetMotorsPosition(PredefinedPosition.Drop_X_3);
            }
        } else {
            keylock_x = false;
        }

        if (gamepad.y) {
            if (!keylock_y) {
                keylock_y = true;
                SetMotorsPosition(PredefinedPosition.Drop_Y_4);
            }
        } else {
            keylock_y = false;
        }
    }

    public void TuningHandMotors(Gamepad gamepad)
    {

    }

    @Override
    public String addTelemetry() {
        String s = "Lifter: " + lifterMotor.getCurrentPosition() + "; ";

        s += "Rotator: " + rotatorMotor.getCurrentPosition() + "; ";

        s += "Arm: " + armMotor.getCurrentPosition() + "; ";

        s += "wristServo: " + wristServo.getPosition() + "; ";

        s += "palmServo: " + palmServo.getPosition() + "; ";

        s += "knukcleServo: " + knukcleServo.getPosition() + "; ";

        s += "fingerServo: " + fingerServo.getPosition() + "; ";

        return s;
    }

    @Override
    public void stop() {

    }

    @Override
    public void autoInit() {

    }

    @Override
    public void teleopInit() {
        SetMotorsPosition(PredefinedPosition.InitPosition);
        armPosition =  armMotor.getCurrentPosition();
    }

    public void setFingerTargetPosition(int position) {
        fingerServo.setPosition(position);
    }
}
