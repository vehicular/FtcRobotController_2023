package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class HardwareMecanum {
    public DcMotor RFMotor = null;
    public DcMotor LFMotor = null;
    public DcMotor RBMotor = null;
    public DcMotor LBMotor = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    double LFMotorMultiplier = 1.0;
    double RFMotorMultiplier = 1.0;
    double LBMotorMultiplier = 1.0;
    double RBMotorMultiplier = 1.0;

    public HardwareMecanum() {

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        RFMotor = hwMap.get(DcMotor.class, "RFMotor");
        LFMotor = hwMap.get(DcMotor.class, "LFMotor");
        LBMotor = hwMap.get(DcMotor.class, "LBMotor");
        RBMotor = hwMap.get(DcMotor.class, "RBMotor");

        RFMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LFMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LBMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RFMotor.setPower(0);
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);

    }
}
