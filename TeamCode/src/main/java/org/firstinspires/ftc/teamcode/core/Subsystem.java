package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PID;

import java.util.ArrayList;

public abstract class Subsystem
{
    //subsystem could be executed in standalone (thread)

    private ArrayList<PID> pidLoops = new ArrayList<PID>();

    HardwareMap hardwaremap;

    public Subsystem(HardwareMap map)
    {
        this.hardwaremap = map;
    }

    public abstract void teleopControls(Gamepad gamepad1, Gamepad gamepad2);

    public abstract String addTelemetry();

    public abstract void stop();

    public abstract void autoInit();

    public abstract void teleopInit();

    public void CrossSubsystemCheck()
    {
        // 1) finger is too low while driving
        // 2) finger needs chassis's help to reach a specified point(3d)
    }
}