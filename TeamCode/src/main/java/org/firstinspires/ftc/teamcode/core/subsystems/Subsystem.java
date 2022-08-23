package org.firstinspires.ftc.teamcode.core.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PID;

import java.util.ArrayList;

public abstract class Subsystem
{
    private ArrayList<PID> pidLoops = new ArrayList<PID>();

    HardwareMap hardwaremap;

    public Subsystem(HardwareMap map)
    {
        this.hardwaremap = map;
    }

    public abstract void teleopControls(Gamepad gamepad1, Gamepad gamepad2);

    public abstract String addTelemetry();

    public abstract void stop();

    public void autoInit()
    {

    }

    public void teleopInit()
    {

    }
}
