package org.firstinspires.ftc.teamcode.core.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.Subsystem;

public class Eye extends Subsystem
{
    public Eye(HardwareMap map)
    {
        super(map);
    }
    
    @Override
    public void teleopInit(Subsystem otherSys)
    {
    
    }
    
    @Override
    public void teleopControls(Gamepad gamepad1, Gamepad gamepad2)
    {
    
    }
    
    @Override
    public void stop()
    {
    
    }
    
    @Override
    public void crossSubsystemCheck()
    {
    
    }
    
    @Override
    public String addTelemetry()
    {
        return null;
    }
    
    public enum ParkingZone
    {
        UNKNOWN,
        ONE,
        TWO,
        THREE,
    }
    public enum InitPosition
    {
        UNKNOWN,
        RED_LEFT,
        RED_RIGHT,
        BLUE_LEFT,
        BLUE_RIGHT,
    }
    public enum PolePosition
    {
        UNKNOWN,
        CENTER,
        LEFT,
        RIGHT,
        FRONT,
        BACK,
    }
    ElapsedTime timeout = new ElapsedTime();
    public ParkingZone ReadParkingID()// need timeout
    {
        timeout.reset();
        while(timeout.seconds()<5){}
        return ParkingZone.ONE;
    }
    
    public InitPosition ReadLeftWallSlotID( boolean isLeft )// need timeout
    {
        timeout.reset();
        while(timeout.seconds()<4){}
        return InitPosition.RED_RIGHT;
    }
    
    int count = 0;
    public PolePosition CheckLowPoleOnCenter()
    {
        timeout.reset();
        while(timeout.milliseconds()<200){}
        if(count < 20)
        {
            count++;
            return PolePosition.LEFT;
        }
        else
        {
            return PolePosition.CENTER;
        }
    }
    
    @Override
    public void autoInit()
    {
    }
    @Override
    
    public void autoControls(boolean isOpActive)
    {
    
    }
}
