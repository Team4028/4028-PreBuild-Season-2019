package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.GearHandler;

import edu.wpi.first.wpilibj.command.Command;

public class ZeroGearTilt extends Command
{
    GearHandler _gearHandler = GearHandler.getInstance();
    double _targetPos = _gearHandler.getCurrentPosition();
    public ZeroGearTilt()
    {
        setInterruptible(false);
    }
    @Override
    protected void initialize()
    {
        _gearHandler.moveToZero();
    }
    @Override
    protected void execute() {}

    @Override
    protected boolean isFinished() 
    {
            return _gearHandler.getIsOnTiltHomeLimtSwitch();
	}
    
    @Override
    protected void end() 
    {
        _gearHandler.zeroSensors();
        _gearHandler.stop();
     
    }
}

   