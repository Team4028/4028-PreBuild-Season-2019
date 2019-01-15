package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.GearHandler;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Gear_OutFeed extends Command
{
    double _startTime;
    double _percentOutput;

    GearHandler _gearHandler = GearHandler.getInstance();
    public Gear_OutFeed(double percentOutput)
    {
       _percentOutput=percentOutput;
    }
    @Override
    protected void initialize() 
    {
        _startTime = Timer.getFPGATimestamp();    
    }
    @Override
    protected void execute() 
    {
        _gearHandler.SpinInfeedWheelsVBus(_percentOutput); //_percentOutput*.5*-1.0 is the current input for SpinInfeedWheelsVBus
    }
    @Override
    protected boolean isFinished() 
    {
        if(Timer.getFPGATimestamp() - _startTime >= 2)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}