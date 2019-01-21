package org.usfirst.frc.team4028.robot.commands.util;

import edu.wpi.first.wpilibj.command.Command;

public class PrintTimeFromStart extends Command
{
    public PrintTimeFromStart() {
    }

    protected void initialize() {    	
    }

    
    protected void execute() {
    }

    protected boolean isFinished() {
        return true;
    }

    protected void end() {
        System.out.println(super.timeSinceInitialized());
    }

    protected void interrupted() {
    }


}