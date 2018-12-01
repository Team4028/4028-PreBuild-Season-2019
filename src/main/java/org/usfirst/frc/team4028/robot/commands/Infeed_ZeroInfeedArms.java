package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Infeed;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

/**
 * This command implements support for ReZeroing the Infeed Arms
 */
public class Infeed_ZeroInfeedArms extends Command 
{
	private Infeed _infeed = Infeed.getInstance();
	
    public Infeed_ZeroInfeedArms() {
        // Use requires() here to declare subsystem dependencies
        requires(_infeed);
        setInterruptible(false);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	_infeed.initReZeroArms();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	_infeed.zeroArms();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return _infeed.get_hasArmsBeenZeroed();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
