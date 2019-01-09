package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.util.BeakXboxController.Thumbstick;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

/**
 * This command implements support for driving the chassis
 * 	FYI: This is the default command for for Chassis Subsystem
 */
public class Chassis_DriveWithControllers extends Command 
{
	private Chassis _chassis = Chassis.getInstance();
    
    private Thumbstick _leftThumbstick;
    private Thumbstick _rightThumbstick;
	
    public Chassis_DriveWithControllers(Thumbstick leftThumbstick, Thumbstick righThumbstick) {
        requires(_chassis);
        setInterruptible(true);
        _leftThumbstick = leftThumbstick;
        _rightThumbstick = righThumbstick;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
        _chassis.arcadeDrive(_leftThumbstick.getY(), _rightThumbstick.getX());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}