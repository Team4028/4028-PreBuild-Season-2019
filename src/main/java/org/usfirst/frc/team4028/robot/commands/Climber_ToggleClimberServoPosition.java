package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Climber;
import org.usfirst.frc.team4028.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
//#endregion

/**
 * This command implements support for toggling the position of the Climber Servo
 */
public class Climber_ToggleClimberServoPosition extends Command 
{
    private Elevator _elevator = Elevator.getInstance();
    private Climber _climber = Climber.getInstance();
	
    public Climber_ToggleClimberServoPosition() {
        // Use requires() here to declare subsystem dependencies
        requires(_climber);
        requires(_elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	setTimeout(4);  // set 4 second timeout
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
    	if(_climber.getIsServoCurrentTargetClosed() && _elevator.isClimberServoEnabledHeight()) {
    		_climber.openServo();
    	}
    	else if(_climber.getIsServoCurrentTargetClosed() && !_elevator.isClimberServoEnabledHeight()) {
    		DriverStation.reportWarning("Here", true);
    	} else {
    		_climber.closeServo();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// either we get to target position or we timed out
   	 	return _climber.getIsServoInPosition() || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
