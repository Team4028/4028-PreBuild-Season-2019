package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.OI;
import org.usfirst.frc.team4028.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

/**
 * This command implements support for climbing
 */
public class Climber_ClimbWithControllers extends Command {

	private Climber _climber = Climber.getInstance();
	private OI _oi = OI.getInstance();
	
    public Climber_ClimbWithControllers() {
        // Use requires() here to declare subsystem dependencies
        requires(_climber);
        setInterruptible(false);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	super.execute();
    
    	_climber.runMotor(_oi.getOperator_Climber_JoystickCmd());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
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
