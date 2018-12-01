package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

public class Elevator_ZeroElevator extends Command {
	private Elevator _elevator = Elevator.getInstance();
	
    public Elevator_ZeroElevator() {
        // Use requires() here to declare subsystem dependencies
        requires(_elevator);
        setInterruptible(false);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	_elevator.initReZeroElevator();
    	setTimeout(5);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	_elevator.zeroElevator();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return _elevator.get_hasElevatorBeenZeroed() || isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
