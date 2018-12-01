package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Elevator;
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
	private Elevator _elevator = Elevator.getInstance();
    
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
    protected void execute() {
       // System.out.println("Throttle:" + _leftThumbstick.getY() + "  Turn:" + _rightThumbstick.getX());
    	// optionally throttle speed if elevator is up to prevent tipping
		if (_elevator.isElevatorAtUnsafeHeight()) {
			_chassis.arcadeDrive(0.5 * _leftThumbstick.getY(), 0.8 * _rightThumbstick.getX());
		} else {
			_chassis.arcadeDrive(_leftThumbstick.getY(), _rightThumbstick.getX());
        }
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
