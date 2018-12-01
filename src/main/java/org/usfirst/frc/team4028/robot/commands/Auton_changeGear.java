package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

/**
 * This command implements support for Shifting Gears on the Drive Chassis
 */
public class Auton_changeGear extends Command 
{
    private boolean highGearQ;
	
	private Chassis _chassis = Chassis.getInstance();

    public Auton_changeGear(boolean isHighGear) {
        // Use requires() here to declare subsystem dependencies
        requires(Chassis.getInstance());
        setInterruptible(true);
        highGearQ = isHighGear; 
    }

    // Called just before this Command runs the first time
    protected void initialize() {

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        _chassis.setHighGear(highGearQ);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}