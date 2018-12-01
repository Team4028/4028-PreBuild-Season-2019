package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Chassis_DriveSetDistanceAction extends Command
{
	private double _targetDistance;

	Chassis _chassis = Chassis.getInstance();
	public Chassis_DriveSetDistanceAction (double targetDistance)
	{
		requires(_chassis);
		setInterruptible(false);
		_targetDistance = targetDistance;
	}
	@Override
	protected void initialize() 
	{
		System.out.println("Initializing command");
		_chassis.setMotionMagicCmdInches(_targetDistance);
	}
	@Override
	protected void execute() 
	{
		_chassis.updateChassis(Timer.getFPGATimestamp());
		//_chassis.moveToTargetPosDriveSetDistance();
		System.out.println("Current Chassis Error: " +  Double.toString(_chassis.get_leftPos()-_chassis._leftMtrDriveSetDistanceCmd));
	
	}

	@Override
	protected void interrupted() {
		System.out.println("Chassis Interrupted");
	}
	@Override
	protected boolean isFinished()
	{
		if(Math.abs(_chassis.get_leftPos()-_chassis._leftMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND
		&& Math.abs(_chassis.get_rightPos()-_chassis._rightMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND)
		{
			System.out.println("Chassis is Finished");
			return true;
		}
		else
		{
			return false;
		}
	}

	@Override
	protected void end(){
		_chassis.stop();
	}

}