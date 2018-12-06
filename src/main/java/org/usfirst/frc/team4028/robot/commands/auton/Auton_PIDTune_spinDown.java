package org.usfirst.frc.team4028.robot.commands.auton;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Auton_PIDTune_spinDown extends Command {

	private TalonSRX _talon;

	public Auton_PIDTune_spinDown(Subsystem requiredSubsystem, TalonSRX talon) {
		_talon = talon;
		requires(requiredSubsystem);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		_talon.configOpenloopRamp(0.0, 10);
		_talon.set(ControlMode.PercentOutput, 0.0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return _talon.getSelectedSensorVelocity(0) == 0;
	}

	// Called once after isFinished returns true
	protected void end() {
		System.out.println("Motor stopped.");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		_talon.set(ControlMode.PercentOutput, 0);
	}
}