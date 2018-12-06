package org.usfirst.frc.team4028.robot.commands.auton;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.ParamEnum;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Auton_PIDTune_finshUp extends Command {

	public TalonSRX _talon;
	public int parameterSlot;

	public Auton_PIDTune_finshUp(Subsystem requiredSubsystem, TalonSRX talon, int srxParameterSlot) {
		this._talon = talon;
		this.parameterSlot = srxParameterSlot;
		requires(requiredSubsystem);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {

        System.out.println("//////////////////////////////////////////////////////////////////////////");
        System.out.println("//////////////////////////////////////////////////////////////////////////");
		System.out.println(
				"Talon F gain set to: " + _talon.configGetParameter(ParamEnum.eProfileParamSlot_F, parameterSlot, 10));
		System.out.println(
                "Talon P gain set to: " + _talon.configGetParameter(ParamEnum.eProfileParamSlot_P, parameterSlot, 10));
        System.out.println("//////////////////////////////////////////////////////////////////////////");
        System.out.println("//////////////////////////////////////////////////////////////////////////");
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}