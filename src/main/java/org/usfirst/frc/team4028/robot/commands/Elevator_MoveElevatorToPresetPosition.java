package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Carriage;
import org.usfirst.frc.team4028.robot.subsystems.Elevator;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

public class Elevator_MoveElevatorToPresetPosition extends Command {
    private Elevator _elevator = Elevator.getInstance();
    private Carriage _carriage = Carriage.getInstance();
	ELEVATOR_TARGET_POSITION _presetPosition;
    long _startTime;
    long _endTime;

    public Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION presetPosition) {
        requires(_elevator);
        requires(_carriage);
        setInterruptible(true);
        _presetPosition = presetPosition;
       
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        _elevator.resetElevatorBumpValue();
        System.out.println("TARGET CALLED");
        System.out.println("TARGET CALLED");
        System.out.println("TARGET CALLED");
        _startTime = System.currentTimeMillis();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if(!_elevator.isFlapUpEnabledHeight()){
            _carriage.tiltCarriageDown();
        }
    	_elevator.MoveToPresetPosition(_presetPosition);    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (Elevator.getInstance().get_isAtTargetPosition())
        {
            System.out.println("AAHHHHHHHHHHHHHHHHH");
            System.out.println("AAHHHHHHHHHHHHHHHHH");
            System.out.println("AAHHHHHHHHHHHHHHHHH");
            System.out.println("AAHHHHHHHHHHHHHHHHH");
            System.out.println("AAHHHHHHHHHHHHHHHHH");
            System.out.println("AAHHHHHHHHHHHHHHHHH");
            System.out.println("AAHHHHHHHHHHHHHHHHH");
            _endTime = System.currentTimeMillis();
            System.out.println("Second Call Start Minus Eend:  " + (_endTime - _startTime)); 
            return true;
        }
        else{
            return false;
        }
        

    }

    private void PrintTimeFromStart() {
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
