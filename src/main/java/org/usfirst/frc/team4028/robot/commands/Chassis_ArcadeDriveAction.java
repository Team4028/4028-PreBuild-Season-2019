package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Chassis_ArcadeDriveAction extends Command
{
    private Chassis _chassis = Chassis.getInstance();
    double _waitTime;
    double _throttle;
    double _turn = 0; //Intitialized to 0 so that if constructor has no turn it will turn
    double _startTime; 



    public Chassis_ArcadeDriveAction(double throttle, double waitTime){
    _throttle = throttle;
    _waitTime = waitTime;
    }

    public Chassis_ArcadeDriveAction(double throttle, double waitTime, double turn){
        _throttle = throttle;
        _waitTime = waitTime;
        _turn = turn;
    }


    protected void initialize() {    	
        _startTime = Timer.getFPGATimestamp();
		_chassis.setHighGear(false);
    }

    
    protected void execute() {
        if ((Timer.getFPGATimestamp() - _startTime) > _waitTime) {
			_chassis.stop();
		} else {
			_chassis.arcadeDrive(-_throttle, _turn);
		}
    }

    protected boolean isFinished() {
        return (Timer.getFPGATimestamp() - _startTime) > _waitTime;
    }

    protected void end() {
        _chassis.stop();
    }


    protected void interrupted() {
    }


}