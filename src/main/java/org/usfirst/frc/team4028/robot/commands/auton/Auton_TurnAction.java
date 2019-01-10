package org.usfirst.frc.team4028.robot.commands.auton;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.command.Command;

public class Auton_TurnAction extends Command
{
    private Chassis _chassis = Chassis.getInstance();

    private double _targetAngle;
    private boolean _isTurnRight;
    private double _initialSign;

    public Auton_TurnAction(double angle, boolean isTurnRight)
    {
        _targetAngle = angle;
        _isTurnRight = isTurnRight;
    }
    @Override
    protected void initialize()
    {
        _chassis.setTargetAngleAndTurnDirection(_targetAngle, _isTurnRight);
        //_initialSign = (_chassis.get_Heading() - _targetAngle)/Math.abs(_chassis.get_Heading() - _targetAngle);
    }

    @Override
    protected void execute() 
    {
        _chassis.moveToTargetAngle();
    }
    
    @Override
    protected boolean isFinished() 
    { 
        //return !(_initialSign == (_chassis.get_Heading() - _targetAngle)/Math.abs(_chassis.get_Heading() - _targetAngle));
		return Math.abs(_targetAngle - _chassis.get_Heading()) < 2.5; // Returns true when chassis is within angle
                                                                     // deadband
    
    }
    @Override
    protected void end()
    {
        _chassis.stop();
        //_targetAngle = _chassis.get_Heading();
    }
}