package org.usfirst.frc.team4028.robot.commands.auton;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.command.Command;

public class Auton_TurnAction extends Command
{
    private Chassis _chassis = Chassis.getInstance();

    private double _targetAngle;
    private boolean _isTurnRight;

    public Auton_TurnAction(double angle, boolean isTurnRight)
    {
        _targetAngle = angle;
        _isTurnRight = isTurnRight;
    }
    @Override
    public void initialize()
    {
        _chassis.setTargetAngleAndTurnDirection(_targetAngle, _isTurnRight);
    }

    @Override
    public void execute() {}
    
    @Override
    public boolean isFinished() 
    {
        if(_targetAngle==180||_targetAngle==-180)
		{
			return _chassis.get_Heading() > 178 || _chassis.get_Heading() < -178;
		}
		else
		{
			return Math.abs(_targetAngle - _chassis.get_Heading()) < 2.5; // Returns true when chassis is within angle
                                                                          // deadband
		}
    }
}