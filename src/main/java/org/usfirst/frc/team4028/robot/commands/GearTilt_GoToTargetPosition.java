package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.GearHandler;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class GearTilt_GoToTargetPosition extends Command 
{
    double _targetPosition;
    double _tolerance = 20;
    double _timeout = 5;
    double _startTime;
    GearHandler _gearHandler = GearHandler.getInstance();

    public GearTilt_GoToTargetPosition(double targetPos, double timeout)
    {
        _targetPosition = targetPos;
        _timeout = timeout;
    }

    public GearTilt_GoToTargetPosition(double targetPos)
    {
        _targetPosition = targetPos;
    }

    @Override
    protected void initialize() 
    {
        _startTime = Timer.getFPGATimestamp();
        _gearHandler.moveTowardsTargetPosition(_targetPosition);

    }
    @Override
    protected void execute() 
    {

    }

    @Override
    protected boolean isFinished() 
    {
        return _gearHandler.getError() < _tolerance || Timer.getFPGATimestamp() - _startTime > _timeout;
    }
}