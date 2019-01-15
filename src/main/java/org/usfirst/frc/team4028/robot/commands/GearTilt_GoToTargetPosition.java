package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.GearHandler;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class GearTilt_GoToTargetPosition extends Command 
{
    double _targetPosition;
    double _tolerance;
    double _timeout;
    double _startTime;
    GearHandler _gearHandler = GearHandler.getInstance();
    public GearTilt_GoToTargetPosition(double targetPos, double tolerance, double timeout)
    {
        _targetPosition = targetPos;
        _tolerance = tolerance;
        _timeout = timeout;
    }
    @Override
    protected void initialize() 
    {
        _startTime = Timer.getFPGATimestamp();
        GearHandler._targetPos = _targetPosition;
    }
    @Override
    protected void execute() 
    {
        _gearHandler.moveTowardsTargetPosition(_targetPosition);
    }
    @Override
    protected boolean isFinished() 
    {
        return _gearHandler.getError() < _tolerance || Timer.getFPGATimestamp() - _startTime > _timeout;
    }
}