package org.usfirst.frc.team4028.robot.commands.auton;

import org.usfirst.frc.team4028.robot.auton.RobotState;
import org.usfirst.frc.team4028.robot.auton.control.Path;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Auton_RunMotionProfileAction extends Command
{
    Chassis _chassis = Chassis.getInstance();
    private Path _path;
    private double _startTime;

    public Auton_RunMotionProfileAction(Path p)
    {
        requires(_chassis);
        _path = p;
    }

    @Override
    protected void initialize() {
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), _path.getStartPose());
		_chassis.setWantDrivePath(_path, _path.isReversed());
		_chassis.setHighGear(true);
		_startTime = Timer.getFPGATimestamp();
    }
    @Override
    protected void execute() {
        if(Timer.getFPGATimestamp() - _startTime > 0.25) {
			if(_chassis.get_leftPos() == 0 || _chassis.get_rightPos() == 0) {
				_chassis.forceDoneWithPath();
				System.out.println("Attention Idiots: You Morons Forgot to Plug in The Encoder");
			}
		}
    }
    @Override
    protected boolean isFinished() {
        return _chassis.isDoneWithPath();
    }
    @Override
    protected void end() {
        _chassis.stop();
    }

}