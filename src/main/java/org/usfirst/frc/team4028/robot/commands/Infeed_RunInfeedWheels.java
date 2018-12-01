package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Infeed;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

public class Infeed_RunInfeedWheels extends Command {
  public enum INFEED_WHEELS_FUNCTION{
    Infeed, Outfeed, SpinCW, SpinCCW, StopWheels,
  }
  private Infeed _infeed = Infeed.getInstance();
  private INFEED_WHEELS_FUNCTION _infeedWheelsFunctions;

  private boolean _areInfeedWheelsStopped = true;

  public Infeed_RunInfeedWheels(INFEED_WHEELS_FUNCTION function) {
    requires(_infeed);
    _infeedWheelsFunctions = function;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(_infeed.get_isSafeToRunInfeedWheels()){
      if(_infeedWheelsFunctions == INFEED_WHEELS_FUNCTION.Infeed){
        _infeed.feedIn();
      }
      else if(_infeedWheelsFunctions == INFEED_WHEELS_FUNCTION.Outfeed){
        _infeed.feedOut();
      }
      else if(_infeedWheelsFunctions == INFEED_WHEELS_FUNCTION.SpinCW){
        _infeed.infeedWheels_SpinCube_CW();
      }
      else if(_infeedWheelsFunctions == INFEED_WHEELS_FUNCTION.SpinCCW){
        _infeed.infeedWheels_SpinCube_CCW();
      }
      else if (_infeedWheelsFunctions == INFEED_WHEELS_FUNCTION.StopWheels){
        _infeed.stopInfeedWheels();
        _areInfeedWheelsStopped = true;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _areInfeedWheelsStopped;    
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
