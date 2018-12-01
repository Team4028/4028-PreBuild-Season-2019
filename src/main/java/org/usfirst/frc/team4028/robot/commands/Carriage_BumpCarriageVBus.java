package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Carriage;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

public class Carriage_BumpCarriageVBus extends Command {
  public enum CARRIAGE_BUMP_FUNCTION {
    BumpUp, BumpDown,
  }
  Carriage _carriage = Carriage.getInstance();

  CARRIAGE_BUMP_FUNCTION _carriageBumpFunction;

  public Carriage_BumpCarriageVBus(CARRIAGE_BUMP_FUNCTION function) {
    //requires(_carriage);
    _carriageBumpFunction = function;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(_carriageBumpFunction == CARRIAGE_BUMP_FUNCTION.BumpUp){
      _carriage.carriageWheels_FeedOut_VBusCmd_BumpUp();
    }
    else if(_carriageBumpFunction == CARRIAGE_BUMP_FUNCTION.BumpDown){
      _carriage.carriageWheels_FeedOut_VBusCmd_BumpDown();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
