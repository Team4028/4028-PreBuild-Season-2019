package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Carriage;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

public class Carriage_ToggleSqueezeSolenoid extends Command {
  Carriage _carriage = Carriage.getInstance();
  CARRIAGE_SQUEEZE_FUNCTIONS _carriageSolenoidFunction;
  public enum CARRIAGE_SQUEEZE_FUNCTIONS{
    Squeeze, Wide,
  }

  public Carriage_ToggleSqueezeSolenoid(CARRIAGE_SQUEEZE_FUNCTIONS function) {
    requires(_carriage);
    _carriageSolenoidFunction = function;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(_carriageSolenoidFunction == CARRIAGE_SQUEEZE_FUNCTIONS.Squeeze){
      _carriage.moveCarriageToSqueezeWidth();
    }
    else if(_carriageSolenoidFunction == CARRIAGE_SQUEEZE_FUNCTIONS.Wide){
      _carriage.moveCarriageToWideWidth();
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
