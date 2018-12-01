 package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Carriage;
import org.usfirst.frc.team4028.robot.subsystems.Elevator;
import org.usfirst.frc.team4028.robot.util.BeakXboxController.Thumbstick;

import edu.wpi.first.wpilibj.command.Command;
//#endregion

public class Carriage_ToggleFlapSolenoid extends Command {
  Carriage _carriage = Carriage.getInstance();
  Elevator _elevator = Elevator.getInstance();
  Thumbstick _thumbstick;
  boolean _isFlapUp;

  public Carriage_ToggleFlapSolenoid(Thumbstick thumbstick) {
    requires(_carriage);
    _thumbstick = thumbstick;
  }
  public Carriage_ToggleFlapSolenoid(boolean isFlapUp) {
    requires(_carriage);
    _isFlapUp = isFlapUp;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override  
  protected void execute() {
    if(_thumbstick !=null)
    {
      if(_elevator.isFlapUpEnabledHeight()){
        if(_thumbstick.getY() > 0){
          _carriage.tiltCarriageUp();
        }
        else if(_thumbstick.getY() < 0){
          _carriage.tiltCarriageDown();
        }
      }
    }
    else
    {
      if(_elevator.isFlapUpEnabledHeight()){
        if(_thumbstick.getY() > 0){
          _carriage.tiltCarriageUp();
        }
        else if(_thumbstick.getY() < 0){
          _carriage.tiltCarriageDown();
        }
      }
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
