package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

import edu.wpi.first.wpilibj.command.CommandGroup;
//#endregion

public class CG_ClimbPosition extends CommandGroup {
  public CG_ClimbPosition() {
    addParallel(new Climber_ToggleClimberServoPosition());
    addSequential(new Infeed_MoveInfeedArmsToPresetPosition(INFEED_ARM_TARGET_POSITION.CLIMB));
  }
}
