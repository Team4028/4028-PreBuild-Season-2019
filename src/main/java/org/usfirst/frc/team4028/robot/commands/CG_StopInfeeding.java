package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.commands.Carriage_RunCarriageWheels;
import org.usfirst.frc.team4028.robot.commands.Infeed_RunInfeedWheels;
import org.usfirst.frc.team4028.robot.commands.Carriage_RunCarriageWheels.CARRIAGE_WHEELS_FUNCTION;
import org.usfirst.frc.team4028.robot.commands.Infeed_RunInfeedWheels.INFEED_WHEELS_FUNCTION;

import edu.wpi.first.wpilibj.command.CommandGroup;
//#endregion

public class CG_StopInfeeding extends CommandGroup {
  public CG_StopInfeeding() {
    addParallel(new Carriage_RunCarriageWheels(CARRIAGE_WHEELS_FUNCTION.StopWheels));
    addSequential(new Infeed_RunInfeedWheels(INFEED_WHEELS_FUNCTION.StopWheels));
  }
}
