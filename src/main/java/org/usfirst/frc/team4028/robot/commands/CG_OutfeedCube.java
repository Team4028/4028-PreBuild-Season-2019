package org.usfirst.frc.team4028.robot.commands;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.commands.Carriage_RunCarriageWheels;
import org.usfirst.frc.team4028.robot.commands.Infeed_RunInfeedWheels;

import org.usfirst.frc.team4028.robot.commands.Carriage_RunCarriageWheels.CARRIAGE_WHEELS_FUNCTION;
import org.usfirst.frc.team4028.robot.commands.Infeed_RunInfeedWheels.INFEED_WHEELS_FUNCTION;

import edu.wpi.first.wpilibj.command.CommandGroup;
//#endregion

public class CG_OutfeedCube extends CommandGroup {
  public CG_OutfeedCube() {
    addParallel(new Carriage_RunCarriageWheels(CARRIAGE_WHEELS_FUNCTION.Outfeed));
    addSequential(new Infeed_RunInfeedWheels(INFEED_WHEELS_FUNCTION.Outfeed));
  }
}
