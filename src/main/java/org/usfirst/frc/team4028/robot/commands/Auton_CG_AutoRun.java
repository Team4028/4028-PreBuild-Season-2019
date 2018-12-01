package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;


public class Auton_CG_AutoRun extends CommandGroup {

  public Auton_CG_AutoRun(){
    addParallel(new Auton_ParallelStarter());
    addSequential(new Chassis_DriveSetDistanceAction(0));
  }
  /*
  public Auton_CG_AutoRun(){
    addSequential(new Chassis_ArcadeDriveAction(.8, 2));
    addParallel(new PrintTimeFromStart());
    
  }
  */

 
  }




