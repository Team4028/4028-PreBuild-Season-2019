package org.usfirst.frc.team4028.robot.commands.auton;

import org.usfirst.frc.team4028.robot.commands.Chassis_DriveSetDistanceAction;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class Auton_CG_AutoRun extends CommandGroup {

  public Auton_CG_AutoRun()
  {
    //addParallel(new Auton_ParallelStarter());
    addSequential(new Chassis_DriveSetDistanceAction(60));
    System.out.println("finish drivesetdistance");
  }
  /*
  public Auton_CG_AutoRun(){
    addSequential(new Chassis_ArcadeDriveAction(.8, 2));
    addParallel(new PrintTimeFromStart());
    
  }
  */

 
  }
