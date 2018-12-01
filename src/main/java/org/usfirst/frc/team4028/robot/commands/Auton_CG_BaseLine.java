package org.usfirst.frc.team4028.robot.commands;



import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths;
import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths.Center;
import org.usfirst.frc.team4028.robot.auton.pathfollowing.control.Path;


import edu.wpi.first.wpilibj.command.CommandGroup;

public class Auton_CG_BaseLine extends CommandGroup
{
    Path _baseLine = Paths.getPath(Center.AUTO_RUN);
    public Auton_CG_BaseLine()
    {
        //addParallel(new Auton_ParallelStarter());
        //addSequential(new Auton_RunMotionProfileAction(_baseLine));
        addParallel(new Auton_ParallelStarter());
        addSequential(new Chassis_DriveSetDistanceAction(30));    
    }
}