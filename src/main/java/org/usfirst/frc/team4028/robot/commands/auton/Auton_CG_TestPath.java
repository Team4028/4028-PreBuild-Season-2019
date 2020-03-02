package org.usfirst.frc.team4028.robot.commands.auton;

import org.usfirst.frc.team4028.robot.auton.control.Path;
import org.usfirst.frc.team4028.robot.auton.Paths;
import org.usfirst.frc.team4028.robot.auton.Paths.Center;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Auton_CG_TestPath extends CommandGroup
{
    public Auton_CG_TestPath() 
    { 
        Path _path = Paths.getPath(Center.TEST);
        addParallel(new Auton_ParallelStarter());
        addSequential(new Auton_RunMotionProfileAction(_path));
    }

}