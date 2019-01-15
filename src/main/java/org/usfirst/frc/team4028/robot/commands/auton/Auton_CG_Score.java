package org.usfirst.frc.team4028.robot.commands.auton;

import java.util.Arrays;

import org.usfirst.frc.team4028.robot.commands.Chassis_ArcadeDriveAction;
import org.usfirst.frc.team4028.robot.commands.Chassis_DriveSetDistanceAction;
import org.usfirst.frc.team4028.robot.commands.GearTilt_GoToTargetPosition;
import org.usfirst.frc.team4028.robot.commands.Gear_OutFeed;
import org.usfirst.frc.team4028.robot.commands.util.Simultaneous_Command;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class Auton_CG_Score extends CommandGroup
{
    public Auton_CG_Score()
    {
        GearHandler _gearHandler = GearHandler.getInstance();
        addParallel(new Auton_ParallelStarter());
        addParallel(new Simultaneous_Command(Arrays.asList(new Command[]
        {
            new GearTilt_GoToTargetPosition(_gearHandler.getFloorPosition()),
            new Chassis_ArcadeDriveAction(.5, 0.3),
            new Gear_OutFeed(1)
        })));
        addSequential(new WaitCommand("Outfeeding", 1));
        addSequential(new Gear_OutFeed(0));
    }
}