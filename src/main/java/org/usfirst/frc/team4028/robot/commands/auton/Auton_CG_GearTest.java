package org.usfirst.frc.team4028.robot.commands.auton;

import org.usfirst.frc.team4028.robot.commands.GearTilt_GoToTargetPosition;
import org.usfirst.frc.team4028.robot.commands.ZeroGearTilt;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class Auton_CG_GearTest extends CommandGroup
{
    GearHandler _gearHandler = GearHandler.getInstance(); //Auton for testing the Gear-Related Motors, Currently tests outfeeding a gear onto a peg
    public Auton_CG_GearTest()
    {
        addParallel(new Auton_ParallelStarter());
        addSequential(new ZeroGearTilt());
        addSequential(new GearTilt_GoToTargetPosition(_gearHandler.getScorePosition()));
        addSequential(new WaitCommand("In Position Wait Command", 2));
        addSequential(new Auton_CG_Score());
    }
}