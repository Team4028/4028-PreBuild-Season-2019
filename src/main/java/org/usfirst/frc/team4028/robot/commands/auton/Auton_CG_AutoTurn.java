package org.usfirst.frc.team4028.robot.commands.auton;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team4028.robot.commands.auton.Auton_TurnAction;

public class Auton_CG_AutoTurn extends CommandGroup
{
    public Auton_CG_AutoTurn()
    {
        addSequential(new Auton_TurnAction(90, true));
    }

}
