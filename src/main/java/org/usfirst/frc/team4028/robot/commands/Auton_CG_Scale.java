package org.usfirst.frc.team4028.robot.commands;

import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths;
import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths.Left;
import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths.Right;
import org.usfirst.frc.team4028.robot.auton.pathfollowing.control.Path;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class Auton_CG_Scale extends CommandGroup
{

    public Auton_CG_Scale(boolean isScaleLeft, boolean isStartingLeft)
    {
        double elevatorWaitTime;
        boolean actuateFlapJack;
        double remainingPathDistance;
        Path toScale;
        if(isStartingLeft)
        {
            if(isScaleLeft)
            {
                toScale = Paths.getPath(Left.L_SCALE);
                elevatorWaitTime = 1.2;
                actuateFlapJack = false;
                remainingPathDistance = 18;
            }
            else
            {
                toScale = Paths.getPath(Left.R_SCALE);
                elevatorWaitTime = 3.0;
                actuateFlapJack = true;
                remainingPathDistance = 12;

            }
        }
        else
        {
            if(isScaleLeft)
            {
                toScale = Paths.getPath(Right.L_SCALE);
                elevatorWaitTime = 3.0;
                actuateFlapJack = true;
                remainingPathDistance = 12;
            }
            else
            {
                toScale = Paths.getPath(Right.R_SCALE);
                elevatorWaitTime = 1.2;
                actuateFlapJack = false;
                remainingPathDistance = 18;
            }
        }

        addParallel(new Auton_RunTimedMotionProfileCommand(toScale, 3));
            addSequential(new WaitCommand(elevatorWaitTime));
        addParallel(new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.SCALE_HEIGHT));
            addSequential(new Carriage_ToggleFlapSolenoid(actuateFlapJack));
        addSequential(new Auton_WaitUntilRemaingDistance(remainingPathDistance));
        addSequential(new CG_OutfeedCube());
        addSequential(new Chassis_DriveSetDistanceAction(-10));
        addSequential(new Carriage_ToggleFlapSolenoid(false));
        addSequential(new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.INFEED_HEIGHT));
        addSequential(new CG_OutfeedCube());
        addSequential(new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.HOME));
    }
}