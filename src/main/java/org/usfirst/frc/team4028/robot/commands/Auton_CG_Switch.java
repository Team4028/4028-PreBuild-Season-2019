package org.usfirst.frc.team4028.robot.commands;

import java.util.Arrays;



import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths;
import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths.Center;
import org.usfirst.frc.team4028.robot.subsystems.Carriage.CARRIAGE_WHEELS_OUT_VBUS_INDEX;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;


import org.usfirst.frc.team4028.robot.auton.pathfollowing.control.Path;
import org.usfirst.frc.team4028.robot.commands.Carriage_RunCarriageWheels.CARRIAGE_WHEELS_FUNCTION;
import org.usfirst.frc.team4028.robot.commands.Infeed_RunInfeedWheels.INFEED_WHEELS_FUNCTION;

public class Auton_CG_Switch extends CommandGroup {
	Path toSwitch;
	double elevatorWaitTime;
	
	public Auton_CG_Switch(boolean isSwitchLeft) {
        
		if (isSwitchLeft) 
			toSwitch = Paths.getPath(Center.L_SWITCH);
		else
			toSwitch = Paths.getPath(Center.R_SWITCH);

        elevatorWaitTime = 1.0;
        addParallel(new Infeed_MoveInfeedArmsToPresetPosition(INFEED_ARM_TARGET_POSITION.STORE));
        addParallel(new Auton_ParallelStarter());
        addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {
                /*new PrintCommand("Time to Print Command: "+ Double.toString(timeSinceInitialized())),*/
                new Auton_RunTimedMotionProfileCommand(toSwitch, 3.0),
           
                new Series_Command(Arrays.asList(new Command[] {
                    new PrintCommand("Series Command Started"),
                    new PrintCommand(Double.toString(timeSinceInitialized())),
                    new PrintCommand("Preparing to Start Wait Command"),
                    new WaitCommand(elevatorWaitTime),
                    new PrintCommand("Wait Command Terminated"),
                    new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.SWITCH_HEIGHT),
                    new PrintCommand("Elevator Command Terminated"),
                }))
                        })));
        // addSequential( new PrintCommand("YIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEEYIPEE"));
        // Outfeed cube for 0.2s

        System.out.println("Preparing to Outfeed Cube");
        addSequential(new CG_OutfeedCube(), .5);
        addSequential(new Carriage_RunCarriageWheels(CARRIAGE_WHEELS_FUNCTION.StopWheels), 0);
        addSequential(new Infeed_RunInfeedWheels(INFEED_WHEELS_FUNCTION.StopWheels), 0);
        addSequential(new PrintTimeFromStart());
        // Move Elevator back to floor
        addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {
                new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.INFEED_HEIGHT),
                new Chassis_DriveSetDistanceAction(-20.0)
        })));
        
        /*
        addParallel(new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.SWITCH_HEIGHT));
        addSequential(new PrintCommand("It Thought Wrong"));
        addSequential(new PrintCommand("It Thought Wrong"));
        addSequential(new PrintCommand("It Thought Wrong"));
        addSequential(new PrintCommand("It Thought Wrong"));
        addSequential(new PrintCommand("It Thought Wrong"));
        */
        
	}
	

}