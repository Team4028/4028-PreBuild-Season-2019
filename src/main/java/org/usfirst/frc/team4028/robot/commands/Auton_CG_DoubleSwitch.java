package org.usfirst.frc.team4028.robot.commands;

import java.util.Arrays;



import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths;
import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths.Center;
import org.usfirst.frc.team4028.robot.auton.pathfollowing.control.Path;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;
//import org.usfirst.frc.team4028.robot.auton.paths.Paths.Center;
//import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_PRESET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;


public class Auton_CG_DoubleSwitch extends CommandGroup
{

    Path toSwitch; // Cube 1
	Path fromSwitchToFrontOfPyramidPath, toPyramid, fromPyramid, sTurnToSwitch; // Cube 2
	Path awayFromSwitch, pyramidAgain;
	double elevatorWaitTimeFirstCube, elevatorWaitTimeSecondCube;
   
    public Auton_CG_DoubleSwitch(boolean isLeftSwitch)
    {
        /////////////////////////////////////////////////////////////////////////////////////////
        // Constuctor Stuff ////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        if (isLeftSwitch) {
			toSwitch = Paths.getPath(Center.L_SWITCH);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(Center.L_SWITCH_TO_PYRAMID_FRONT);
			sTurnToSwitch = Paths.getPath(Center.S_TURN_TO_L_SWITCH);
			awayFromSwitch = Paths.getPath(Center.AWAY_FROM_L_SWITCH);
			pyramidAgain = Paths.getPath(Center.PYRAMID_AGAIN_FROM_L);
		} else {
			toSwitch = Paths.getPath(Center.R_SWITCH);
			fromSwitchToFrontOfPyramidPath = Paths.getPath(Center.R_SWITCH_TO_PYRAMID_FRONT);
			sTurnToSwitch = Paths.getPath(Center.S_TURN_TO_R_SWITCH);
			awayFromSwitch = Paths.getPath(Center.AWAY_FROM_R_SWITCH);
			pyramidAgain = Paths.getPath(Center.PYRAMID_AGAIN_FROM_R);
		}
		
		elevatorWaitTimeFirstCube = 1.0;
		elevatorWaitTimeSecondCube = 0.8;
		
		toPyramid = Paths.getPath(Center.TO_PYRAMID);
        fromPyramid = Paths.getPath(Center.FROM_PYRAMID);     
        
        /////////////////////////////////////////////////////////////////////////////////////////
        // Routine Stuff ///////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {	
            new Auton_RunTimedMotionProfileCommand(toSwitch, 2.6),
            new Series_Command(Arrays.asList(new Command[] {
                    new WaitCommand(elevatorWaitTimeFirstCube),
                    new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.SWITCH_HEIGHT)
            }))
        })));
        addSequential(new PrintTimeFromStart());
		// Outfeed cube for 0.2s
		addSequential(new CG_OutfeedCube());
		// Drive to front of pyramid while moving elevator to floor and swinging out infeeds
		addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {
				new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.INFEED_HEIGHT),	
				new Auton_RunMotionProfileAction(fromSwitchToFrontOfPyramidPath),	
				new Series_Command(Arrays.asList(new Command[] {
							new WaitCommand(1.7),
							new Infeed_MoveInfeedArmsToPresetPosition(INFEED_ARM_TARGET_POSITION.WIDE)
					}))
        })));
        addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {
            new Auton_RunTimedMotionProfileCommand(toPyramid, 2.0),
            new Series_Command(Arrays.asList(new Command[] {
                    new WaitCommand(1),
                    new Simultaneous_Command(Arrays.asList(new Command[] {
                            new CG_InfeedCube()
                    }))
            }))
    })));
    // Move back from pyramid while continuing to infeed
		addSequential(new Auton_RunMotionProfileAction(fromPyramid));
		// Drive back to switch while storing infeed and raising elevator
		addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {
					new Auton_RunTimedMotionProfileCommand(sTurnToSwitch, 2.7),
					new Infeed_MoveInfeedArmsToPresetPosition(INFEED_ARM_TARGET_POSITION.STORE),
					new Series_Command(Arrays.asList(new Command[] {
							new WaitCommand(elevatorWaitTimeSecondCube),
							new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.SWITCH_HEIGHT),
					}))
        }))); 
        // Move back from pyramid while continuing to infeed
		addSequential(new Auton_RunMotionProfileAction(fromPyramid));
		// Drive back to switch while storing infeed and raising elevator
		addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {
					new Auton_RunTimedMotionProfileCommand(sTurnToSwitch, 2.7),
					new Infeed_MoveInfeedArmsToPresetPosition(INFEED_ARM_TARGET_POSITION.STORE),
					new Series_Command(Arrays.asList(new Command[] {
							new WaitCommand(elevatorWaitTimeSecondCube),
							new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.SWITCH_HEIGHT),
					}))
        }))); 
        addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {
            new WaitCommand(0.2),
            new CG_OutfeedCube()
        })));
        // Move elevator to floor
         addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {
            new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.INFEED_HEIGHT),
            new Auton_RunMotionProfileAction(awayFromSwitch)
        })));
         addSequential(new Simultaneous_Command(Arrays.asList(new Command[] {
            new Auton_RunMotionProfileAction(pyramidAgain),
            new CG_OutfeedCube()
        })));
        addSequential(new PrintTimeFromStart());  
    
        
        
    }

    

    



}