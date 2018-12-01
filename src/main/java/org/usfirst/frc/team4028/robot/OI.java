package org.usfirst.frc.team4028.robot;

//#region Define Imports
import org.usfirst.frc.team4028.robot.commands.Elevator_MoveElevatorToPresetPosition;
import org.usfirst.frc.team4028.robot.commands.Infeed_MoveInfeedArmsToPresetPosition;
import org.usfirst.frc.team4028.robot.commands.Infeed_RunInfeedWheels;
import org.usfirst.frc.team4028.robot.commands.Infeed_ZeroInfeedArms;
import org.usfirst.frc.team4028.robot.commands.Infeed_RunInfeedWheels.INFEED_WHEELS_FUNCTION;
import org.usfirst.frc.team4028.robot.commands.Carriage_ToggleSqueezeSolenoid.CARRIAGE_SQUEEZE_FUNCTIONS;
import org.usfirst.frc.team4028.robot.commands.Elevator_BumpElevatorPosition.ELEVATOR_BUMP_FUNCTION;
import org.usfirst.frc.team4028.robot.commands.Chassis_ShiftGear;
import org.usfirst.frc.team4028.robot.commands.Elevator_BumpElevatorPosition;
import org.usfirst.frc.team4028.robot.commands.Chassis_DriveWithControllers;
import org.usfirst.frc.team4028.robot.commands.ToggleActiveCamera;
import org.usfirst.frc.team4028.robot.commands.Carriage_BumpCarriageVBus.CARRIAGE_BUMP_FUNCTION;
import org.usfirst.frc.team4028.robot.commands.Carriage_ToggleFlapSolenoid;
import org.usfirst.frc.team4028.robot.commands.Carriage_ToggleSqueezeSolenoid;
import org.usfirst.frc.team4028.robot.commands.CG_ClimbPosition;
import org.usfirst.frc.team4028.robot.commands.CG_InfeedCube;
import org.usfirst.frc.team4028.robot.commands.CG_OutfeedCube;
import org.usfirst.frc.team4028.robot.commands.CG_StopInfeeding;
import org.usfirst.frc.team4028.robot.commands.Carriage_BumpCarriageVBus;
import org.usfirst.frc.team4028.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;
import org.usfirst.frc.team4028.robot.subsystems.Infeed.INFEED_ARM_TARGET_POSITION;
import org.usfirst.frc.team4028.robot.util.BeakXboxController;

import edu.wpi.first.wpilibj.GenericHID.Hand;
//#endregion

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//#region CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	//#endregion

	private BeakXboxController DriverController;
	private BeakXboxController OperatorController;
		
	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static OI _instance = new OI();
	
	public static OI getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private OI() 
	{	
		// =========== Driver ======================================
		DriverController = new BeakXboxController(RobotMap.DRIVER_GAMEPAD_USB_PORT);
		//==========================================================
		
		// Driver Controller -> Command Mapping
		DriverController.a.whenPressed(new Infeed_MoveInfeedArmsToPresetPosition(INFEED_ARM_TARGET_POSITION.SQUEEZE));
		DriverController.b.whenPressed(new Infeed_MoveInfeedArmsToPresetPosition(INFEED_ARM_TARGET_POSITION.WIDE));
		DriverController.x.whenPressed(new Infeed_MoveInfeedArmsToPresetPosition(INFEED_ARM_TARGET_POSITION.STORE));
		DriverController.y.whenPressed(new Infeed_ZeroInfeedArms());
		DriverController.lb.whenPressed(new Infeed_RunInfeedWheels(INFEED_WHEELS_FUNCTION.SpinCW));
		DriverController.rb.whenPressed(new Infeed_RunInfeedWheels(INFEED_WHEELS_FUNCTION.SpinCCW));
		DriverController.lt.whenPressed(new CG_InfeedCube());
		DriverController.rt.whenPressed(new CG_OutfeedCube());
		DriverController.start.whenPressed(new Chassis_ShiftGear());
		
		DriverController.lb.whenReleased(new Infeed_RunInfeedWheels(INFEED_WHEELS_FUNCTION.StopWheels));
		DriverController.rb.whenReleased(new Infeed_RunInfeedWheels(INFEED_WHEELS_FUNCTION.StopWheels));
		DriverController.lt.whenReleased(new CG_StopInfeeding());
		DriverController.rt.whenReleased(new CG_StopInfeeding());
		
		DriverController.dPad.up.whenPressed(new Carriage_BumpCarriageVBus(CARRIAGE_BUMP_FUNCTION.BumpUp));
		DriverController.dPad.down.whenPressed(new Carriage_BumpCarriageVBus(CARRIAGE_BUMP_FUNCTION.BumpDown));

		DriverController.leftStick.whileActive(new Chassis_DriveWithControllers(DriverController.leftStick, DriverController.rightStick));
		DriverController.rightStick.whileActive(new Chassis_DriveWithControllers(DriverController.leftStick, DriverController.rightStick));
 
		DriverController.leftStick.whenReleased(new Chassis_DriveWithControllers(DriverController.leftStick, DriverController.rightStick));
		DriverController.rightStick.whenReleased(new Chassis_DriveWithControllers(DriverController.leftStick, DriverController.rightStick));

		// =========== Operator ======================================
		OperatorController = new BeakXboxController(RobotMap.OPERATOR_GAMEPAD_USB_PORT);
		//==========================================================
		System.out.println("Creating Gamepad");
		// Operator Controller -> Command Mapping
		OperatorController.a.whenPressed(new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.INFEED_HEIGHT));
		OperatorController.y.whenPressed(new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.SCALE_HEIGHT));
		OperatorController.b.whenPressed(new Elevator_BumpElevatorPosition(ELEVATOR_BUMP_FUNCTION.BumpUp));
		OperatorController.x.whenPressed(new Elevator_BumpElevatorPosition(ELEVATOR_BUMP_FUNCTION.BumpDown));
		OperatorController.back.whenPressed(new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.CLIMB_HEIGHT));
		OperatorController.rb.whenPressed(new Elevator_MoveElevatorToPresetPosition(ELEVATOR_TARGET_POSITION.SWITCH_HEIGHT));
		OperatorController.lb.whenPressed(new CG_ClimbPosition());
		OperatorController.lt.whenPressed(new Carriage_ToggleSqueezeSolenoid(CARRIAGE_SQUEEZE_FUNCTIONS.Squeeze));
		OperatorController.rt.whenPressed(new Carriage_ToggleSqueezeSolenoid(CARRIAGE_SQUEEZE_FUNCTIONS.Wide));
		OperatorController.start.whenPressed(new ToggleActiveCamera());

		OperatorController.leftStick.whenActive(new Carriage_ToggleFlapSolenoid(OperatorController.leftStick));
	}
		
	public double getOperator_Climber_JoystickCmd() {
		if(Math.abs(OperatorController.getY(Hand.kRight)) >= 0.5){
			// flip the sign, pushing the joystick up is a # < 0
			return OperatorController.getY(Hand.kRight) * -1.0;
		} 
		else {
			return 0.0;
		}
	}
}
