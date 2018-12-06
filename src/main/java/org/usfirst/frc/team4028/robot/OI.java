package org.usfirst.frc.team4028.robot;

//#region Define Imports

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

		// =========== Operator ======================================
		OperatorController = new BeakXboxController(RobotMap.OPERATOR_GAMEPAD_USB_PORT);
		//==========================================================
		System.out.println("Creating Gamepad");
		// Operator Controller -> Command Mapping
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
