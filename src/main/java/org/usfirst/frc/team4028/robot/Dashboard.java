package org.usfirst.frc.team4028.robot;

// #region

import org.usfirst.frc.team4028.robot.commands.auton.Auton_CG_AutoRun;
import org.usfirst.frc.team4028.robot.commands.auton.Auton_CG_AutoTurn;
import org.usfirst.frc.team4028.robot.commands.auton.Auton_CG_ChassisTune;
import org.usfirst.frc.team4028.robot.commands.auton.Auton_CG_GearTest;
import org.usfirst.frc.team4028.robot.commands.auton.Auton_CG_TestPath;
import org.usfirst.frc.team4028.robot.commands.auton.Auton_DoNothing;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// #endregion

/**
 *  This class contains code to interface with the Dashboard on the Driver's Station
 *	We read values from 
 *		- Sendable Choosers to control Auton
 *		- FMS Data for real time game data
 *  We write values to
 *		- provide real-time info to the drive team
 */
public class Dashboard 
{
	private enum AUTON_MODE 
	{
		UNDEFINED,
		DO_NOTHING,
		AUTO_RUN,
		AUTO_TURN,
		EXPERIMENTAL,
		AUTO_TUNE,
		TEST_AUTON,
		GEAR_TEST
	}
	private enum STARTING_SIDE 
	{
		LEFT,
		RIGHT
	}
	
	private SendableChooser<AUTON_MODE> _autonModeChooser = new SendableChooser<>();
	private SendableChooser<STARTING_SIDE> _autonStartingSideChooser = new SendableChooser<>();
	
	private boolean _isSwitchLeft, _isScaleLeft, _isStartingLeft = true;
	
	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Dashboard _instance = new Dashboard();
	
	public static Dashboard getInstance() 
	{
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Dashboard() 
	{
		_autonModeChooser.addDefault("Do Nothing", AUTON_MODE.DO_NOTHING);
		_autonModeChooser.addObject("Auto Run", AUTON_MODE.AUTO_RUN);
		_autonModeChooser.addObject("Auto Turn", AUTON_MODE.AUTO_TURN);
		_autonModeChooser.addObject("DO NOT SELECT", AUTON_MODE.TEST_AUTON);
		_autonModeChooser.addObject("Tune Chassis", AUTON_MODE.AUTO_TUNE);
		SmartDashboard.putData("AUTON MODE: ", _autonModeChooser);
		_autonModeChooser.addObject("Test", AUTON_MODE.TEST_AUTON);
		_autonModeChooser.addObject("GearTest", AUTON_MODE.GEAR_TEST);
		
		_autonStartingSideChooser.addDefault("LEFT", STARTING_SIDE.LEFT);
		_autonStartingSideChooser.addObject("RIGHT", STARTING_SIDE.RIGHT);
		SmartDashboard.putData("AUTON STARTING SIDE: ", _autonStartingSideChooser);
	}
	
	public boolean isGameDataReceived() 
	{
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		//String gameData = "LLL";
		
		if (gameData.length() > 0) 
		{
			_isSwitchLeft = (gameData.charAt(0) == 'L');
			_isScaleLeft = (gameData.charAt(1) == 'L');
			DriverStation.reportWarning("GAMEDATA: "+ gameData, false);
			return true;
		} 
		else 
		{
			return false;
		}
	}
	
	public boolean isBlueAlliance() 
	{
		return DriverStation.getInstance().getAlliance() == Alliance.Blue;
	}
	
	/** This prints once during robotInit */
	public void printStartupMessage() 
	{
		boolean isFMSAttached = DriverStation.getInstance().isFMSAttached();
		
		DriverStation.reportWarning(">>>>> Is FMS Attached : [" + isFMSAttached + "] <<<<<<", false);
	}
	
	/** Returns the autonBase object associated with the auton selected on the dashboard */
	public CommandGroup getSelectedAuton() 
	{
		//return new Auton_CG_BaseLine();
		_isStartingLeft = (_autonStartingSideChooser.getSelected() == STARTING_SIDE.LEFT);
		
		switch(_autonModeChooser.getSelected()) 
		{
			case DO_NOTHING:
				return new Auton_DoNothing();
			case AUTO_TUNE:
				return new Auton_CG_ChassisTune();
			case AUTO_RUN:
				return new Auton_CG_AutoRun();
			case TEST_AUTON:
				return new Auton_CG_TestPath();
			case AUTO_TURN:
				return new Auton_CG_AutoTurn();
			case GEAR_TEST:
				return new Auton_CG_GearTest();
			default:
				return new Auton_DoNothing(); 
		}
	}
	
	public void outputToDashboard() 
	{
		SmartDashboard.putString("AUTON SELECTED", _autonModeChooser.getSelected().toString());
		SmartDashboard.putNumber("Angle", Chassis.getInstance().get_Heading());		
		//SmartDashboard.putString("FMS Debug Msg", _fmsDebugMsg);
	}
}