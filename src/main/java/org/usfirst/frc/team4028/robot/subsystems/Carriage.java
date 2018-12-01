package org.usfirst.frc.team4028.robot.subsystems;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.RobotMap;
import org.usfirst.frc.team4028.robot.util.LogDataBE;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//#endregion

/**
 * This class defines the Carriage Subsystem, it is responsible for:
 * 	- Left & Right Motors that drive the infeed/outfeed wheels
 *  - Solenoid that controls the squeeze
 *  - Solenoid that controls the tilt
 *  - Limit Switch that indicates a cube is fully in the carriage
 */
public class Carriage extends Subsystem 
{
	// define class level working variables
	private TalonSRX _carriageLeftMotor; 
	private TalonSRX _carriageRightMotor;
	
	private DigitalInput _carriageLimitSwitch;
	private DoubleSolenoid _squeezeCylinder;
	private DoubleSolenoid _tiltCylinder;

	private double _currentCarriageWheelsFeedInVBusCmd = .45;
	
	private CARRIAGE_WHEELS_OUT_VBUS_INDEX _currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50;

	// for limit switch debouncing
	private long _consecutiveScansCubeIsPresent = 0;
	private static final int MIN_CONSECUTIVE_SCANS = 5;

	private enum CARRIAGE_WHEELS_STATE {
		STOPPED,
		FEED_IN,
		FEED_OUT,
	}

	public enum CARRIAGE_WHEELS_OUT_VBUS_INDEX {
		VBUS_10,
		VBUS_20,
		VBUS_30,
		VBUS_35,
		VBUS_40,
		VBUS_50,
		VBUS_60,
		VBUS_70,
		VBUS_80,
		VBUS_90,
		VBUS_100
	}

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Carriage _instance = new Carriage();
	
	public static Carriage getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Carriage() {
		//====================================================================================
		//	config master & slave talon objects
		//====================================================================================
		_carriageLeftMotor = new TalonSRX(RobotMap.CARRIAGE_LEFT_CAN_ADDRESS);
		_carriageRightMotor = new TalonSRX(RobotMap.CARRIAGE_RIGHT_CAN_ADDRESS);
		
		// set motor phasing
		_carriageLeftMotor.setInverted(false);
		_carriageRightMotor.setInverted(true);
		
		// config limit switches
		_carriageLeftMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_carriageLeftMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		_carriageRightMotor.configForwardLimitSwitchSource(RemoteLimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0, 0);
		_carriageRightMotor.configReverseLimitSwitchSource(RemoteLimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0, 0);
		
		// turn off all soft limits
		_carriageLeftMotor.configForwardSoftLimitEnable(false, 0);
		_carriageLeftMotor.configReverseSoftLimitEnable(false, 0);
		_carriageRightMotor.configForwardSoftLimitEnable(false, 0);
		_carriageRightMotor.configReverseSoftLimitEnable(false, 0);
		
		// config brake mode
		_carriageLeftMotor.setNeutralMode(NeutralMode.Coast);
		_carriageRightMotor.setNeutralMode(NeutralMode.Coast);
		
		//Enable Current Limiting
		_carriageLeftMotor.enableCurrentLimit(true);
		_carriageLeftMotor.configPeakCurrentDuration(200, 0);
		_carriageLeftMotor.configPeakCurrentLimit(17, 0);
		
		// config quad encoder & phase (invert = true)
		_carriageLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.None, 0, 0);
		
		//configure the peak and nominal output voltages in both directions for both Talons
		_carriageLeftMotor.configNominalOutputForward(0, 0);
		_carriageLeftMotor.configNominalOutputReverse(0, 0);
		_carriageLeftMotor.configPeakOutputForward(1, 0);
		_carriageLeftMotor.configPeakOutputReverse(-1, 0);
		
		_carriageRightMotor.configNominalOutputForward(0, 0);
		_carriageRightMotor.configNominalOutputReverse(0, 0);
		_carriageRightMotor.configPeakOutputForward(1, 0);
		_carriageRightMotor.configPeakOutputReverse(-1, 0);
		
		// set motor mode
		_carriageLeftMotor.set(ControlMode.PercentOutput, 0, 0);
		_carriageRightMotor.set(ControlMode.PercentOutput, 0, 0);
	
		// DisableSoftLimits
		_carriageLeftMotor.configReverseSoftLimitEnable(false, 0);
		_carriageLeftMotor.configForwardSoftLimitEnable(false, 0);
		_carriageRightMotor.configReverseSoftLimitEnable(false, 0);
		_carriageRightMotor.configForwardSoftLimitEnable(false, 0);
		
		//Setup Limit Switch
		_carriageLimitSwitch = new DigitalInput(RobotMap.CARRIAGE_LIMIT_SWITCH_DIO_PORT);
		
		//Setup Solenoid for Cylinder
		_squeezeCylinder = new DoubleSolenoid(RobotMap.PCM_CAN_ADDR, RobotMap.CARRIAGE_SQUEEZE_PCM_PORT, RobotMap.CARRIAGE_WIDE_PCM_PORT);
		_squeezeCylinder.set(Constants.CARRIAGE_WIDE_POS);
		
		//Setup Solenoid for Tilt
		_tiltCylinder = new DoubleSolenoid(RobotMap.PCM_CAN_ADDR, RobotMap.CARRIAGE_FLAP_UP_PCM_PORT, RobotMap.CARRIAGE_FLAP_DOWN_PCM_PORT);
		
		this.tiltCarriageDown();
	}
	
    // Put methods for controlling this subsystem hereb (Call these from Commands)

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
	
	//#region Feed Out VBUS Bumping
	//=====================================================================================
	// Handle Bump Feed Out Up/Down Command
	//=====================================================================================
	public void carriageWheels_FeedOut_VBusCmd_BumpUp() {
		switch (_currentCarriageWheelsFeedOutVBusIndex)	{
			case VBUS_100:
				// do nothing
				break;
	
			case VBUS_90:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_100;
				break;
				
			case VBUS_80:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_90;
				break;
	
			case VBUS_70:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_80;
				break;
				
			case VBUS_60:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_70;
				break;
	
			case VBUS_50:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_60;
				break;
				
			case VBUS_40:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50;
				break;
				
			
			case VBUS_30:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_40;
				break;
	
			case VBUS_20:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_30;
				break;
				
			case VBUS_10:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_20;
				break;
				
			default:
				break;
		}
	}
	
	public void carriageWheels_FeedOut_VBusCmd_BumpDown() {
		switch (_currentCarriageWheelsFeedOutVBusIndex)	{
			case VBUS_100:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_90;
				break;

			case VBUS_90:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_80;
				break;
				
			case VBUS_80:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_70;
				break;

			case VBUS_70:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_60;
				break;
				
			case VBUS_60:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_50;
				break;

			case VBUS_50:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_40;
				break;
				
			case VBUS_40:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_30;
				break;

			case VBUS_30:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_20;
				break;

			case VBUS_20:
				_currentCarriageWheelsFeedOutVBusIndex = CARRIAGE_WHEELS_OUT_VBUS_INDEX.VBUS_10;
				break;
				
			case VBUS_10:
				// do nothing
				break;
				
			default:
				break;
		}
	}
	//#endregion

	//=====================================================================================
	// Public Methods to infeed motors
	//=====================================================================================
	public void stop() 
	{
		_carriageLeftMotor.set(ControlMode.PercentOutput, 0, 0);
		_carriageRightMotor.set(ControlMode.PercentOutput, 0, 0);
	}

	public void feedIn() 
	{
		// stop infeeding when cube is in the carriage
		if(!get_isCubeInCarriage()) 
		{
			_carriageLeftMotor.set(ControlMode.PercentOutput, get_currentCarriageWheelsFeedInVBusCmd(), 0);
			_carriageRightMotor.set(ControlMode.PercentOutput, get_currentCarriageWheelsFeedInVBusCmd(), 0);		
		} 
		else 
		{
			stop();
		}
	}

	public void feedOut() 
	{
		_carriageLeftMotor.set(ControlMode.PercentOutput, -1.0 * get_currentCarriageWheelsFeedOutVBusCmd(), 0);
		_carriageRightMotor.set(ControlMode.PercentOutput, -1.0 * get_currentCarriageWheelsFeedOutVBusCmd(), 0);
	}

	public void feedOut(CARRIAGE_WHEELS_OUT_VBUS_INDEX setSpeed) 
	{
		_currentCarriageWheelsFeedOutVBusIndex = setSpeed;
		feedOut();
	}

	//=====================================================================================
	// Handle Carriage Squeeze Solenoid 
	//=====================================================================================
	public void moveCarriageToSqueezeWidth() {
		if(get_isCarriageInSqueezePosition()) {
			System.out.println("Carriage Already In Thin Position");
		} else {
			_squeezeCylinder.set(Constants.CARRIAGE_SQUEEZE_POS);
		}
	}
	
	public void moveCarriageToWideWidth() {
		if(get_isCarriageInSqueezePosition()) {
			_squeezeCylinder.set(Constants.CARRIAGE_WIDE_POS);
		} else {
			System.out.println("Carriage Already In Wide Position");
		}	
	}

	//=====================================================================================
	// Handle Carriage Tilt Solenoid 
	//=====================================================================================
	public void tiltCarriageUp() {
		_tiltCylinder.set(Constants.CARRIAGE_FLAP_UP);
	}
	
	public void tiltCarriageDown() {
		_tiltCylinder.set(Constants.CARRIAGE_FLAP_DOWN);	
	}

	//=====================================================================================
	// Property Accessors
	//=====================================================================================
	public boolean get_isCubeInCarriage() 
	{
		// debounce limit switch
		boolean isCubePresentThisScan = _carriageLimitSwitch.get(); // normally closed switch, input is pulled low
		if(isCubePresentThisScan)
		{
			_consecutiveScansCubeIsPresent++;		
			return (_consecutiveScansCubeIsPresent >= MIN_CONSECUTIVE_SCANS);
		}
		else
		{
			_consecutiveScansCubeIsPresent = 0;
			return false;
		}
	} 

	public boolean get_isCarriageInSqueezePosition()
	{
		return _squeezeCylinder.get() == Constants.CARRIAGE_SQUEEZE_POS;
	}

	public boolean get_isFlapInUpPosition()
	{
		return _tiltCylinder.get() == Constants.CARRIAGE_FLAP_UP;
	}

	private CARRIAGE_WHEELS_STATE get_carriageWheelsState() 
	{
		return CARRIAGE_WHEELS_STATE.STOPPED;
	}

	private double get_currentCarriageWheelsFeedInVBusCmd()
	{
		return _currentCarriageWheelsFeedInVBusCmd;
	}

	private double get_currentCarriageWheelsFeedOutVBusCmd(){
		switch (_currentCarriageWheelsFeedOutVBusIndex)	
		{
			case VBUS_100:
				return 1;
			
			case VBUS_90:
				return .9;
				
			case VBUS_80:
				return .8;
				
			case VBUS_70:
				return .7;
				
			case VBUS_60:
				return .6;
				
			case VBUS_50:
				return .5;
				
			case VBUS_40:
				return .4;
			
			case VBUS_35:
				return .35;

			case VBUS_30:
				return .3;
				
			case VBUS_20:
				return .2;
				
			case VBUS_10:
				return .1;
				
			default:
				return 0;
		}
	}

	public void updateLogData(LogDataBE logData) 
	{
		logData.AddData("Carriage: LimitSwitch", String.valueOf(get_isCubeInCarriage()));
		logData.AddData("Carriage: VBus Cmd", String.valueOf(get_currentCarriageWheelsFeedOutVBusCmd()));
		logData.AddData("State: Carriage", get_carriageWheelsState().toString());
	}
	
	public void updateDashboard() 
	{
		SmartDashboard.putNumber("Carriage: Wheels Feed In %VBus", get_currentCarriageWheelsFeedInVBusCmd());
		SmartDashboard.putNumber("Carriage: Wheels Feed Out %VBus", get_currentCarriageWheelsFeedOutVBusCmd());
		SmartDashboard.putBoolean("Carriage: Is Cube In Carriage?", get_isCubeInCarriage());
		SmartDashboard.putBoolean("Carriage: Is Squeezed", get_isCarriageInSqueezePosition());
		SmartDashboard.putBoolean("Carriage: Is Flapped Up", get_isFlapInUpPosition());
		SmartDashboard.putString("State: Carriage", get_carriageWheelsState().toString());
	}
}