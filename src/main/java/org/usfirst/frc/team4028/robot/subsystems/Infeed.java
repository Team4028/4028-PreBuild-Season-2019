package org.usfirst.frc.team4028.robot.subsystems;

//#region  == Define Imports ==
import org.usfirst.frc.team4028.robot.RobotMap;
import org.usfirst.frc.team4028.robot.util.GeneralUtilities;
import org.usfirst.frc.team4028.robot.util.LogDataBE;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
//#endregion
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class defines the Infeed Subsystem, it is responsible for:
 * 	- Left & Right Swing (Drive) Motors
 * 	- Left & Right Infeed (Drive) Motors
 *  - Left & Right (Home) Limit Switches
 */
public class Infeed extends Subsystem 
{
	public enum INFEED_ARM_TARGET_POSITION {
		HOME,
		INFEED,
		WIDE,
		SQUEEZE,
		CLIMB,
		STORE,
	}
	
	
	// motor controllers
	private TalonSRX _leftSwitchbladeArmMotor; 
	private TalonSRX _rightSwitchbladeArmMotor;
	private TalonSRX _leftInfeedWheelMotor;
	private TalonSRX _rightInfeedWheelMotor;
	
	private boolean _hasLeftArmBeenHomed;
	private boolean _hasRightArmBeenHomed;
	private double _targetLeftInfeedArmPosition;
	private double _targetRightInfeedArmPosition;

	private double _currentInFeedWheelsVBusCmd = .50;
	
	//====================================================================================
	//	Constants for Closed Loop Gains for Infeed Motors
	//====================================================================================
	// PID gains for infeed
	private static final int INFEED_POSITIONS_PID_SLOT_INDEX = 0; //Infeed
	private static final int STORING_ARMS_PID_SLOT_INDEX = 1; //Store
	
	private static final double SWITCHBLADE_INFEED_MOTION_MAGIC_F = 0.3354098361; //Since the two motors have different gear boxes 
	private static final double SWITCHBLADE_INFEED_MOTION_MAGIC_P = 1.5;			//for testing, the F-values are very different
    private static final double SWITCHBLADE_INFEED_MOTION_MAGIC_I = 0;			// |
    private static final double SWITCHBLADE_INFEED_MOTION_MAGIC_D = 0;			// |
    private static final int SWITCHBLADE_INFEED_MOTION_MAGIC_IZONE = 0;
    
    private static final double SWITCHBLADE_STORE_MOTION_MAGIC_F = 0.3354098361; //Since the two motors have different gear boxes 
	private static final double SWITCHBLADE_STORE_MOTION_MAGIC_P = 3.0;			//for testing, the F-values are very different
    private static final double SWITCHBLADE_STORE_MOTION_MAGIC_I = 0;			// |
    private static final double SWITCHBLADE_STORE_MOTION_MAGIC_D = 0;			// |
    private static final int SWITCHBLADE_STORE_MOTION_MAGIC_IZONE = 0;					
            
    private static final int INFEED_ARM_MOTION_MAGIC_MAX_VEL = 3000;
    private static final int INFEED_ARM_MOTION_MAGIC_MAX_ACC = 2000;
    
    private static final int INFEED_ARM_FORWARD_SOFT_LIMIT = 3000;
	
	// Infeed Position Constants [THESE ARE ANGLE MEASURES IN DEGREES]
	private static final double HOME_POSITION_ANGLE = 0; //Is Home
    private static final double INFEED_POSITION_ANGLE = 200; //160;	
    private static final double INFEED_RIGHT_ARM_CLIMB_ANGLE = 219;
	private static final double WIDE_INFEED_POSITION_ANGLE = 140;
	private static final double STORE_POSITION_ANGLE = 30;
	
	private static final double SQUEEZE_INFEED_POSITION_TARGET_ANGLE_BUMP = 1.0;
	
	private static final double INFEED_ALLOWED_ERROR_ANGLE = 5;
	
	// Infeed Drive Wheel Constant
	private static final double INFEED_DRIVE_WHEELS_VBUS_COMMAND_BUMP = 0.05;
	//Infeed Homing Speed
	private static final double INFEED_HOMING_VBUS_COMMAND = 0.15;
	
	//Conversion Constant
	private static final double DEGREES_TO_NATIVE_UNITS_CONVERSION = (4096/360);
	
	private static final boolean IS_VERBOSE_LOGGING_ENABLED = false;
		
	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Infeed _instance = new Infeed();
	
	public static Infeed getInstance() {
		return _instance;
	}

	// private constructor
	private Infeed() 
	{
		//====================================================================================
		//	Begin Setting Up Motors
		//====================================================================================
		
		//Left Arm Rotator Motor
		_leftSwitchbladeArmMotor = new TalonSRX(RobotMap.LEFT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
				
		_leftSwitchbladeArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_leftSwitchbladeArmMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		_leftSwitchbladeArmMotor.setNeutralMode(NeutralMode.Brake);
		
		_leftSwitchbladeArmMotor.configForwardSoftLimitEnable(false, 0);
		_leftSwitchbladeArmMotor.configReverseSoftLimitEnable(false, 0);
		_leftSwitchbladeArmMotor.configForwardSoftLimitThreshold(INFEED_ARM_FORWARD_SOFT_LIMIT, 20);
		
		_leftSwitchbladeArmMotor.setInverted(false);
		
		_leftSwitchbladeArmMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
		
		_leftSwitchbladeArmMotor.config_kF(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_F, 0);
		_leftSwitchbladeArmMotor.config_kP(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_P, 0);
		_leftSwitchbladeArmMotor.config_kI(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_I, 0);
		_leftSwitchbladeArmMotor.config_kD(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_D, 0);
		_leftSwitchbladeArmMotor.config_IntegralZone(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_IZONE, 0);
		
		_leftSwitchbladeArmMotor.config_kF(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_F, 0);
		_leftSwitchbladeArmMotor.config_kP(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_P, 0);
		_leftSwitchbladeArmMotor.config_kI(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_I, 0);
		_leftSwitchbladeArmMotor.config_kD(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_D, 0);
		_leftSwitchbladeArmMotor.config_IntegralZone(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_IZONE, 0);
		
		_leftSwitchbladeArmMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
		_leftSwitchbladeArmMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
		
		//=====================================================================================
		//Right Arm Rotator Motor
		_rightSwitchbladeArmMotor = new TalonSRX(RobotMap.RIGHT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
		
		_rightSwitchbladeArmMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		_rightSwitchbladeArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
		_rightSwitchbladeArmMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		
		_rightSwitchbladeArmMotor.setNeutralMode(NeutralMode.Brake);
		
		_rightSwitchbladeArmMotor.configForwardSoftLimitEnable(false, 0);
		_rightSwitchbladeArmMotor.configReverseSoftLimitEnable(false, 0);
		_rightSwitchbladeArmMotor.configForwardSoftLimitThreshold(INFEED_ARM_FORWARD_SOFT_LIMIT, 20);
		
		_rightSwitchbladeArmMotor.setInverted(true);
		
		_rightSwitchbladeArmMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
		
		_rightSwitchbladeArmMotor.config_kF(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_F, 0);
		_rightSwitchbladeArmMotor.config_kP(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_P, 0);
		_rightSwitchbladeArmMotor.config_kI(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_I, 0);
		_rightSwitchbladeArmMotor.config_kD(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_D, 0);
		_rightSwitchbladeArmMotor.config_IntegralZone(INFEED_POSITIONS_PID_SLOT_INDEX, SWITCHBLADE_INFEED_MOTION_MAGIC_IZONE, 0);
		
		_rightSwitchbladeArmMotor.config_kF(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_F, 0);
		_rightSwitchbladeArmMotor.config_kP(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_P, 0);
		_rightSwitchbladeArmMotor.config_kI(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_I, 0);
		_rightSwitchbladeArmMotor.config_kD(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_D, 0);
		_rightSwitchbladeArmMotor.config_IntegralZone(STORING_ARMS_PID_SLOT_INDEX, SWITCHBLADE_STORE_MOTION_MAGIC_IZONE, 0);
		
		_rightSwitchbladeArmMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
		_rightSwitchbladeArmMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
		
		//=====================================================================================
		//Left Arm Drive Motor
		_leftInfeedWheelMotor = new TalonSRX(RobotMap.LEFT_INFEED_DRIVE_CAN_ADDRESS);
		_leftInfeedWheelMotor.setInverted(true);
			
		//=====================================================================================
		//Right Arm Drive Motor
		_rightInfeedWheelMotor = new TalonSRX(RobotMap.RIGHT_INFEED_DRIVE_CAN_ADDRESS);
		_rightInfeedWheelMotor.setInverted(true);
				
		//=====================================================================================
		_leftSwitchbladeArmMotor.configPeakOutputForward(1, 0);
		_leftSwitchbladeArmMotor.configPeakOutputReverse(-1, 0);
		_rightSwitchbladeArmMotor.configPeakOutputForward(1, 0);
		_rightSwitchbladeArmMotor.configPeakOutputReverse(-1, 0);
		
		_hasLeftArmBeenHomed = false;
		_hasRightArmBeenHomed = false;
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public void initReZeroArms()
	{
		_hasLeftArmBeenHomed = false;
		_hasRightArmBeenHomed = false;
	}
	
	public void zeroArms()
	{		
		// ==== left side ====
		if (_leftSwitchbladeArmMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
			_leftSwitchbladeArmMotor.setSelectedSensorPosition(0, 0, 0); // zero encoder
			_leftSwitchbladeArmMotor.set(ControlMode.PercentOutput, 0); // stop motor
			
			if(!_hasLeftArmBeenHomed) {
				ReportStateChg("Left Arm [AT_HOME]");
			}
			_hasLeftArmBeenHomed = true;
		}
		else if (_hasLeftArmBeenHomed == false) {// only run this side if home not reached, may still be running the other side
			_leftSwitchbladeArmMotor.set(ControlMode.PercentOutput, -1 * INFEED_HOMING_VBUS_COMMAND); // run motor "backwards"
		}
		
		// ==== right side ====
		if (_rightSwitchbladeArmMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
			_rightSwitchbladeArmMotor.setSelectedSensorPosition(0, 0, 0); // zero encoder
			_rightSwitchbladeArmMotor.set(ControlMode.PercentOutput, 0); // stop motor
			
			if(!_hasRightArmBeenHomed) {
				ReportStateChg("Right Arm [AT_HOME]");
			}
			_hasRightArmBeenHomed = true;
		}				
		else if (_hasRightArmBeenHomed == false) { // only run this side if home not reached, may still be running the other side
			_rightSwitchbladeArmMotor.set(ControlMode.PercentOutput, -1 * INFEED_HOMING_VBUS_COMMAND); // run motor "backwards"
		}						
		
		if (get_hasArmsBeenZeroed()) { // only when both arms have been homed
			//ReportStateChg("Infeed Arm (State) [" + _infeedArmState.toString() + "] ==> [AT_HOME]");
			//_infeedArmState = INFEED_ARM_STATE.AT_HOME;
		}
	}
		
	public void MoveToPresetPosition(INFEED_ARM_TARGET_POSITION presetPosition)
	{
		switch(presetPosition) {
			case HOME:
				_targetLeftInfeedArmPosition = HOME_POSITION_ANGLE;
				_targetRightInfeedArmPosition = HOME_POSITION_ANGLE;
				break;
			case INFEED:
				_targetLeftInfeedArmPosition = INFEED_POSITION_ANGLE;
				_targetRightInfeedArmPosition = INFEED_POSITION_ANGLE;
				 break;
			case WIDE:
				_targetLeftInfeedArmPosition = WIDE_INFEED_POSITION_ANGLE;
				_targetRightInfeedArmPosition = WIDE_INFEED_POSITION_ANGLE;
				 break;
			case SQUEEZE:
				_targetLeftInfeedArmPosition = INFEED_POSITION_ANGLE;
				_targetRightInfeedArmPosition = INFEED_POSITION_ANGLE;
				 break;
			case STORE:
				_targetLeftInfeedArmPosition = STORE_POSITION_ANGLE;
				_targetRightInfeedArmPosition = STORE_POSITION_ANGLE;
				 break;
			case CLIMB:
				_targetLeftInfeedArmPosition = STORE_POSITION_ANGLE;
				_targetRightInfeedArmPosition = INFEED_RIGHT_ARM_CLIMB_ANGLE;
				break;
		}
		
		_leftSwitchbladeArmMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
		_leftSwitchbladeArmMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
		
		_rightSwitchbladeArmMotor.configMotionCruiseVelocity(INFEED_ARM_MOTION_MAGIC_MAX_VEL, 0);
		_rightSwitchbladeArmMotor.configMotionAcceleration(INFEED_ARM_MOTION_MAGIC_MAX_ACC, 0);
		
		//set appropriate gain slot in use
		if(_targetLeftInfeedArmPosition == STORE_POSITION_ANGLE) {
			_leftSwitchbladeArmMotor.selectProfileSlot(STORING_ARMS_PID_SLOT_INDEX, 0);
		} else {
			_leftSwitchbladeArmMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
		}
		
		if(_targetRightInfeedArmPosition == STORE_POSITION_ANGLE) {
			_rightSwitchbladeArmMotor.selectProfileSlot(STORING_ARMS_PID_SLOT_INDEX, 0);
		} else {
			_rightSwitchbladeArmMotor.selectProfileSlot(INFEED_POSITIONS_PID_SLOT_INDEX, 0);
		}
					
		_leftSwitchbladeArmMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(_targetLeftInfeedArmPosition));
		_rightSwitchbladeArmMotor.set(ControlMode.MotionMagic, degreesToNativeUnits(_targetRightInfeedArmPosition));
	}

	//=====================================================================================
	// Methods for Driving Infeed Wheels
	//=====================================================================================
	public void feedIn() {
		_leftInfeedWheelMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
		_rightInfeedWheelMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
	}
	
	public void feedOut() {
		_leftInfeedWheelMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
		_rightInfeedWheelMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
	}   
	
	public void infeedWheels_SpinCube_CCW() {
		_leftInfeedWheelMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
		_rightInfeedWheelMotor.set(ControlMode.PercentOutput, -1.0 * _currentInFeedWheelsVBusCmd);
	}
	
	public void infeedWheels_SpinCube_CW() {
		_leftInfeedWheelMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
		_rightInfeedWheelMotor.set(ControlMode.PercentOutput, _currentInFeedWheelsVBusCmd);
	}
	
	public void infeedWheels_SpinCube_Auton() {
	}
	
	public void infeedWheels_VBusCmd_BumpUp() {
		double newCmd = _currentInFeedWheelsVBusCmd + INFEED_DRIVE_WHEELS_VBUS_COMMAND_BUMP;
		
		// only bump if new cmd is not over max
		if(newCmd <= 1.0) {
			_currentInFeedWheelsVBusCmd = newCmd;
		}
	}
	
	public void infeedWheels_VBusCmd_BumpDown() {		
		double newCmd = _currentInFeedWheelsVBusCmd - INFEED_DRIVE_WHEELS_VBUS_COMMAND_BUMP;
		
		// only bump if new cmd is not under min
		if(newCmd >= 0.0) {
			_currentInFeedWheelsVBusCmd = newCmd;
		}
	}

	public void stopInfeedWheels(){
		_leftInfeedWheelMotor.set(ControlMode.PercentOutput, 0);
		_rightInfeedWheelMotor.set(ControlMode.PercentOutput, 0);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    //=====================================================================================
	//Method for determining if Arms are In Position
	//=====================================================================================	
	public boolean get_areArmsInPosition() {
		double currentErrorL = Math.abs(nativeUnitsToDegrees(get_currentLeftInfeedPosition()) - _targetLeftInfeedArmPosition);
		double currentErrorR = Math.abs(nativeUnitsToDegrees(get_currentRightInfeedPosition()) - _targetRightInfeedArmPosition);
		
		if(currentErrorL < INFEED_ALLOWED_ERROR_ANGLE && currentErrorR < INFEED_ALLOWED_ERROR_ANGLE
				&& _targetLeftInfeedArmPosition != HOME_POSITION_ANGLE
				&& _targetLeftInfeedArmPosition != STORE_POSITION_ANGLE
				&& _targetRightInfeedArmPosition != HOME_POSITION_ANGLE
				&& _targetRightInfeedArmPosition != STORE_POSITION_ANGLE) {
			return true;
		} 
		//else if(_targetLeftInfeedArmPosition == _currentInFeedArmSqueezeTargetAngle 
		//		&& _targetRightInfeedArmPosition == _currentInFeedArmSqueezeTargetAngle) { 
		//	return true;
		else {
			return false;
		}
	}
	
	//=====================================================================================
	//Methods for Exposing Properties of Infeed Motors
	//=====================================================================================
	public boolean get_hasArmsBeenZeroed()
	{
		return get_hasLeftArmBeenHomed() && get_hasRightArmBeenHomed();
	}

	public double get_currentLeftInfeedPosition() {
		return _leftSwitchbladeArmMotor.getSelectedSensorPosition(0);
	}
	
	public double get_currentRightInfeedPosition() {
		return _rightSwitchbladeArmMotor.getSelectedSensorPosition(0);
	}

	public boolean get_isSafeToRunInfeedWheels(){
		if((_targetLeftInfeedArmPosition == STORE_POSITION_ANGLE
			|| _targetLeftInfeedArmPosition == HOME_POSITION_ANGLE)
			&& (_targetRightInfeedArmPosition == STORE_POSITION_ANGLE
			|| _targetRightInfeedArmPosition == HOME_POSITION_ANGLE)){
			return false;
		} else {
			return true;
		}
	}
	
	private double get_currentInFeedWheelsVBusCmd()
	{
		return _currentInFeedWheelsVBusCmd;
	}

	private boolean get_hasLeftArmBeenHomed()
	{
		return _hasLeftArmBeenHomed;
	}

	private boolean get_hasRightArmBeenHomed()
	{
		return _hasRightArmBeenHomed;
	}

	public boolean get_areArmsInSafePosition() {
		if (get_currentLeftInfeedPosition() <= (degreesToNativeUnits(WIDE_INFEED_POSITION_ANGLE) + 100) 
				&& get_currentLeftInfeedPosition() <= (degreesToNativeUnits(WIDE_INFEED_POSITION_ANGLE) + 100)) {
			return true;
		} else {
			return false;
		}
	}

	//=====================================================================================
	public void updateLogData(LogDataBE logData) 
	{
		logData.AddData("Infeed: Left Target Arm Position", String.valueOf(nativeUnitsToDegrees(_targetLeftInfeedArmPosition)));
		logData.AddData("Infeed: Right Target Arm Position", String.valueOf(nativeUnitsToDegrees(_targetRightInfeedArmPosition)));
		logData.AddData("Infeed: L Position", String.valueOf(nativeUnitsToDegrees(get_currentLeftInfeedPosition())));
		logData.AddData("Infeed: R Position:", String.valueOf(nativeUnitsToDegrees(get_currentRightInfeedPosition())));
		logData.AddData("Infeed: L/R Arms Homed?", String.valueOf(get_hasLeftArmBeenHomed()) + " / " + String.valueOf(get_hasRightArmBeenHomed()));
		logData.AddData("State: Infeed", "N/A for 2019");
	}
    
	public void updateDashboard() 
	{
		SmartDashboard.putString("InfeedWheels:State", "N/A for 2019");
		SmartDashboard.putNumber("InfeedWheels:%VBus", get_currentInFeedWheelsVBusCmd());
		
		SmartDashboard.putNumber("InfeedArms:Target Wide Angle:", degreesToNativeUnits(WIDE_INFEED_POSITION_ANGLE));
		SmartDashboard.putNumber("InfeedArms:Target Squeeze Angle", -1.0); //_currentInFeedArmSqueezeTargetAngle);
		
		SmartDashboard.putBoolean("InfeedArms:Left Homed?", get_hasLeftArmBeenHomed());
		SmartDashboard.putBoolean("InfeedArms:Right Homed?", get_hasRightArmBeenHomed());
		SmartDashboard.putBoolean("InfeedArms:Are Safe?", get_areArmsInSafePosition());		
		SmartDashboard.putBoolean("InfeedArms:InPosition?", get_areArmsInPosition());
		SmartDashboard.putString("InfeedArms:State", "N/A for 2019");
		
		SmartDashboard.putNumber("InfeedArms:Left Current PositionNU", get_currentLeftInfeedPosition());
		SmartDashboard.putNumber("InfeedArms:Right Current PositionNU:", get_currentRightInfeedPosition());
		
		SmartDashboard.putNumber("InfeedArms:Left Current Angle", GeneralUtilities.roundDouble(nativeUnitsToDegrees(get_currentLeftInfeedPosition()), 1));
		SmartDashboard.putNumber("InfeedArms:Right Current Angle:", GeneralUtilities.roundDouble(nativeUnitsToDegrees(get_currentRightInfeedPosition()), 1));
	}
	
	//=====================================================================================
	//Methods for Conversions between Native Units and Degrees
	//=====================================================================================
	private double degreesToNativeUnits(double degreeMeasure) {
		double nativeUnits = degreeMeasure * DEGREES_TO_NATIVE_UNITS_CONVERSION;
		return nativeUnits;
	}
	
	private double nativeUnitsToDegrees(double nativeUnitsMeasure) {
		double degrees = nativeUnitsMeasure / DEGREES_TO_NATIVE_UNITS_CONVERSION;
		return degrees;
	}
	
	// private helper method to control how we write to the drivers station
	private void ReportStateChg(String message) {
		if(IS_VERBOSE_LOGGING_ENABLED) {
			System.out.println(message);
		}
	}
}

