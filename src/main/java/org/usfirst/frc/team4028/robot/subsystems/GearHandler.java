package org.usfirst.frc.team4028.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.Constants;
import org.usfirst.frc.team4028.robot.RobotMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearHandler extends Subsystem {
	public static GearHandler _instance = new GearHandler();
	
	public static GearHandler getInstance() {
		return _instance;
	}
	
	// define class level public constants
	public static final double INFEED_TARGET_CMD = -0.5;
	public static final double OUTFEED_TARGET_CMD = 0.5;
	
	// define class level variables for Robot objects
	private TalonSRX _gearTiltMotor;
	private TalonSRX _gearInfeedMotor;
	
	// define class level private working variables
	private double _targetPositionRotations;
	private long _flapAnnReentrantRunningMsec;
	
	// --------------------------------------------------------
	// define Tilt Motor PID constants
	private static final int 	TILT_PID_P_PROFILE = 0;
	private static final double TILT_PID_P_CONSTANT = 1.6;
	private static final double TILT_PID_I_CONSTANT = 0.0;
	private static final double TILT_PID_D_CONSTANT = 50.0;
	private static final double TILT_PID_RAMP_RATE = 0.1;
		
	private static final double TILT_MAX_V_DOWN_TILT = +3.0; // Down is positive (RIP MAXIMUM...)
	private static final double TILT_MAX_V_UP_TILT = -6.0;
	// --------------------------------------------------------
	
	// --------------------------------------------------------
	// define Working variables and constants for homing the tilt axix
	private enum GEAR_TILT_HOMING_STATE {
		UNDEFINED,
		MOVING_TO_HOME,
		AT_HOME,
		TIMEOUT,
		ZEROED
	}
	
	private enum GEAR_TILT_MOVE_LAST_TARGET_POSITION {
		UNDEFINED,
		MOVING_TO_SCORING_POSITION,
		MOVING_TO_HOME,
		MOVING_TO_FLOOR
	}
	
	private static final double GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS = 00.00;
	private static final double GEAR_TILT_SCORING_POSITION_IN_ROTATIONS = 0.1;
	private static final double GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS = 00.48;
	private static final double TARGET_DEADBAND = 00.03;
	
	private static final double GEAR_MOVE_TO_HOME_VELOCITY_CMD = -0.40;   //set
	private static final long GEAR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC = 5000;
	private String _gearTiltState;
	
	private long _gearTiltAxisStateStartTime;
	private GEAR_TILT_HOMING_STATE _gearTiltAxisZeroCurrentState;
	private GEAR_TILT_MOVE_LAST_TARGET_POSITION _gearTiltMoveLastTargetPosition;
	private boolean _isLastTiltMoveToFloorCallComplete;

	// --------------------------------------------------------
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	private GearHandler() 
	{
		// Tilt Motor
		_gearTiltMotor = new TalonSRX(RobotMap.GEAR_TILT_CAN_ADDR);
		_gearTiltMotor.set(ControlMode.PercentOutput,Constants.PERCENT_OUTPUT);	// open loop throttle
		_gearTiltMotor.setNeutralMode(NeutralMode.Coast);							// default to brake mode DISABLED
		_gearTiltMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,10);	// set encoder to be feedback device
		_gearTiltMotor.setSensorPhase(false);  							// do not invert encoder feedback
		_gearTiltMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed,10);
		_gearTiltMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
		//_gearTiltMotor.ConfigRevLimitSwitchNormallyOpen(false);
		
		//_gearTiltMotor.setProfile(TILT_PID_P_PROFILE);
		_gearTiltMotor.set(ControlMode.MotionMagic,TILT_PID_P_PROFILE);
		_gearTiltMotor.set(ControlMode.MotionMagic, TILT_PID_I_CONSTANT, TILT_PID_D_CONSTANT);
		_gearTiltMotor.configNominalOutputForward(0.0f, 10);
		_gearTiltMotor.configNominalOutputReverse(-0.0f, 10);
		//_gearTiltMotor.configPeakOutputVoltage(TILT_MAX_V_DOWN_TILT, TILT_MAX_V_UP_TILT);
		_gearTiltMotor.configPeakOutputForward(TILT_MAX_V_DOWN_TILT, 10);
		_gearTiltMotor.configPeakOutputReverse(TILT_MAX_V_UP_TILT, 10);

		
		// Infeed Motor
		_gearInfeedMotor = new TalonSRX(RobotMap.GEAR_INFEED_CAN_ADDR);
		_gearInfeedMotor.set(ControlMode.PercentOutput, Constants.PERCENT_OUTPUT);	// open loop throttle
		//_gearInfeedMotor.enableBrakeMode(false);							// default to brake mode DISABLED
		_gearInfeedMotor.setNeutralMode(NeutralMode.Coast);
		//_gearInfeedMotor.enableLimitSwitch(false, false);
		_gearInfeedMotor.overrideLimitSwitchesEnable(false);
		_gearInfeedMotor.overrideLimitSwitchesEnable(false);		
		ZeroGearTiltAxisInit();
	}

	//============================================================================================
	// Methods follow
	//============================================================================================	
	
    public void ZeroGearTiltAxisInit() {
		_gearTiltMoveLastTargetPosition = GEAR_TILT_MOVE_LAST_TARGET_POSITION.UNDEFINED;   	
		
		// snapshot the current time so we can enforce the timeout
		_gearTiltAxisStateStartTime = System.currentTimeMillis();
    	
    	// did we start on the limit switch? (remember switch is normally closed!)
		if(getIsOnTiltHomeLimtSwitch()) {
			_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.AT_HOME;
			DriverStation.reportWarning("TiltAxis (Zero State) [INITIAL] ==> [AT_HOME]", false);
		} else {
			_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.MOVING_TO_HOME;
			DriverStation.reportWarning("TiltAxis (Zero State) [INITIAL] ==> [MOVING_TO_HOME]", false);
		}
    }
	
	// Re-entrant method that will zero the Tilt Axis
    public void ZeroGearTiltAxisReentrant() {
    	switch(_gearTiltAxisZeroCurrentState) {    					
    		case MOVING_TO_HOME:
    			// are we on the limit switch? (remember switch is normally closed!
				if(getIsOnTiltHomeLimtSwitch()) 
				{
    				_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.AT_HOME;
				} 
				else 
				{
    				// check for timeout
    				long elapsedTime = System.currentTimeMillis() - _gearTiltAxisStateStartTime;
					if (elapsedTime < GEAR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC) 
					{
							_gearTiltMotor.set(ControlMode.PercentOutput,GEAR_MOVE_TO_HOME_VELOCITY_CMD);
					} 
					else 
					{
						System.out.println("Timing Out");
    					_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.TIMEOUT;
    				}		
    			}
    			break;
    			
    		case AT_HOME:
    			// chg to PID-Position mode
				_gearTiltMotor.set(ControlMode.MotionMagic, 0);
    			
    			// reset encoder position
    			_gearTiltMotor.setSelectedSensorPosition(0, 0, 10);
    			
    			// set current target position to be the home position
    			_gearTiltMotor.set(ControlMode.MotionMagic, GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS);
    			
    			_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.ZEROED;
    			_gearTiltMoveLastTargetPosition = GEAR_TILT_MOVE_LAST_TARGET_POSITION.MOVING_TO_HOME;
    			DriverStation.reportWarning("TiltAxis (Zero State) [AT_HOME] ==> [ZEROED]", false);
    			break;
    			
    		case TIMEOUT:
    			DriverStation.reportWarning("Gear Tilt Zero Timed Out", false);
    			break;
    			
    		case UNDEFINED:
    			DriverStation.reportWarning("Gear Tilt Zero State Undefined", false);
    			break;
    	}
    }
    
    public void MoveGearToHomePosition() {
    	MoveTiltAxisPIDP(GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS);
    	_isLastTiltMoveToFloorCallComplete = true;
    	
    	DriverStation.reportWarning("Move Gear To Home Position", false);
    }
    
    public void MoveGearToScorePosition() {
    	MoveTiltAxisPIDP(GEAR_TILT_SCORING_POSITION_IN_ROTATIONS);
    	_isLastTiltMoveToFloorCallComplete = true;
    	
    	//DriverStation.reportWarning("Move Gear To Score Position", false);
    }
    
    public void MoveGearToFloorPositionReentrant() {
		if(_gearTiltMotor.getSelectedSensorPosition(0) >= (GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS - TARGET_DEADBAND)) {
			// gravity fall to floor
			MoveTiltAxisVBus(0.0);
			_isLastTiltMoveToFloorCallComplete = true;
		} else {
			MoveTiltAxisPIDP(GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS);
			_isLastTiltMoveToFloorCallComplete = false;
		}
		
		DriverStation.reportWarning("Move Gear To Floor Position", false);
    }
    
	public void MoveTiltAxisPIDP (double positionCmd) 
	{
		_gearTiltMotor.set(ControlMode.MotionMagic,positionCmd);
    }

	public void MoveTiltAxisVBus(double percentVBusCmd) 
	{
		MoveTiltAxisVBus(percentVBusCmd, true);
	}
    
	public void MoveTiltAxisVBus(double percentVBusCmd, boolean isUseRawCmd) {
		// if(_gearTiltMotor.getControlMode() != CANTalon.TalonControlMode.PercentVbus) {
		// 	_gearTiltMotor.changeControlMode(TalonControlMode.PercentVbus);
		// }
		
		if(isUseRawCmd) 
		{
			_gearTiltMotor.set(ControlMode.PercentOutput,percentVBusCmd);			
		} 
		else 
		{
			// limit max speed to 25%	=>  * 0.25
			_gearTiltMotor.set(ControlMode.PercentOutput, percentVBusCmd * 0.25);	
		}
		
		// cancel any button move that was in process
		_isLastTiltMoveToFloorCallComplete = true;
	}
	
	public void SpinInfeedWheelsVBus(double percentVBusCmd) 
	{
		// invert motor command		=>	* -1.0
		// limit max speed to 50%	=>  * 0.50
		_gearInfeedMotor.set(ControlMode.PercentOutput, percentVBusCmd * -1.0 * 0.50);
	}
	
	private  String getTiltPosition() {
		if((_gearTiltMotor.getControlMode() == CANTalon.TalonControlMode.PercentVbus) 
				&& (Math.abs(_gearTiltMotor.get()) > 0.0)) {
			_gearTiltState = "Joystick";
		}
		else if ((_gearTiltAxisZeroCurrentState != GEAR_TILT_HOMING_STATE.ZEROED)) {
			_gearTiltState = "Unknown";
		}
		else if(Math.abs(_gearTiltMotor.getSelectedSensorPosition(0)) <= TARGET_DEADBAND) {
			_gearTiltState = "Home";
		}
		else if(Math.abs(_gearTiltMotor.getSelectedSensorPosition(0) - GEAR_TILT_SCORING_POSITION_IN_ROTATIONS) <= TARGET_DEADBAND) {
			_gearTiltState = "Scoring";
		}
		else if(Math.abs(_gearTiltMotor.getSelectedSensorPosition(0) - GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS) > 0) {
			_gearTiltState = "Floor";
		}
		else {
			_gearTiltState = "Unknown.";
		}
		
		return _gearTiltState;
	}
	
	public void FlapAnnReentrant()
	{	
		if((System.currentTimeMillis() - _flapAnnReentrantRunningMsec) < 1600)	{		// 800 ; 500
			MoveGearToScorePosition();
		}
		//else if((System.currentTimeMillis() - _flapAnnReentrantRunningMsec) < 4000) {	// 800 ; 500
			// pause
		//}
		else if((System.currentTimeMillis() - _flapAnnReentrantRunningMsec) < 2000) {	// 40; 20; 40
			MoveGearToHomePosition();
		} 
		//else if((System.currentTimeMillis() - _flapAnnReentrantRunningMsec) < 8000) {	// 800 ; 500
			// pause
		//}
		else {
			_flapAnnReentrantRunningMsec = System.currentTimeMillis();
		}
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	private boolean getIsOnTiltHomeLimtSwitch() {
		// remember switch is normally closed!
		return !_gearTiltMotor.isRevLimitSwitchClosed();
	}
	
	public boolean hasTiltAxisBeenZeroed() {
		if (_gearTiltAxisZeroCurrentState == GEAR_TILT_HOMING_STATE.ZEROED) {
			return true;
		} else {
			return false;
		}
	}
	
	//allows robot class to check if we are in moving to floor mode so it can keep calling
	public GEAR_TILT_MOVE_LAST_TARGET_POSITION get_gearTiltMoveToPosition() {
		return _gearTiltMoveLastTargetPosition;
	}
	
	public boolean IsGearInScoringPosition() {
		if(Math.abs(_gearTiltMotor.getPosition() - GEAR_TILT_SCORING_POSITION_IN_ROTATIONS) <= TARGET_DEADBAND) {
			return true;
		} else {
			return false;
		}
	}
	
	public boolean getIsLastTiltMoveToFloorCallComplete() {
		return _isLastTiltMoveToFloorCallComplete;
	}

	@Override
	public void stop() {
		MoveTiltAxisVBus(0.0);
		_gearInfeedMotor.set(0.0);
	}

	@Override
	public void zeroSensors() {
		_gearTiltMotor.setPosition(0.0);
	}

	@Override
	public void outputToSmartDashboard() {
		//%s - insert a string
		//%d - insert a signed integer (decimal)
		//%f - insert a real number, standard notation
		
		String gearTiltMtrData = "?";
		// we only really knwo position after we have zeroed
		if(_gearTiltAxisZeroCurrentState == GEAR_TILT_HOMING_STATE.ZEROED) {
			gearTiltMtrData = String.format("%s (%.3f)", getTiltPosition(), _gearTiltMotor.getPosition());
		} else {
			gearTiltMtrData = String.format("%s (%s)", getTiltPosition(), "???");
		}
		SmartDashboard.putString("Gear Tilt Position", gearTiltMtrData);
		
		SmartDashboard.putString("Gear Tilt State", getTiltPosition());
		
		String gearInFeedMtrData = "?";
		if(Math.abs(_gearInfeedMotor.getOutputVoltage()) > 0) {
			gearInFeedMtrData = String.format("%s (%.0f%%)", 
												"ON", 
												(_gearInfeedMotor.getOutputVoltage() / _gearInfeedMotor.getBusVoltage())* 100);
        } 
        else 
        {
			gearInFeedMtrData = String.format("%s (%.0f%%)", "off", 0.0);
		}	
	}

	@Override
	protected void initDefaultCommand() 
	{

	}
}