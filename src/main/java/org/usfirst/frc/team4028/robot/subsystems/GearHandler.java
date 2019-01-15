package org.usfirst.frc.team4028.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

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
	
	// --------------------------------------------------------
	// define Tilt Motor PID constants
	private static final int 	TILT_PID_P_PROFILE = 0;
	private static final double TILT_PID_P_CONSTANT = 1.6;
	private static final double TILT_PID_I_CONSTANT = 0.0;
	private static final double TILT_PID_D_CONSTANT = 50.0;
	private static final double TILT_PID_F_CONSTANT = 0.2;
	private static final double TILT_PID_RAMP_RATE = 0.1;
		
	private static final double TILT_MAX_V_DOWN_TILT = +3.0; // Down is positive (RIP MAXIMUM...)
	private static final double TILT_MAX_V_UP_TILT = -6.0;
	// --------------------------------------------------------
	private static final double ZERO_POSITION = 0;
	private static final double SCORE_POSITION = 351;
	private static final double FLOOR_POSITION = 1700;
	
	private static final double GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS = 00.00;
	private static final double GEAR_TILT_SCORING_POSITION_IN_ROTATIONS = 0.1;
	private static final double GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS = 00.48;
	private static final double TARGET_DEADBAND = 00.03;
	private static final double GEAR_TILT_ENCODER_COUNTS_PER_ROTATION = 3580; //Calculated in 2019
	
	public static double _targetPos = GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS;

	//============================================================================================
	// constructors follow
	//============================================================================================
	private GearHandler() 
	{
		// Tilt Motor
		_gearTiltMotor = new TalonSRX(RobotMap.GEAR_TILT_CAN_ADDR);
		_gearTiltMotor.setNeutralMode(NeutralMode.Coast);							// default to brake mode DISABLED
		_gearTiltMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,10);	// set encoder to be feedback device
		_gearTiltMotor.setSensorPhase(false);  							// do not invert encoder feedback
		_gearTiltMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled,10);
		_gearTiltMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
	
		_gearTiltMotor.config_kP(0, TILT_PID_P_CONSTANT, 10);
		_gearTiltMotor.config_kD(0, TILT_PID_D_CONSTANT, 10);
		_gearTiltMotor.config_kF(0, TILT_PID_F_CONSTANT, 10);
		_gearTiltMotor.configMotionCruiseVelocity(1200, 10);
		_gearTiltMotor.configMotionAcceleration(1200, 10);
		_gearTiltMotor.configNominalOutputForward(0.0, 10);
		_gearTiltMotor.configNominalOutputReverse(-0.0, 10);
		_gearTiltMotor.configPeakOutputForward(TILT_MAX_V_DOWN_TILT, 10);
		_gearTiltMotor.configPeakOutputReverse(TILT_MAX_V_UP_TILT, 10);

		
		// Infeed Motor
		_gearInfeedMotor = new TalonSRX(RobotMap.GEAR_INFEED_CAN_ADDR);
		_gearInfeedMotor.setNeutralMode(NeutralMode.Coast);
		_gearInfeedMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated,LimitSwitchNormal.Disabled,10);
		_gearInfeedMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated,LimitSwitchNormal.Disabled, 10);		
	}

	//============================================================================================
	// Methods follow
	//============================================================================================	

	public double getScorePosition()
	{
		return SCORE_POSITION;
	}

	public double getFloorPosition()
	{
		return FLOOR_POSITION;
	}

	public double getZeroPosition()
	{
		return ZERO_POSITION;
	}
	
    
	
	public void SpinInfeedWheelsVBus(double percentVBusCmd) 
	{
		// invert motor command		=>	* -1.0
		// limit max speed to 50%	=>  * 0.50
		_gearInfeedMotor.set(ControlMode.PercentOutput, percentVBusCmd*.5);
	}
	
	

	public void stop() {
		_gearTiltMotor.set(ControlMode.PercentOutput,0.0);
		_gearInfeedMotor.set(ControlMode.PercentOutput,0.0);
	}

	public void zeroSensors() {
		_gearTiltMotor.setSelectedSensorPosition(0, 0, 10);
	}

	public double getCurrentPosition()
	{
		return _gearTiltMotor.getSelectedSensorPosition(0);
	}

	public void outputToSmartDashboard()
	{
	
	}
	public void moveToZero()
	{
		_gearTiltMotor.set(ControlMode.PercentOutput,-0.5);
		System.out.println("moving to zero");
	}
	public boolean getIsOnTiltHomeLimtSwitch()
	{
		return !_gearTiltMotor.getSensorCollection().isRevLimitSwitchClosed();
	}

	public void moveTowardsTargetPosition(double _targetPos){
		_gearTiltMotor.set(ControlMode.MotionMagic, _targetPos);
	}

	protected void initDefaultCommand() 
	{

	}
	public double getError()
	{
		return Math.abs(_gearTiltMotor.getSelectedSensorPosition(0) - _targetPos);

	}
	public double getTargetPosition()
	{
		return (_targetPos);
	}


}