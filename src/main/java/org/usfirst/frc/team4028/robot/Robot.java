/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.DriverStation;
// #region Import Statements
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Date;

import org.usfirst.frc.team4028.robot.auton.pathfollowing.Paths;
import org.usfirst.frc.team4028.robot.commands.Elevator_ZeroElevator;
import org.usfirst.frc.team4028.robot.commands.Infeed_ZeroInfeedArms;
import org.usfirst.frc.team4028.robot.sensors.SwitchableCameraServer;
import org.usfirst.frc.team4028.robot.subsystems.Carriage;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Climber;
import org.usfirst.frc.team4028.robot.subsystems.Elevator;
import org.usfirst.frc.team4028.robot.subsystems.Infeed;
import org.usfirst.frc.team4028.robot.util.GeneralUtilities;
import org.usfirst.frc.team4028.robot.util.LogDataBE;
import org.usfirst.frc.team4028.robot.util.MovingAverage;
import org.usfirst.frc.team4028.robot.util.DataLogger;
// #endregion

/**
 * The VM is configured to automatically run this class
 */
public class Robot extends TimedRobot 
{
	private static final String ROBOT_NAME = "2018 PowerUp (ECLIPSE)-CMD BASED";
	
	// create instance of singelton Subsystems
	private Dashboard _dashboard = Dashboard.getInstance();
	
	private Carriage _carriage = Carriage.getInstance();
	private Chassis _chassis = Chassis.getInstance();
	private Climber _climber = Climber.getInstance();
	private Elevator _elevator = Elevator.getInstance();
	private Infeed _infeed = Infeed.getInstance();
	private OI _oi = OI.getInstance();
	private SwitchableCameraServer _camera = SwitchableCameraServer.getInstance();

	
	// class level working variables
	private DataLogger _dataLogger = null;
	private String _buildMsg = "?";
 	long _lastScanEndTimeInMSec = 0;
 	long _lastDashboardWriteTimeMSec;
 	MovingAverage _scanTimeSamples;
 	
	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit() 
	{
		_chassis.stop();
		Paths.buildPaths();
		_buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);

		outputAllToDashboard();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();

	}

	/**
	 * This method runs 1x when the robot enters auton mode
	 */
	@Override
	public void autonomousInit() {
		_chassis.stop();
		_dashboard.getSelectedAuton().start();
		Scheduler.getInstance().run();

		_chassis.recordAutonStartTime();
		_chassis.zeroSensors();
		_chassis.setHighGear(true);

		int retries = 100;
		
		while(!_dashboard.isGameDataReceived() && retries > 0) {
			retries--;
			try { 
				Thread.sleep(5);
			} catch (InterruptedException ie) {}
		}
		
		if (retries == 0) {
			DriverStation.reportError("Failed To Receive Game Data", false);
		}
		if (!_infeed.get_hasArmsBeenZeroed()) {
			Command reZeroInfeedArmsCommand = new Infeed_ZeroInfeedArms();
			reZeroInfeedArmsCommand.start();
		}
		if (!_elevator.get_hasElevatorBeenZeroed()) {
			Command reZeroElevatorCommand = new Elevator_ZeroElevator();
			reZeroElevatorCommand.start();
		}
		_lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
		_dataLogger = GeneralUtilities.setupLogging("Auton"); // init data logging	
		_dashboard.outputToDashboard();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() 
	{
		Scheduler.getInstance().run();
		_chassis.updateChassis(Timer.getFPGATimestamp());
		// System.out.println(_chassis.isDoneWithPath());
		
		// ============= Refresh Dashboard =============
		_dashboard.outputToDashboard();
		outputAllToDashboard();
		
		// ============= Optionally Log Data =============
		logAllData();
	}

	/**
	 * This method runs 1x when the robot enters telop mode
	 */
	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.


		
		if (!_infeed.get_hasArmsBeenZeroed()) {
			Command reZeroInfeedArmsCommand = new Infeed_ZeroInfeedArms();
			reZeroInfeedArmsCommand.start();
		}
		if (!_elevator.get_hasElevatorBeenZeroed()) {
			Command reZeroElevatorCommand = new Elevator_ZeroElevator();
			reZeroElevatorCommand.start();
		}
		_chassis.stop();
		_lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
		_dataLogger = GeneralUtilities.setupLogging("Teleop"); // init data logging
		_lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		// ============= Refresh Dashboard =============
		outputAllToDashboard();
		_dashboard.outputToDashboard();
		
		// ============= Optionally Log Data =============
		logAllData();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {}
	
	/** Method to Push Data to ShuffleBoard */
	private void outputAllToDashboard() {
		// limit spamming
    	long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    	// add scan time sample to calc scan time rolling average
    	//_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
    		// each subsystem should add a call to a outputToSmartDashboard method
    		// to push its data out to the dashboard

    		_chassis.updateDashboard(); 
    		_elevator.updateDashboard();
    		_infeed.updateDashboard();
    		_carriage.updateDashboard();
	    	_climber.updateDashboard();
	    	
    		// write the overall robot dashboard info
	    	SmartDashboard.putString("Robot Build", _buildMsg);
	    	
	    	//BigDecimal movingAvg = _scanTimeSamples.getAverage();
	    	//DecimalFormat df = new DecimalFormat("####");
	    	//SmartDashboard.putString("Scan Time (2 sec roll avg)", df.format(movingAvg) + " mSec");
    		// snapshot last time
    		_lastDashboardWriteTimeMSec = new Date().getTime();
    	}
    	
    	// snapshot when this scan ended
    	_lastScanEndTimeInMSec = new Date().getTime();
	}
	
	/** Method for Logging Data to the USB Stick plugged into the RoboRio */
	private void logAllData() { 
		// always call this 1st to calc drive metrics
    	if(_dataLogger != null) {    	
	    	// create a new, empty logging class
        	LogDataBE logData = new LogDataBE();
	    	
	    	// ask each subsystem that exists to add its data
	    	_chassis.updateLogData(logData);
	    	_elevator.updateLogData(logData);
	    	_infeed.updateLogData(logData);
	    	_carriage.updateLogData(logData);
			_climber.updateLogData(logData);
			
	    	_dataLogger.WriteDataLine(logData);
    	}
	}
}
