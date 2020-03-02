/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4028.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap 
{
	// Drivers Station Gamepad USB Ports
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	public static final int ENGINEERING_GAMEPAD_USB_PORT = 2;
	public static final int ENGINEERING_GAMEPAD_B_USB_PORT = 3;
	
	// PCM Can Bus Address
	public static final int PCM_CAN_ADDR = 0;	
	
	// Talons Can Bus Address
	public static final int LEFT_DRIVE_MASTER_CAN_ADDR = 11;
	public static final int LEFT_DRIVE_SLAVE_CAN_ADDR = 12;
	public static final int RIGHT_DRIVE_MASTER_CAN_ADDR = 9;
	public static final int RIGHT_DRIVE_SLAVE_CAN_ADDR = 10;
	public static final int GEAR_INFEED_CAN_ADDR = 8;
	public static final int GEAR_TILT_CAN_ADDR = 7;
	
	// DIO Ports
	
	// Analog Ports
	public static final int STORED_PRESSURE_SENSOR_AIO_PORT = 0;	
	
	// NavX (on Roborio)
	public static final SPI.Port NAVX_PORT = Port.kMXP;
	
	// PWM Ports on RoboRIO
	public static final int CLIMBER_SERVO_PWM_ADDRESS = 0;
	
	// PCM Ports

	public static final int SHIFTER_EXTEND_PCM_PORT = 6;
	public static final int SHIFTER_RETRACT_PCM_PORT = 7;
}