package org.usfirst.frc.team4028.robot.commands.auton;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.RobotMap;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.util.GeneralUtilities;

public class Auton_CG_ChassisTune extends CommandGroup{

TalonSRX _leftMaster;
TalonSRX _rightMaster;
TalonSRX _leftSlave;
TalonSRX _rightSlave;
Chassis _chassis = Chassis.getInstance();

    public Auton_CG_ChassisTune(){

        _leftMaster = new TalonSRX(RobotMap.LEFT_DRIVE_MASTER_CAN_ADDR);
        _leftSlave = new TalonSRX(RobotMap.LEFT_DRIVE_SLAVE_CAN_ADDR);
        _rightMaster = new TalonSRX(RobotMap.RIGHT_DRIVE_MASTER_CAN_ADDR);
        _rightSlave = new TalonSRX(RobotMap.RIGHT_DRIVE_SLAVE_CAN_ADDR);
    

        _leftSlave.follow(_leftMaster);
        _rightSlave.follow(_leftMaster);
        _rightMaster.follow(_leftMaster);
    
        _leftMaster.setInverted(true);
        _leftSlave.setInverted(true);
        _rightMaster.setInverted(false);
        _rightSlave.setInverted(false);

        _leftMaster.setIntegralAccumulator(0, 0, 10);
        _leftSlave.setIntegralAccumulator(0, 0, 10);
        _rightMaster.setIntegralAccumulator(0, 0, 10);
        _rightSlave.setIntegralAccumulator(0, 0, 10);

        double[] zeroes = {.1,0,0,0};

        GeneralUtilities.setPIDFGains(_leftMaster, zeroes);
        GeneralUtilities.setPIDFGains(_leftSlave, zeroes);
        GeneralUtilities.setPIDFGains(_rightMaster, zeroes);
        GeneralUtilities.setPIDFGains(_rightSlave, zeroes);


        TalonSRX[] listOSlaves = {_rightMaster, _rightSlave, _leftSlave};


        addParallel(new Auton_ParallelStarter());
        addSequential(new Auton_CG_PIDTune(_chassis, _leftMaster, 0, Chassis.getInstance().inchesPerSecToNU(180) , 100, listOSlaves));

        

    }
}