package org.usfirst.frc.team4028.robot.commands.auton;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team4028.robot.auton.util.computeMean;
import org.usfirst.frc.team4028.robot.auton.util.beakCircularBuffer;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;


class Auton_PIDConfig extends Command{




private int _samplesRequired;
private int _samplesGathered = 0;
private int _paramterSlot = 0;
double _desiredVelocity = 0;
private boolean _fQ;


private TalonSRX _talon;
private StringBuilder _sb;
private beakCircularBuffer cBuffSpeed;
private beakCircularBuffer cBuffError;
private TalonSRX[] _slavesList = {};


private computeMean meanComputer = new computeMean();

public Auton_PIDConfig(Subsystem requiredSubsystem, TalonSRX talon, int srxParameterSlot, double desiredVelocity,
        int numSamplesRequired, boolean isF) {
    this._fQ = isF;
    this._talon = talon;
    this._samplesRequired = numSamplesRequired;
    this._samplesGathered = 0;
    this.cBuffSpeed = new beakCircularBuffer(_samplesRequired);
    this.cBuffError = new beakCircularBuffer(_samplesRequired);
    this._paramterSlot = srxParameterSlot;
    this._sb = new StringBuilder();
    this._desiredVelocity = desiredVelocity;
    requires(requiredSubsystem);
}

public Auton_PIDConfig(Subsystem requiredSubsystem, TalonSRX talon, int srxParameterSlot, double desiredVelocity,
        int numSamplesRequired, boolean isF, TalonSRX[] slaveTalons) {
    this._fQ = isF;
    this._talon = talon;
    this._samplesRequired = numSamplesRequired;
    this._samplesGathered = 0;
    this.cBuffSpeed = new beakCircularBuffer(_samplesRequired);
    this.cBuffError = new beakCircularBuffer(_samplesRequired);
    this._paramterSlot = srxParameterSlot;
    this._sb = new StringBuilder();
    this._desiredVelocity = desiredVelocity;
    this._slavesList = slaveTalons;
    requires(requiredSubsystem);
}

// Called just before this Command runs the first time
protected void initialize() {
    System.out.println("Gathering Data...");
}

// Called repeatedly when this Command is scheduled to run
protected void execute() {
    double outputSignal = _talon.getMotorOutputVoltage() / _talon.getBusVoltage();

    double closedLoopError = Math.PI;

    if (_fQ){
        double speed = _talon.getSelectedSensorVelocity(_paramterSlot);
        cBuffSpeed.addLast(speed);
    } else {

        cBuffError.addLast(Math.abs((double)_talon.getClosedLoopError(_paramterSlot)));
    
    }

    _samplesGathered++;
    
    /*
    _sb.append("\tOutput: ");
    _sb.append(outputSignal);
    _sb.append("\tSpeed: ");
    _sb.append(speed);
    _sb.append("\n");
    */
    

    if (_samplesGathered % 10 == 0) {
         System.out.println("Closed loop error: " + _talon.getClosedLoopError(_paramterSlot));
     }
}

// Make this return true when this Command no longer needs to run execute()
protected boolean isFinished() {
    return _samplesGathered >= _samplesRequired;
}

// Called once after isFinished returns true
protected void end() {
    if (_fQ){
        double kF =  1023/ meanComputer.mean(cBuffSpeed.toArray());
        _talon.config_kF(_paramterSlot, kF, 10);
        System.out.println("Calculated F gain = " + kF);
        for (TalonSRX slave : _slavesList){
            slave.config_kF(_paramterSlot, kF, 10);
        }
    } else {
        double kP = .1*1023/meanComputer.mean(cBuffError.toArray());
        _talon.config_kP(_paramterSlot, kP, 10);
        System.out.println("Calculated P Gain = " + kP);
        for (TalonSRX slave : _slavesList){
            slave.config_kF(_paramterSlot, kP, 10);
        }
    }



}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
protected void interrupted() {
    _talon.set(ControlMode.PercentOutput, 0);
}

}
