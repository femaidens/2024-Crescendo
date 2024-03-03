// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.Ports.*;

public class Intake extends SubsystemBase {

  private final CANSparkMax intakeMotor;

  private final RelativeEncoder intakeEncoder;
  

  //private final PIDController intakePID;
  private final SimpleMotorFeedforward intakeFF;

 // private final PIDController hopperPID;


  private final DigitalInput receiver;
  // private final DigitalOutput emitter;

  private final SysIdRoutine intakeRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          volts -> setVoltage(volts.in(Units.Volts)), null, this));

 

  private double vIntakeSetpoint;

  public Intake() {
    intakeMotor =
      new CANSparkMax(IntakePorts.INTAKE_ROLLER, MotorType.kBrushless);
    

    intakeEncoder = intakeMotor.getEncoder();

    intakeEncoder.setVelocityConversionFactor(IntakeConstants.VEL_CFACTOR);
   

    // intakePID = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    // hopperPID = new PIDController(HopperConstants., vSetpoint, vSetpoint)
    
    intakeFF = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV);
    
    receiver = new DigitalInput(HopperPorts.RECEIVER);
    // emitter = new DigitalOutput(HopperPorts.EMITTER);

    intakeMotor.setIdleMode(IdleMode.kCoast); // should freely spin?

    intakeMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);

    intakeMotor.burnFlash();
   

    // setEmitter(true);

    vIntakeSetpoint = 0;
 
  }

  public void setIntakeVelocity() {
    double voltage = intakeFF.calculate(vIntakeSetpoint);
    //double error = intakePID.calculate(intakeEncoder.getVelocity(), vSetpoint);

    intakeMotor.setVoltage(voltage);
  }

  public void setIntakeVelocitySetpoint(double setpoint) {
    vIntakeSetpoint = setpoint;
  }

  public void setVoltage(double voltage){
    intakeMotor.setVoltage(voltage);
  }


  // for testing; see if vel pid is absolutely necessary
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

 

  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }


  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
  }

  // beam breaker code
  public boolean getReceiverStatus() {
    return receiver.get();
  }

  public Command intakeQuas(SysIdRoutine.Direction direction) {
    return intakeRoutine.quasistatic(direction);
  }

  public Command intakeDyna(SysIdRoutine.Direction direction) {
    return intakeRoutine.dynamic(direction);
  }





  // public boolean getEmitterStatus() {
  //   return emitter.get();
  // }

  // public void setEmitter(boolean status) {
  //   emitter.set(status);
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake vel", getIntakeVelocity());

    SmartDashboard.putNumber("intake sp", vIntakeSetpoint);

    SmartDashboard.putBoolean("beam break", getReceiverStatus());
    // System.out.println("hopper velocity: " + getHopperVelocity());
  }
}
