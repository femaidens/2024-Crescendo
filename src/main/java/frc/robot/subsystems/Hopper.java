// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Ports.HopperPorts;

public class Hopper extends SubsystemBase {
  private final DigitalInput beamBreakIntake;
  private final DigitalInput beamBreakShooter;

  private final CANSparkMax hopperMotor;

  /** Creates a new Hopper. */
  public Hopper() {
    beamBreakIntake = new DigitalInput(HopperPorts.BEAM_BREAK_INTAKE_PORT);
    beamBreakShooter = new DigitalInput(HopperPorts.BEAM_BREAK_SHOOTER_PORT);

    hopperMotor = new CANSparkMax(HopperPorts.HOPPER_MOTOR_PORT, MotorType.kBrushless);
    hopperMotor.setIdleMode(IdleMode.kBrake);
    hopperMotor.setSmartCurrentLimit(HopperConstants.HOPPER_CURRENT_LIMIT);
    
  }

  public void stopHopperMotor(){
    hopperMotor.setVoltage(0);
  }

  public void checkbeamBreakIntake(){
    if(beamBreakIntake.get()){
      hopperMotor.set(0.5);
      //retract intake + stop intake motors
  }

  public void checkbeamBreakShooter(){
    if(!beamBreakShooter.get()){
      hopperMotor.set(0.5);
    }else{
      stopHopperMotor();
      System.out.println("beam break shooter sensor activated \n")
    }
  }

  public void 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
