// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports.BeamBreakPorts;

public class BeamBreak extends SubsystemBase {

  /** Creates a new BeamBreak. */
  private final DigitalInput receiver;
  // private final DigitalOutput emitter;

  public BeamBreak() {
    receiver = new DigitalInput(BeamBreakPorts.RECEIVER);
    // emitter = new DigitalOutput(BeamBreakPorts.EMITTER);
    // setEmitter(true); // added it as default command in robot container
  }

  public boolean getReceiverStatus() {
    return receiver.get();
    // true -> not broken
    // false -> broken
  }

  // public boolean getEmitterStatus() {
  //   return emitter.get();
  // }

  // public void setEmitter(boolean status) {
  //   emitter.set(status);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("receiver status: " + getReceiverStatus());
    // System.out.println("emitter status: " + getEmitterStatus());

    SmartDashboard.putBoolean("receiver status", getReceiverStatus());
    // SmartDashboard.putBoolean("emitter status", getEmitterStatus());
  }
}
