// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HopperConstants;
import frc.robot.Ports.HopperPorts;

public class Hopper extends SubsystemBase {

  private final CANSparkMax hopperMotor;
  private final RelativeEncoder hopperEncoder;

  private final SimpleMotorFeedforward hopperFF;
  // private final PIDController hopperPID;
  private final DigitalInput receiver;

  private final SysIdRoutine hopperRoutine;

  private double vSetpoint;
  private boolean currentState, lastState;
  private int stateCount = 0;

  /** Creates a new Hopper. */
  public Hopper() {
    hopperMotor = new CANSparkMax(HopperPorts.HOPPER_MOTOR, MotorType.kBrushless);
    hopperEncoder = hopperMotor.getEncoder();

    hopperEncoder.setVelocityConversionFactor(HopperConstants.VEL_CFACTOR);

    hopperFF = new SimpleMotorFeedforward(
        HopperConstants.kS,
        HopperConstants.kV);

    receiver = new DigitalInput(HopperPorts.RECEIVER);

    hopperMotor.setIdleMode(IdleMode.kBrake); // prevent note from slipping out of hopper
    hopperMotor.setSmartCurrentLimit(HopperConstants.CURRENT_LIMIT);
    hopperMotor.burnFlash();

    hopperRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts -> setVoltage(volts.in(Units.Volts)), null, this));

    vSetpoint = 0;
    currentState = getReceiverStatus();
    lastState = currentState;
  }

  /* COMMANDS */
  public Command feedNote() {
    return Commands.waitUntil(() -> isHopperFull())
      .andThen(Commands.waitUntil(() -> isHopperEmpty())) // denotes when cmd ends
      .deadlineWith(setVelocityCmd()) // runs hopper motors until note has been fed into shooter
      .finallyDo(() -> resetStateCountCmd()); // reset the state count
  }

  public Command setHopperSpeedCmd(double speed) {
    System.out.println("setting hopper speed");
    return this.runOnce(() -> setSpeed(speed));
  }

  public Command setVelocitySetpointCmd(double setpoint) {
    // return Commands.print("setting hopper vel setpoint");
    return this.runOnce(() -> setVelocitySetpoint(setpoint));
  }

  public Command setVelocityCmd() {
    // return Commands.print("running hopper velocity pid");
    return this.run(() -> setVelocity());
  }

  public Command stopMotorCmd() {
    return this.runOnce(() -> stopMotor());
  }

  public Command resetStateCountCmd() {
    return this.runOnce(() -> resetStateCount());
  }

  /* * * BEAM BREAK * * */
  public boolean getReceiverStatus() {
    return receiver.get();
    // true = unbroken
    // false = broken
  }

  // hopper is empty once state change count == 2;
  public boolean isHopperEmpty() {
    if (hasStateChanged()) {
      stateCount++;
      System.out.println(stateCount);
    }
    return stateCount == 2;
  }

  public boolean isHopperFull() {
    return !getReceiverStatus();
  }

  public void resetStateCount() {
    stateCount = 0;
  }

  // checks if beam break has changed from broken to unbroken
  public boolean hasStateChanged() {

    boolean stateChange;
    currentState = getReceiverStatus();
    stateChange = currentState && !lastState; // currently true, was false;
    lastState = currentState;

    return stateChange;
  }

  // sets fractional duty cycle
  public void setSpeed(double speed) {
    hopperMotor.set(speed);
  }

  public void setVelocity() {
    double voltage = hopperFF.calculate(vSetpoint);
    hopperMotor.setVoltage(voltage);
  }

  public void setVelocitySetpoint(double setpoint) {
    vSetpoint = setpoint;
  }

  public double getVelocity() {
    return hopperEncoder.getVelocity();
  }

  public void stopMotor() {
    hopperMotor.stopMotor();
  }

  /* SYSID */
  public void setVoltage(double voltage) {
    hopperMotor.setVoltage(voltage);
  }

  public Command hopperQuas(SysIdRoutine.Direction direction) {
    return hopperRoutine.quasistatic(direction);
  }

  public Command hopperDyna(SysIdRoutine.Direction direction) {
    return hopperRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("hopper vel", getVelocity());
    SmartDashboard.putNumber("hopper sp", vSetpoint);

    SmartDashboard.putBoolean("beam break", getReceiverStatus());
  }
}
