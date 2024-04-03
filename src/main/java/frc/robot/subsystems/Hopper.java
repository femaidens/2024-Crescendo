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
import frc.robot.Ports.BeamBreakPorts;
import frc.robot.Ports.HopperPorts;
import monologue.Logged;
import monologue.Annotations.Log;

public class Hopper extends SubsystemBase implements Logged {

  // @Log.NT
  private final CANSparkMax hopperMotor;

  // @Log.NT
  private final RelativeEncoder hopperEncoder;

  // @Log.NT
  private final SimpleMotorFeedforward hopperFF;
  // private final PIDController hopperPID;

  // @Log.NT
  private final DigitalInput receiver;

  private final SysIdRoutine hopperRoutine;

  // @Log.NT
  private double vSetpoint;
  
  private boolean currentState, lastState;
  private int stateCount = 0;
  private int stateLimit = 2;

  /** Creates a new Hopper. */
  public Hopper() {
    hopperMotor = new CANSparkMax(HopperPorts.HOPPER_MOTOR, MotorType.kBrushless);
    hopperEncoder = hopperMotor.getEncoder();

    hopperEncoder.setVelocityConversionFactor(HopperConstants.VEL_CFACTOR);

    hopperFF = new SimpleMotorFeedforward(
        HopperConstants.kS,
        HopperConstants.kV);

    receiver = new DigitalInput(BeamBreakPorts.RECEIVER);

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
    // .beforeStarting(resetStateCountCmd())
    .andThen(setVelocitySetpointCmd(HopperConstants.TRANSITION_SPEED))
    .andThen(Commands.waitUntil(() -> isHopperEmpty()))
    .andThen(setVelocitySetpointCmd(0));
    // .finallyDo(() -> setVelocitySetpointCmd(0));
  }

  public Command setSpeedCmd(double speed) {
    System.out.println("setting hopper speed");
    return this.runOnce(() -> setSpeed(speed));
  }

  // is default command, DO NOT ADD AS PROXY
  public Command setVelocityCmd() {
    // return Commands.print("running hopper velocity pid");
    return this.run(() -> setVelocity());
  }

  public Command setVelocitySetpointCmd(double setpoint) {
    // return Commands.print("setting hopper vel setpoint");
    return this.runOnce(() -> setVelocitySetpoint(setpoint)).asProxy();
  }

  public Command setVelocityCmd(double setpoint) {
    return this.run(() -> setVelocity(setpoint));
  }

  public Command stopMotorCmd() {
    return this.runOnce(() -> stopMotor());
  }

  public Command setStateLimitCmd(int limit) {
    return this.runOnce(() -> setStateLimit(limit)).asProxy();
  }

  public Command resetStateCountCmd() {
    System.out.println("state count reset");
    return this.runOnce(() -> resetStateCount()).asProxy();
  }

  public Command resetStateEmergencyCmd() {
    return this.runOnce(() -> resetStateEmergency()).asProxy();
  }

  public Command getPositionCmd(){
    return this.runOnce(() -> getHopperPosition());
  }

  /* * * BEAM BREAK * * */
  public boolean getReceiverStatus() {
    return receiver.get();
    // true = unbroken
    // false = broken
  }

  // hopper is empty once state change count == 2;
  // @Log.NT
  public boolean isHopperEmpty() {
    // int temp;

    if (hasStateChanged()) {
      stateCount++;
      System.out.println(stateCount);
    }

    // temp = stateCount;
    // resetStateCount();
    return stateCount == stateLimit;
  }

  // @Log.NT
  public boolean isHopperFull() {
    // if(stateCount>=2) {
    //   resetStateCount();
    // }
    return !getReceiverStatus();
  }

  public void resetStateCount() {
    if(stateCount >= stateLimit) {
      stateCount = 0;
    }
    // System.out.println("hopper state count reset");
  }

  public void resetStateEmergency() {
    stateCount = 0;
  }

  public void setStateLimit(int limit) {
    stateLimit = limit;
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

  public void setVelocity(double setpoint) {
    setVelocitySetpoint(setpoint);
    setVelocity();
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

  public double getHopperPosition(){
    return hopperEncoder.getPosition();
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
    SmartDashboard.putNumber("state count", stateCount);
    SmartDashboard.putNumber("hopper vel", getVelocity());
    SmartDashboard.putNumber("hopper sp", vSetpoint);

    SmartDashboard.putBoolean("isHopperFull", isHopperFull());
    SmartDashboard.putBoolean("isHopperEmpty", isHopperEmpty());
    SmartDashboard.putBoolean("beam break", getReceiverStatus());
  }
}
