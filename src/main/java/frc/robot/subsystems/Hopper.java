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

public class Hopper extends SubsystemBase implements Logged {

  private final CANSparkMax hopperMotor;

  private final RelativeEncoder hopperEncoder;

  private final SimpleMotorFeedforward hopperFF;

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
  public Command feedNote(double vel) {
    return Commands.waitUntil(() -> isHopperFull())
    .andThen(setVelocitySetpointCmd(vel))
    .andThen(Commands.waitUntil(() -> isHopperEmpty()))
    .andThen(setVelocitySetpointCmd(0));
  }

  public Command setOuttakeSpeedCmd(double speed) {
    return this.run(() -> setSpeed(speed));
  }

  public Command setVelocityCmd() {
    double voltage = hopperFF.calculate(vSetpoint);
    return this.run(() -> hopperMotor.setVoltage(voltage));
  }

   public Command setVelocityCmd(double setpoint) {
    return setVelocitySetpointCmd(setpoint)
    .andThen(setVelocityCmd());
  }

  //not sure if this works
  public Command setVelocitySetpointCmd(double setpoint) {
    return this.runOnce(() -> vSetpoint = setpoint).asProxy();
  }

  public Command autonSetVelocitySetpointCmd(double setpoint) {
    // return Commands.print("setting hopper vel setpoint");
    return this.runOnce(() -> vSetpoint = setpoint);
  }

  public Command stopMotorCmd() {
    return this.runOnce(() -> setSpeed(0));
  }

  public Command setStateLimitCmd(int limit) {
    return this.runOnce(() -> setStateLimit(limit)).asProxy();
  }

  public Command autonSetStateLimitCmd(int limit) {
    return this.runOnce(() -> setStateLimit(limit));
  }

  public Command resetStateCountCmd() {
    System.out.println("state count reset");
    return this.runOnce(() -> resetStateCount()).asProxy();
  }

  public Command autonResetStateCountCmd() {
    System.out.println("state count reset");
    return this.runOnce(() -> resetStateCount());
  }

  public Command resetStateEmergencyCmd() {
    return this.runOnce(() -> resetStateEmergency()).asProxy();
  }

  public Command autonResetStateEmergencyCmd() {
    return this.runOnce(() -> resetStateEmergency());
  }

   // sets fractional duty cycle
  public void setSpeed(double speed) {
    hopperMotor.set(speed);
  }

  public double getVelocity() {
    return hopperEncoder.getVelocity();
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