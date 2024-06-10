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
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  /**
   * Feeds the note from the hopper to the shooter, until it is shot out
   * @param vel Velocity of the hopper, in degrees per second
   * @return Sequential Wait and Proxy Commands
   */
  public Command feedNote(double vel) {
    return Commands.waitUntil(() -> isHopperFull())
    .andThen(setVelocitySetpointCmd(vel))
    .andThen(Commands.waitUntil(() -> isHopperEmpty()))
    .andThen(setVelocitySetpointCmd(0));
  }

  /**
   * Sets the fractional duty cycle of the hopper
   * @param speed PWM values, positive is intake, negative is outtake
   * @return Run Command
   */
  public Command setSpeedCmd(double speed) {
    return this.run(() -> setSpeed(speed));
  }

  /**
   * Runs the velocity of the hopper, based on setpoint in the subsystem. In degrees per second
   * @return Run Command
   */
  public Command setVelocityCmd() {
    return this.run(() -> setVelocity());
  }

  /**
   * Runs the velocity of the hopper
   * @param setpoint Velocity of the hopper in degrees per second
   * @return Sequential RunOnce, then Run Command
   */
  public Command setVelocityCmd(double setpoint) {
    return setVelocitySetpointCmd(setpoint)
    .andThen(setVelocityCmd());
  }

  /**
   * Sets the setpoint for hopper velocity
   * @param setpoint Desired velocity in degrees per second
   * @return Proxy Command
   */
  public Command setVelocitySetpointCmd(double setpoint) {
    return this.runOnce(() -> vSetpoint = setpoint).asProxy();
  }

  /**
   * Sets the setpoint for hopper velocity
   * @param setpoint Desired velocity in degrees per second
   * @return RunOnce Command
   */
  public Command autonSetVelocitySetpointCmd(double setpoint) {
    // return Commands.print("setting hopper vel setpoint");
    return this.runOnce(() -> vSetpoint = setpoint);
  }

  /**
   * Sets the state limit for the beam break, as proxy
   * @param limit Either 1 - speaker, or 2 - amp
   * @return Proxy Command
   */
  public Command setStateLimitCmd(int limit) {
    return this.runOnce(() -> stateLimit = limit).asProxy();
  }

  /**
   * Not a proxy, sets the state limit for the beam break
   * @param limit Either 1 - speaker, or 2 - amp
   * @return RunOnce Command
   */
  public Command autonSetStateLimitCmd(int limit) {
    return this.runOnce(() -> stateLimit = limit);
  }

  /**
   * Reset the state count, if state count is greater than the threshold
   * @return Proxy Command
   */
  public Command resetStateCountCmd() {
    if(stateCount >= stateLimit) {
      return this.runOnce(() -> stateCount = 0).asProxy();
    } else {
      return new PrintCommand("State count is still under threshold");
    }
  }

  /**
   * Not a proxy, reset the state count, if state count is greater than the threshold
   * @return RunOnce Command
   */
  public Command autonResetStateCountCmd() {
    System.out.println("state count reset");
    if(stateCount >= stateLimit) {
      return this.runOnce(() -> stateCount = 0);
    } else {
      return new PrintCommand("State count is still under threshold");
    }
  }

  public Command forceResetStateCmd() {
    return this.runOnce(() ->  stateCount = 0);
  }

  public Command autonForceResetStateCmd() {
    return this.runOnce(() ->  stateCount = 0);
  }

  /* METHODS */
  /**
   * Sets fractional duty cycle
   * @param speed PWM value, negative is outtake, positive is intake
   */
  public void setSpeed(double speed) {
    hopperMotor.set(speed);
  }

  /**
   * Runs the velocity of the hopper, based on the setpoint in the subsystem. In degrees per second
   */
  public void setVelocity(){
    double voltage = hopperFF.calculate(vSetpoint);
    hopperMotor.setVoltage(voltage);
  }

  /**
   * Returns the velocity of the hopper, in degrees per second
   * @return double, in degrees per second
   */
  public double getVelocity() {
    return hopperEncoder.getVelocity();
  }


  /* * * BEAM BREAK * * */
   
  /**
   * Return the receiver status
   * @return Boolean, true = unbroken; false = broken
   */
  public boolean getReceiverStatus() {
    return receiver.get();
  }

  
  // @Log.NT
  /**
   * Returns if the hopper is empty, i.e. if state count is equal to 2 (amp) or 1 (speaker)
   * @return Boolean
   */
  public boolean isHopperEmpty() {
    if (hasStateChanged()) {
      stateCount++;
      System.out.println(stateCount);
    }
    return stateCount == stateLimit;
  }

  // @Log.NT
  /**
   * Returns if the hopper is full, using receiver status
   * @return Boolean
   */
  public boolean isHopperFull() {
    return !getReceiverStatus();
  }

  // checks if beam break has changed from broken to unbroken
  /**
   * Returns if the receiver status has changed
   * @return Boolean
   */
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