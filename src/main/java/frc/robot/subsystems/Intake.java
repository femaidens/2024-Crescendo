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
  private final CANSparkMax hopperMotor;

  private final RelativeEncoder intakeEncoder;
  private final RelativeEncoder hopperEncoder;

  private final PIDController intakePID;
  private final SimpleMotorFeedforward ff;

  private final DigitalInput receiver;

  private final SysIdRoutine intakeRoutine;

  private double vSetpoint;
  private boolean currentState, lastState;
  private int stateCount = 0;


  public Intake() {
    intakeMotor = new CANSparkMax(IntakePorts.INTAKE_ROLLER, MotorType.kBrushless);
    hopperMotor = new CANSparkMax(HopperPorts.HOPPER_MOTOR, MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    hopperEncoder = hopperMotor.getEncoder();

    intakeEncoder.setVelocityConversionFactor(IntakeConstants.VEL_CFACTOR);
    hopperEncoder.setVelocityConversionFactor(HopperConstants.VEL_CFACTOR);

    intakePID = new PIDController(
        IntakeConstants.kP,
        IntakeConstants.kI,
        IntakeConstants.kD);
    ff = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV);

    receiver = new DigitalInput(HopperPorts.RECEIVER);

    hopperMotor.setIdleMode(IdleMode.kBrake); // prevent note from slipping out of hopper
    intakeMotor.setIdleMode(IdleMode.kCoast); // should freely spin?

    hopperMotor.setSmartCurrentLimit(HopperConstants.CURRENT_LIMIT);
    intakeMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);

    intakeMotor.burnFlash();
    hopperMotor.burnFlash();

    intakeRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts -> setVoltage(volts.in(Units.Volts)), null, this));

    vSetpoint = 0;
    currentState = getReceiverStatus();
    lastState = currentState;

  }

  public void setVelocity() {
    double voltage = ff.calculate(vSetpoint);
    double error = intakePID.calculate(intakeEncoder.getVelocity(), vSetpoint);

    intakeMotor.setVoltage(error + voltage);
  }

  public void setVelocitySetpoint(double setpoint) {
    vSetpoint = setpoint;
  }

  // for testing; see if vel pid is absolutely necessary
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  // for transition between hopper and shooter wheels
  public void setHopperSpeed(double speed) {
    hopperMotor.set(speed);
  }

  public double getIntakeVelocity() {
    return intakeEncoder.getVelocity();
  }

  public double getHopperVelocity() {
    return hopperEncoder.getVelocity();
  }

  // beam breaker code
  public boolean getReceiverStatus() {
    return receiver.get();
    // true = unbroken
    // false = broken
  }

  // hopper is empty once state change count == 2;
  public boolean isHopperEmpty() {
    if(hasStateChanged()) {
      stateCount++;
      System.out.println(stateCount);
    }
    return stateCount == 2;
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

  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
  }

  public void stopHopperMotor() {
    hopperMotor.stopMotor();
  }

  /* COMMANDS */
  public Command SetIntakeSpeed(double speed) {
    return this.runOnce(() -> setIntakeSpeed(speed));
  }

  public Command ResetStateCount() {
    return this.runOnce(() -> resetStateCount());
  }

  public Command SetHopperVelocity() {
    return this.run(() -> setVelocity());
  }

  public Command stopHopperMotorCommand() {
    return this.runOnce(() -> stopHopperMotor());
  }

  /* SYSID */

  public void setVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public Command leftQuas(SysIdRoutine.Direction direction) {
    return intakeRoutine.quasistatic(direction);
  }

  public Command leftDyna(SysIdRoutine.Direction direction) {
    return intakeRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake vel", getIntakeVelocity());
    SmartDashboard.putNumber("hopper vel", getHopperVelocity());

    SmartDashboard.putNumber("intake sp", vSetpoint);
    // System.out.println("hopper velocity: " + getHopperVelocity());
  }
}
