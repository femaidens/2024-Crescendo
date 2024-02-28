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
  // private final DigitalOutput emitter;

  // private final SysIdRoutine intakeRoutine;

  private double vSetpoint;

  public Intake() {
    intakeMotor =
      new CANSparkMax(IntakePorts.INTAKE_ROLLER, MotorType.kBrushless);
    hopperMotor =
      new CANSparkMax(HopperPorts.HOPPER_MOTOR, MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    hopperEncoder = hopperMotor.getEncoder();

    intakeEncoder.setVelocityConversionFactor(IntakeConstants.VEL_CFACTOR);
    hopperEncoder.setVelocityConversionFactor(HopperConstants.VEL_CFACTOR);

    intakePID =
      new PIDController(
        IntakeConstants.kP,
        IntakeConstants.kI,
        IntakeConstants.kD
      );
    ff = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV);

    receiver = new DigitalInput(HopperPorts.RECEIVER);
    // emitter = new DigitalOutput(HopperPorts.EMITTER);

    hopperMotor.setIdleMode(IdleMode.kBrake); // prevent note from slipping out of hopper
    intakeMotor.setIdleMode(IdleMode.kCoast); // should freely spin?

    hopperMotor.setSmartCurrentLimit(HopperConstants.CURRENT_LIMIT);
    intakeMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);

    intakeMotor.burnFlash();
    hopperMotor.burnFlash();

    // setEmitter(true);

    vSetpoint = 0;
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

  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
  }

  public void stopHopperMotor() {
    hopperMotor.stopMotor();
  }

  // beam breaker code
  public boolean getReceiverStatus() {
    return receiver.get();
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
    SmartDashboard.putNumber("hopper vel", getHopperVelocity());

    SmartDashboard.putNumber("intake sp", vSetpoint);
    // System.out.println("hopper velocity: " + getHopperVelocity());
  }
}
