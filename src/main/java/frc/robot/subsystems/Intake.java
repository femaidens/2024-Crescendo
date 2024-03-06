// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
// import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.Ports.*;
import monologue.Logged;
import monologue.Annotations.Log;

public class Intake extends SubsystemBase implements Logged {

  @Log.NT
  private final CANSparkMax intakeMotor;

  @Log.NT
  private final RelativeEncoder intakeEncoder;

  // private final PIDController intakePID;
  private final SimpleMotorFeedforward intakeFF;

  private final SysIdRoutine intakeRoutine;

  @Log.NT
  private double vSetpoint;

  public Intake() {

    intakeMotor = new CANSparkMax(IntakePorts.INTAKE_ROLLER, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();

    intakeEncoder.setVelocityConversionFactor(IntakeConstants.VEL_CFACTOR);

    intakeFF = new SimpleMotorFeedforward(
        IntakeConstants.kS,
        IntakeConstants.kV);

    // intakePID = new PIDController(
    //     IntakeConstants.kP,
    //     IntakeConstants.kI,
    //     IntakeConstants.kD);

    intakeMotor.setIdleMode(IdleMode.kCoast); // should freely spin?
    intakeMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    intakeMotor.burnFlash();

    intakeRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts -> setVoltage(volts.in(Units.Volts)), null, this));

    vSetpoint = 0;
  }

  /* COMMANDS */
  
  // default command DO NOT ADD AS PROXY
  public Command setVelocityCmd() {
    return this.run(() -> setVelocity());
  }

  public Command setSpeedCmd(double speed) {
    // return Commands.print("setting intake speed");
    return this.runOnce(() -> setSpeed(speed));
  }

  public Command setVelocitySetpointCmd(double setpoint) {
    // return Commands.print("setting intake vel setpoint");
    return this.runOnce(() -> setVelocitySetpoint(setpoint)).asProxy();
  }

  public Command setVelocityCmd(double setpoint) {
    return this.run(() -> setVelocity(setpoint));
  }

  public Command stopMotorCmd() {
    // return Commands.print("stopping intake motors");
    return this.runOnce(() -> stopMotor());
  }

  // sets fractional duty cycle
  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setVelocity(double setpoint) {
    setVelocitySetpoint(setpoint);
    setVelocity();
  }

  public void setVelocity() {
    double voltage = intakeFF.calculate(vSetpoint);
    // double error = intakePID.calculate(intakeEncoder.getVelocity(), vSetpoint);

    intakeMotor.setVoltage(voltage);
  }

  public void setVelocitySetpoint(double setpoint) {
    vSetpoint = setpoint;
  }

  public double getVelocity() {
    return intakeEncoder.getVelocity();
  }

  public void stopMotor() {
    intakeMotor.stopMotor();
  }

  /* SYSID */
  public void setVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public Command intakeQuas(SysIdRoutine.Direction direction) {
    return intakeRoutine.quasistatic(direction);
  }

  public Command intakeDyna(SysIdRoutine.Direction direction) {
    return intakeRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake vel", getVelocity());
    SmartDashboard.putNumber("intake sp", vSetpoint);
  }
}
