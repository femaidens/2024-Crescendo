// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Ports.ShooterPorts;
import monologue.Annotations.Log;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import monologue.Logged;

public class ShooterAngle extends SubsystemBase implements Logged {
  private final CANSparkMax shooterAngleMotor;

  private final AbsoluteEncoder shooterAngleEncoder;

  @Log.NT
  private final PIDController shooterAnglePID;

  private double pSetpoint;

  // private final MutableMeasure<Voltage> rampRate = MutableMeasure.mutable(Units.Volts.of(0));

  // private final SysIdRoutine.Config config = new SysIdRoutine.Config(
  //   Volts.of(3)
  // );

   private final SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.5).per(Seconds.of(1)),
            Volts.of(3),
            Seconds.of(5),
            null);

  private final SysIdRoutine angleRoutine = new SysIdRoutine(
      config,
      new SysIdRoutine.Mechanism(
          volts -> setVoltage(volts.in(Volts)), null, this));

  // possibly add an armFF later

  public ShooterAngle() {
    shooterAngleMotor = new CANSparkMax(ShooterPorts.SHOOTER_ANGLE, MotorType.kBrushless);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake); // check with engineering
    shooterAngleMotor.setSmartCurrentLimit(ShooterAngleConstants.CURRENT_LIMIT);

    shooterAngleEncoder = shooterAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterAngleEncoder.setPositionConversionFactor(ShooterAngleConstants.POS_CFACTOR);

    shooterAnglePID = new PIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD);

    shooterAngleMotor.burnFlash();

    shooterAnglePID.setTolerance(ShooterAngleConstants.P_TOLERANCE);

    pSetpoint = ShooterAngleConstants.INITIAL_ANGLE;
  }

  /* COMMANDS */
  // default commands
  public Command setManualAngleCmd(double input) {
    // return Commands.print("setting manual angle setpoint");
    return this.run(() -> setManualAngle(input));
  }

  public Command setAngleSetpointCmd(double angle) {
    // return Commands.print("set regular angle setpoint");
    return this.runOnce(() -> setAngleSetpoint(angle)).asProxy();
  }

  public Command setAngleCmd(double angle) {
    return this.run(() -> setAngle(angle)).asProxy();
  }

  public Command setAngleCmd() {
    // return Commands.print("running angle pid");
    return this.run(() -> setAngle());
  }

  // sets shooter angle based on joystick input
  // accounts for the max and min angle limits
  public void setManualAngle(double input) {

    // move up if below max angle
    if (input > 0 && getAngle() < ShooterAngleConstants.SHOOTER_MAX_ANGLE) {
      shooterAngleMotor.set(ShooterAngleConstants.CONSTANT_SPEED);
      pSetpoint = getAngle();
    }
    // move down if above min angle
    else if (input < 0 && getAngle() > ShooterAngleConstants.SHOOTER_MIN_ANGLE) {
      shooterAngleMotor.set(-ShooterAngleConstants.CONSTANT_SPEED);
      pSetpoint = getAngle();
    }
    // run PID
    else {
      setAngle();
      // stopMotor();
    }
  }

  // sets shooter angle to current setpoint
  public void setAngle() {
    double voltage = shooterAnglePID.calculate(getAngle(), pSetpoint);
    shooterAngleMotor.setVoltage(voltage);

    // System.out.println("angle voltage: " + voltage);
    // System.out.println("setting angle");
  }

  // for auton commands; overloads setAngle no params
  public void setAngle(double setpoint) {
    setAngleSetpoint(setpoint);
    setAngle();
  }

  // changes setpoint accordingly
  public void setAngleSetpoint(double setpoint) {
    // isManual = false;
    pSetpoint = setpoint;
    System.out.println("shooter angle changed");
  }

  @Log.NT
  public double getSetpoint() {
    return shooterAnglePID.getSetpoint();
  }

  @Log.NT
  // added physical offset lowest angle is 18.3 deg above the horizontal
  public double getAngle() {
    return shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET;
  }

  // public boolean getIsManual() {
  // return isManual;
  // }

  public void stopMotor() {
    shooterAngleMotor.stopMotor();
  }

  public boolean atAngle() {

    return shooterAnglePID.atSetpoint();
  }

  /* SYSID */
  public void setVoltage(double voltage) {
    shooterAngleMotor.setVoltage(voltage);
  }

  public boolean atMaxAngle(){
    return (shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET) > ShooterAngleConstants.SHOOTER_MAX_ANGLE;
  }

  public boolean atMinAngle(){
    return (shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET) < ShooterAngleConstants.SHOOTER_MIN_ANGLE;
  }

  public Command quasiCmd(SysIdRoutine.Direction direction) {
    return angleRoutine.quasistatic(direction);
  }

  public Command dynaCmd(SysIdRoutine.Direction direction) {
    return angleRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("current arm angle", getAngle());
    SmartDashboard.putNumber("desired angle", pSetpoint);

    SmartDashboard.putBoolean("at min angle", atAngle());
    SmartDashboard.putBoolean("at amp angle", atAngle());
  }
}
