// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Ports.ShooterPorts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAngle extends SubsystemBase {
  private final CANSparkMax shooterAngleMotor;

  private final AbsoluteEncoder shooterAngleEncoder;

  private final PIDController shooterAnglePID;

  // private boolean isManual = true;
  private double pSetpoint;

  // possibly add an armFF later

  public ShooterAngle() {
    shooterAngleMotor = new CANSparkMax(ShooterPorts.SHOOTER_ANGLE, MotorType.kBrushless);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake); // check with engineering
    shooterAngleMotor.setSmartCurrentLimit(ShooterAngleConstants.CURRENT_LIMIT);

    shooterAngleEncoder = shooterAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterAngleEncoder.setPositionConversionFactor(ShooterAngleConstants.POS_CFACTOR);

    shooterAnglePID = new PIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD);

    shooterAngleMotor.burnFlash();

    pSetpoint = getAngle();
  }

  // sets shooter angle based on joystick input
  // accounts for the max and min angle limits
  public void setManualAngle(double input) {
    // isManual = true;
    /*
    // move up if below max angle
    if (input > 0 && getAngle() < ShooterAngleConstants.SHOOTER_MAX_ANGLE) {
      shooterAngleMotor.set(ShooterAngleConstants.CONSTANT_SPEED);
    }
    // move down if above min angle
    else if (input < 0 && getAngle() > ShooterAngleConstants.SHOOTER_MIN_ANGLE) {
      shooterAngleMotor.set(-ShooterAngleConstants.CONSTANT_SPEED);
    }
    // run PID
    else {
      // setAngle();
      stopMotor();
    }
*/
    if (input > 0) {
      shooterAngleMotor.set(ShooterAngleConstants.CONSTANT_SPEED);
      // pSetpoint = getAngle();
    }
    // move down if above min angle
    else if (input < 0) {
      shooterAngleMotor.set(-ShooterAngleConstants.CONSTANT_SPEED);
      // pSetpoint = getAngle();
    }
    // run PID
    pSetpoint = getAngle();
    setAngle();
    // else {
    //   // setAngle();
    //   stopMotor();
    // }

  }

  // sets shooter angle to current setpoint
  public void setAngle() {
    double voltage = shooterAnglePID.calculate(getAngle(), pSetpoint);
    shooterAngleMotor.setVoltage(voltage);
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
  }

  // added physical offset lowest angle is 18.3 deg above the horizontal
  public double getAngle() {
    return shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET;
  }

  // public boolean getIsManual() {
  //   return isManual;
  // }

  public void stopMotor() {
    shooterAngleMotor.stopMotor();
  }

  public boolean atAngle(double angle) {
    return Math.abs(angle - getAngle()) < ShooterAngleConstants.ERROR_MARGIN;
  }

  /* COMMANDS */
  public Command SetAngleSetpointCmd(double angle) {
    return this.runOnce(() -> setAngleSetpoint(angle));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm angle", getAngle());
  }
}
