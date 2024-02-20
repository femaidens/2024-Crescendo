// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Ports.ShooterPorts;

public class ShooterAngle extends SubsystemBase {
  private final CANSparkMax shooterAngleMotor;

  private final AbsoluteEncoder shooterAngleEncoder;

  private final PIDController shooterAnglePID;

  private double pSetpoint;

  // possibly add an armFF later

  public ShooterAngle() {
    shooterAngleMotor = new CANSparkMax(ShooterPorts.SHOOTER_ANGLE, MotorType.kBrushless);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake); // check with engineering
    shooterAngleMotor.setSmartCurrentLimit(ShooterAngleConstants.CURRENT_LIMIT);

    shooterAngleEncoder = shooterAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterAngleEncoder.setPositionConversionFactor(ShooterAngleConstants.POS_CFACTOR);

    shooterAnglePID = new PIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD);
  }

  /*
   * @param the angle setpoint - angle is relative to the horizontal, in degrees
   * sets the shooter angle, in degrees. Subtracted 18.3, because zero is 18.3
   * degrees above horizontal
   */
  public void setAutoAngle(double angle) {
    // add physical angle offset
    double voltage = shooterAnglePID.calculate(shooterAngleEncoder.getPosition(), angle - 18.3);
                                                                                               
    shooterAngleMotor.setVoltage(voltage);
  }

  // auto moves the shooter up, increases angle relative to horizontal axis
  public void angleUp() {

    if (shooterAngleEncoder.getPosition() < ShooterAngleConstants.SHOOTER_MAX_ANGLE) {
      shooterAngleMotor.set(0.5);
    } else {
      shooterAngleMotor.set(0);
      System.out.println("max angle reached!");
    }

  }

  // auto moves the shooter down, decreases angle relative to the horizontal
  public void angleDown() {

    if (shooterAngleEncoder.getPosition() > ShooterAngleConstants.SHOOTER_MIN_ANGLE) {
      shooterAngleMotor.set(-0.5);
    } else {
      shooterAngleMotor.set(0);
      System.out.println("min angle reached!");
    }

  }

  /*
   * @param input joystick axis
   * changes shooter angle
   * accounts for the max and min angle limits
   */
  public void setAngle(double input) {
    double speed = input * 0.5;

    if (input < -0.1 && shooterAngleEncoder.getPosition() > ShooterAngleConstants.SHOOTER_MIN_ANGLE) {
      shooterAngleMotor.set(speed);
    }
    else if (input > 0.1 && shooterAngleEncoder.getPosition() < ShooterAngleConstants.SHOOTER_MAX_ANGLE) {
      shooterAngleMotor.set(speed);
    }
    else {
      maintainAngle();
    }

    pSetpoint = shooterAngleEncoder.getPosition();
  }

  // maintains the angle that it was last set to with the joystick axis
  public void maintainAngle() {
    double voltage = shooterAnglePID.calculate(shooterAngleEncoder.getPosition(), pSetpoint);
    shooterAngleMotor.setVoltage(voltage);
  }

  /*
   * @param the angle to compare the encoder reading with
   * @return if the angle of the shooter is within the threshold of the setpoint
   */
  public boolean isAtAngle(double angle) {
    return Math.abs(angle - 18.3 - shooterAngleEncoder.getPosition()) < 2;
  }

  public double getAngle() {
    return shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET;
  }
  /*
   * stops motor for shooter angle
   */
  public void stopAngle() {
    shooterAngleMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm angle: ", shooterAngleEncoder.getPosition() + 18.3);
  }
}
