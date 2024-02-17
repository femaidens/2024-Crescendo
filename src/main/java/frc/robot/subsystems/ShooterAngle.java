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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports.ShooterPorts;

public class ShooterAngle extends SubsystemBase {
  private final CANSparkMax shooterAngleMotor;

  private final AbsoluteEncoder shooterAngleEncoder;

  private final PIDController shooterAnglePID;

  private double setpoint;

  // possibly add an armFF later

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {
    shooterAngleMotor = new CANSparkMax(ShooterPorts.SHOOTER_ANGLE_PORT, MotorType.kBrushless);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake); // check with engineering
    shooterAngleMotor.setSmartCurrentLimit(ShooterAngleConstants.SHOOTER_ANGLE_CURRENT_LIMIT);

    shooterAngleEncoder = shooterAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterAngleEncoder.setPositionConversionFactor(ShooterAngleConstants.POSITION_CONVERSION_FACTOR);

    shooterAnglePID = new PIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD);

  }

  /*
   * @param the angle setpoint, in degrees
   * sets the shooter angle, in degrees
   */
  public void setShooterAutoAngle(double angle) {
    double voltage = shooterAnglePID.calculate(shooterAngleEncoder.getPosition(), angle);
    shooterAngleMotor.setVoltage(voltage);

  }

  /*
   * auto moves the shooter up, increases angle relative to horizontal axis
   */
  public void shooterAngleUp() {
    shooterAngleMotor.setVoltage(0.5);
  }

  /*
   * auto moves the shooter down, decreases angle relative to the horizontal
   */
  public void shooterAngleDown() {
    shooterAngleMotor.setVoltage(-0.5);
  }

  /*
   * joystick axis changes shooter angle
   */
  public void setShooterAngle(double input) {
    if (Math.abs(input) > 0) {
      double voltage = input * 0.7;
      shooterAngleMotor.setVoltage(voltage);
      setpoint = shooterAngleEncoder.getPosition();
    } else {
      maintainAngle();
    }

  }

  /*
   * maintains the angle that it was last set to
   */
  public void maintainAngle() {
    double voltage = shooterAnglePID.calculate(shooterAngleEncoder.getPosition(), setpoint);
    shooterAngleMotor.setVoltage(voltage);
  }

  /*
   * @param the angle to compare the encoder reading with
   * 
   * @return if the angle of the shooter is within the threshold of the setpoint
   */
  public boolean isAtAngle(double angle) {
    return Math.abs(angle - shooterAngleEncoder.getPosition()) < 2;
  }

  /*
   * stops motor for shooter angle
   */
  public void stopShooterAngle() {
    shooterAngleMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
