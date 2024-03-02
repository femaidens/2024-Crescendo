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

  private double pSetpoint;

  private boolean isManual = true;

  // possibly add an armFF later

  public ShooterAngle() {
    shooterAngleMotor = new CANSparkMax(ShooterPorts.SHOOTER_ANGLE, MotorType.kBrushless);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake); // check with engineering
    shooterAngleMotor.setSmartCurrentLimit(ShooterAngleConstants.CURRENT_LIMIT);

    shooterAngleEncoder = shooterAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterAngleEncoder.setPositionConversionFactor(ShooterAngleConstants.POS_CFACTOR);

    shooterAnglePID = new PIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD);

    shooterAngleMotor.burnFlash();
    
    pSetpoint = shooterAngleEncoder.getPosition();
  }

  // sets shooter angle based on joystick input
  // accounts for the max and min angle limits
  public void setManualAngle(double input) { 
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
    isManual = true;

    if (input > 0.1) {
      shooterAngleMotor.set(ShooterAngleConstants.CONSTANT_SPEED);
      setAngleSetpoint(getAngle());
    }
    // move down if above min angle
    else if (input < -0.1) {
      shooterAngleMotor.set(-ShooterAngleConstants.CONSTANT_SPEED);
      setAngleSetpoint(getAngle());
    }
    // run PID
    else {
      setAngle();
      // stopMotor();
    }

    
  }

  // sets shooter angle to current setpoint
  public void setAngle() {
    // if(getAngle() <= 20){
    //   shooterAngleMotor.setVoltage(0);
    // }
    // else{
      double voltage = shooterAnglePID.calculate(getAngle(), pSetpoint);
      shooterAngleMotor.setVoltage(voltage);
       System.out.println("angle voltage: " + voltage);
    // }
    System.out.println("setting angle");
  }

  // changes setpoint accordingly
  public void setAngleSetpoint(double setpoint) {
    isManual = false;
    pSetpoint = setpoint;
    System.out.println("setpoint changed");
  }

  // added physical offset lowest angle is 18.3 deg above the horizontal
  public double getAngle() {
    return shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET;
  }

  public void stopMotor() {
    shooterAngleMotor.stopMotor();
  }

  // @param the angle to compare the encoder reading with
  // @return if the angle of the shooter is within the threshold of the setpoint
  public boolean isAtAngle(double angle) {
    return Math.abs(angle - getAngle()) < 2;
  }

  /* COMMANDS */
  public Command SetShooterAngle(double angle) {
    return this.runOnce(() -> setAngleSetpoint(angle));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("current arm angle", getAngle());
    SmartDashboard.putNumber("desired angle", pSetpoint);
  }
}
