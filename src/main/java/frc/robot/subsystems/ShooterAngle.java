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

  //possibly add an armFF later

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {
    shooterAngleMotor = new CANSparkMax(ShooterPorts.SHOOTER_ANGLE_PORT, MotorType.kBrushless);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake); //check with engineering
    shooterAngleMotor.setSmartCurrentLimit(ShooterAngleConstants.SHOOTER_ANGLE_CURRENT_LIMIT);

    shooterAngleEncoder = shooterAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterAngleEncoder.setPositionConversionFactor(ShooterAngleConstants.POSITION_CONVERSION_FACTOR);
    

    shooterAnglePID = new PIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD);


  }

  public void setShooterAngle(double angle){
    double voltage = shooterAnglePID.calculate(shooterAngleEncoder.getPosition(), angle);
    shooterAngleMotor.setVoltage(voltage);
  }

  public void shooterAngleUp(){
    shooterAngleMotor.setVoltage(0.5);
  }

  public void shooterAngleDown(){
    shooterAngleMotor.setVoltage(-0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
