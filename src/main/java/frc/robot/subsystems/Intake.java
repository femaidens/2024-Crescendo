// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static CANSparkMax rotationNEO;
  private static CANSparkMax rollerNEO;
  private static SparkAbsoluteEncoder encoder;
  private static PIDController intakePIDController;
  //private double setpoint;

  public Intake() {
    rotationNEO = new CANSparkMax(Ports.IntakePorts.rotationNEOPort, MotorType.kBrushless);
    rollerNEO = new CANSparkMax(Ports.IntakePorts.rollerNEOPort, MotorType.kBrushless);
    encoder = rotationNEO.getAbsoluteEncoder(Type.kDutyCycle); 
    intakePIDController = new PIDController(Constants.IntakeConstants.PIDConstants.kP, Constants.IntakeConstants.PIDConstants.kI, Constants.IntakeConstants.PIDConstants.kD);
    //setpoint = encoder.getPosition();
  }

  public void setRotationSpeed(double speed)
  {
    rotationNEO.set(speed);
  }

  public void setRollerSpeed()
  {
    rollerNEO.set(Constants.IntakeConstants.rollerSpeed);
  }

  public void stopRotation()
  {
    rotationNEO.set(0);
  }

  public void stopRoller()
  {
    rollerNEO.set(0);
  }

  public double getAbsoluteEncoderAngle()
  {
    return encoder.getPosition();
  }
  public void liftIntake()
  {
      double intakeAngleVoltage = intakePIDController.calculate(getAbsoluteEncoderAngle(), Constants.IntakeConstants.rotLiftSetPoint);
      rotationNEO.setVoltage(intakeAngleVoltage);
      System.out.println("Running Intake Lift Rotation PID");
  }

  public void lowerIntake()
  {
     double intakeAngleVoltage = intakePIDController.calculate(getAbsoluteEncoderAngle(), Constants.IntakeConstants.rotLowerSetPoint);
      rotationNEO.setVoltage(intakeAngleVoltage);
      System.out.println("Running Intake Lower Rotation PID");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
