// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static CANSparkMax intakeMotor;
  private static RelativeEncoder intakeEncoder;
  private static PIDController intakePID;

  private static boolean isRunning;
  private double vSetpoint;

  public Intake() {
    //rotationNEO = new CANSparkMax(Ports.IntakePorts.rotationNEOPort, MotorType.kBrushless);
    intakeMotor = new CANSparkMax(Ports.IntakePorts.rollerNEOPort, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder(); 
    intakePID = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

    isRunning = false;
    vSetpoint = 0;
  }

  // public void setRotationSpeed(double speed)
  // {
  //   rotationNEO.set(speed);
  //   if( speed == 0)
  //   {
  //     isRunning = false;
  //   }
  //   else
  //   {
  //     isRunning = true;
  //   }
  // }

  public void setRollerSpeed(double speed)
  {
    intakeMotor.set(speed);
  }

  // public void stopRotation()
  // {
  //   rotationNEO.set(0);
  // }

  public double getAbsoluteEncoderAngle()
  {
    return intakeEncoder.getPosition();
  }

  // public void liftIntake()
  // {
  //   double intakeAngleVoltage = intakePIDController.calculate(getAbsoluteEncoderAngle(), Constants.IntakeConstants.rotLiftSetPoint);
  //   rotationNEO.setVoltage(intakeAngleVoltage);
  //   System.out.println("Running Intake Lift Rotation PID");
  // }

  // public void lowerIntake()
  // {
  //   double intakeAngleVoltage = intakePIDController.calculate(getAbsoluteEncoderAngle(), Constants.IntakeConstants.rotLowerSetPoint);
  //   rotationNEO.setVoltage(intakeAngleVoltage);
  //   System.out.println("Running Intake Lower Rotation PID");
  // }

  public boolean getRollerStatus()
  {
    return isRunning;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}