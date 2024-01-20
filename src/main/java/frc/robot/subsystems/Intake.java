// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private static CANSparkMax rotationNEO;
  private static CANSparkMax rollerNEO;
  private static SparkAbsoluteEncoder encoder;

  public Intake() {
    rotationNEO = new CANSparkMax(Ports.IntakePorts.rotationNEOPort, MotorType.kBrushless);
    rollerNEO = new CANSparkMax(Ports.IntakePorts.rollerNEOPort, MotorType.kBrushless);
    encoder = rotationNEO.getAbsoluteEncoder(Type.kDutyCycle); 
  }

  public void setRotationSpeed()
  {
    rotationNEO.set(Constants.IntakeConstants.rotationSpeed);
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

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
