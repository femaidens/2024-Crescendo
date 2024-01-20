// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private static CANSparkMax armRight;
  private static CANSparkMax armLeft;

  public Climber() 
  {
    armRight = new CANSparkMax(Ports.ClimbPorts.armRightMotor, MotorType.kBrushless);
    armLeft = new CANSparkMax(Ports.ClimbPorts.armLeftMotor, MotorType.kBrushless);
  }

  public static void extendClimbArm()
  {
    armRight.set(Constants.ClimberConstants.climbArmSpeed);
    armLeft.set(-Constants.ClimberConstants.climbArmSpeed);
  }

  public static void retractClimbArm()
  {
    armRight.set(-Constants.ClimberConstants.climbArmSpeed);
    armLeft.set(Constants.ClimberConstants.climbArmSpeed);
  }

  public static void stopClimb()
  {
    armRight.set(0);
    armLeft.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
