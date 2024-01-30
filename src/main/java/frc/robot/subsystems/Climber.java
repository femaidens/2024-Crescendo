// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private static CANSparkMax armRight;
  private static CANSparkMax armLeft;
  private static DigitalInput topSwitch;// are these static?
  private static DigitalInput botSwitch;

  public Climber() 
  {
    armRight = new CANSparkMax(Ports.ClimbPorts.armRightMotor, MotorType.kBrushless);
    armLeft = new CANSparkMax(Ports.ClimbPorts.armLeftMotor, MotorType.kBrushless);
  
    topSwitch = new DigitalInput(Ports.ClimbPorts.TOP_SWITCH_PORT);
    botSwitch = new DigitalInput(Ports.ClimbPorts.BOTTOM_SWITCH_PORT);
  }

  public static void extendClimbArm()
  {
    if(!topSwitch.get()){
      armRight.set(Constants.ClimberConstants.climbArmSpeed);
      armLeft.set(-Constants.ClimberConstants.climbArmSpeed);
      System.out.println("extending arm");
    }
    else
    {
      armRight.set(0);
      armLeft.set(0);
      System.out.println("top limit switch hit!");
    }
    
  }

  public static void retractClimbArm()
  {
    if(!botSwitch.get()){
      armRight.set(-Constants.ClimberConstants.climbArmSpeed);
      armLeft.set(Constants.ClimberConstants.climbArmSpeed);
      System.out.println("retracting arm");
      
    }
    else
    {
      armRight.set(0);
      armLeft.set(0);
      System.out.println("bottom switch activated!");
    }
    
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
