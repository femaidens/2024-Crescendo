// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Ports.*;
import monologue.Logged;

public class Climb extends SubsystemBase implements Logged {

  /** Creates a new Climb. */
  private CANSparkMax rightArm;
  private CANSparkMax leftArm;
  private DigitalInput topSwitch;
  private DigitalInput botSwitch;

  public Climb() {
    rightArm = new CANSparkMax(ClimbPorts.RIGHT_ARM_MOTOR, MotorType.kBrushless);
    leftArm = new CANSparkMax(ClimbPorts.LEFT_ARM_MOTOR, MotorType.kBrushless);
  
    topSwitch = new DigitalInput(ClimbPorts.TOP_SWITCH);
    botSwitch = new DigitalInput(ClimbPorts.BOTTOM_SWITCH);
  }

  /* COMMANDS */
  public Command extendClimbCmd() {
    return this.run(() -> extendClimbArm());
  }

  public Command retractClimbCmd() {
    return this.run(() -> retractClimbArm());
  }

  public Command stopMotorsCmd() {
    return this.runOnce(() -> stopMotors());
  }

  public void extendClimbArm() {
    if(isBotActivated()){ // stop extending if bottom hits top
      rightArm.set(0);
      leftArm.set(0);
      System.out.println("bottom limit switch hit!");
    }

    else {
      rightArm.set(ClimberConstants.ARM_SPEED);
      leftArm.set(-ClimberConstants.ARM_SPEED);
      System.out.println("extending arm");
    }
  }

  public void retractClimbArm() {
    if(isTopActivated()){ // stop retracting if top hits bottom
      rightArm.set(0);
      leftArm.set(0);
      System.out.println("top switch activated!");
    }

    else {
      // check inversion of motors
      rightArm.set(-ClimberConstants.ARM_SPEED);
      leftArm.set(ClimberConstants.ARM_SPEED);
      System.out.println("retracting arm");
    }
    
  }

  public boolean isTopActivated() {
    return topSwitch.get(); // check to see if it needs to be negated
  }

  public boolean isBotActivated() {
    return !botSwitch.get(); // check to see if it needs to be negated
  }

  public void stopMotors() {
    rightArm.set(0);
    leftArm.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("top limit switch", isTopActivated());
    SmartDashboard.putBoolean("bot limit switch", isBotActivated());
  }
}