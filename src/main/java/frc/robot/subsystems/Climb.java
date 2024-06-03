// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  /**
   * Extends arm until bottom limit switch is activated
   * 
   */
  public Command extendClimbArm() {
    if(isBotActivated()){ // stop extending if bottom hits top
      System.out.println("bottom limit switch hit!");
      return this.run(() -> stopMotors())
      .alongWith(new PrintCommand("Stopped Arm"));
    }

    else {
      return this.run(() -> setSpeed(ClimberConstants.ARM_SPEED))
      .alongWith(new PrintCommand("Extending Arm"));
    }
  }

  public void retractClimbArm() {
    if(isTopActivated()){ // stop retracting if top hits bottom
      this.run(() -> stopMotors());
      System.out.println("top switch activated!");
    }

    else {
      // check inversion of motors
      System.out.println("retracting arm");
      this.run(() -> setSpeed(-ClimberConstants.ARM_SPEED));
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

  /**
   * Sets the speed of both climb motors
   * @param speed PWM value
   */
  public void setSpeed(double speed){  //positive up, negative down
    rightArm.set(speed);
    leftArm.set(-speed);
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("top limit switch", isTopActivated());
    SmartDashboard.putBoolean("bot limit switch", isBotActivated());
  }
}