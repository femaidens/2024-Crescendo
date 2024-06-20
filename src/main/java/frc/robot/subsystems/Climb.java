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
import edu.wpi.first.wpilibj2.command.ProxyCommand;
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
  /**
   * Extends arm until bottom limit switch is activated
   * @return Command
   */
  public Command extendClimbCmd() {
    if(isBotActivated()){ // stop extending if bottom hits top
      return this.run(() -> setSpeed(0)).alongWith(new PrintCommand("bottom limit switch hit!"));
    }
    else {
      return this.run(() -> setSpeed(ClimberConstants.ARM_SPEED)).alongWith(new PrintCommand("extending arm"));
    }
  }

  /**
   * Retracts arm until top limit switch is activated
   * @return Command
   */
  public Command retractClimbCmd() {
    return new ProxyCommand(() -> {
      if(isTopActivated()){ // stop retracting if top hits bottom
        return this.run(() -> setSpeed(0)).alongWith(new PrintCommand("top switch activated!"));
      }
      else {
        // check inversion of motors
        return this.run(() -> setSpeed(-ClimberConstants.ARM_SPEED)).alongWith(new PrintCommand("retracting arm"));
      }
    });
  }

  /**
   * Stops both climb motors
   * @return Command 
   */
  public Command stopClimbCmd() {
    return this.run(() -> setSpeed(0));
  }

  /**
   * Sets the speed of both climb motors
   * @param speed PWM value, positive is extend, negative is retract
   */
  public void setSpeed(double speed){
    rightArm.set(speed);
    leftArm.set(-speed);
  }

  public boolean isTopActivated() {
    return topSwitch.get(); // check to see if it needs to be negated
  }

  public boolean isBotActivated() {
    return !botSwitch.get(); // check to see if it needs to be negated
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("top limit switch", isTopActivated());
    SmartDashboard.putBoolean("bot limit switch", isBotActivated());
  }
}