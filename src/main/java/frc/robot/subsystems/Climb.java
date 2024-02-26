// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Ports.*;

public class Climb extends SubsystemBase {
  /** Creates a new Climber. */

  private CANSparkMax rightArm;
  private CANSparkMax leftArm;
  private DigitalInput topSwitch;// are these?
  private DigitalInput botSwitch;

  public Climb() {
    rightArm = new CANSparkMax(ClimbPorts.RIGHT_ARM_MOTOR, MotorType.kBrushless);
    leftArm = new CANSparkMax(ClimbPorts.LEFT_ARM_MOTOR, MotorType.kBrushless);
  
    topSwitch = new DigitalInput(ClimbPorts.TOP_SWITCH);
    botSwitch = new DigitalInput(ClimbPorts.BOTTOM_SWITCH);
  }

  public void extendClimbArm() {
    if(isTopActivated()){ // hits limit switch
      rightArm.set(0);
      leftArm.set(0);
      System.out.println("top limit switch hit!");
    }

    else {
      rightArm.set(ClimberConstants.ARM_SPEED);
      leftArm.set(-ClimberConstants.ARM_SPEED);
      System.out.println("extending arm");
    }
  }

  public void retractClimbArm() {
    if(isBotActivated()){
      rightArm.set(0);
      leftArm.set(0);
      System.out.println("bottom switch activated!");
    }

    else {
      // check inversion of motors
      rightArm.set(-ClimberConstants.ARM_SPEED);
      leftArm.set(ClimberConstants.ARM_SPEED);
      System.out.println("retracting arm");
    }
    
  }

  public boolean isTopActivated() {
    return !topSwitch.get();
  }

  public boolean isBotActivated() {
    return !botSwitch.get();
  }

  public void stopClimb() {
    rightArm.set(0);
    leftArm.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
