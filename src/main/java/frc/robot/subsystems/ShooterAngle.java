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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports.ShooterPorts;

public class ShooterAngle extends SubsystemBase {
  private final CANSparkMax angleMotor;

  private final AbsoluteEncoder angleEncoder;

  private final PIDController shooterAnglePID;

  private double setpoint;

  // possibly add an armFF later

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {
    angleMotor = new CANSparkMax(ShooterPorts.SHOOTER_ANGLE_PORT, MotorType.kBrushless);
    angleMotor.setIdleMode(IdleMode.kBrake); // check with engineering
    angleMotor.setSmartCurrentLimit(ShooterAngleConstants.SHOOTER_ANGLE_CURRENT_LIMIT);

    angleEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    angleEncoder.setPositionConversionFactor(ShooterAngleConstants.POSITION_CONVERSION_FACTOR);

    shooterAnglePID = new PIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD);

  }

  /*
   * @param the angle setpoint - angle is relative to the horizontal, in degrees
   * sets the shooter angle, in degrees. Subtracted 18.3, because zero is 18.3 degrees above horizontal
   */
  public void setAutoAngle(double angle) {
    double voltage = shooterAnglePID.calculate(angleEncoder.getPosition(), angle - 18.3); //offset of 18.3 degrees, is subtracted
    angleMotor.setVoltage(voltage);

  }

  /*
   * auto moves the shooter up, increases angle relative to horizontal axis
   */
  public void angleUp() {
    if(angleEncoder.getPosition() < ShooterAngleConstants.SHOOTER_MAX_ANGLE){
      angleMotor.set(0.5);
    } else {
      angleMotor.set(0);
      System.out.println("max angle reached!");
    }
    
  }

  /*
   * auto moves the shooter down, decreases angle relative to the horizontal
   */
  public void angleDown() {
    if(angleEncoder.getPosition() > ShooterAngleConstants.SHOOTER_MIN_ANGLE){
      angleMotor.set(-0.5);
    } else {
      angleMotor.set(0);
      System.out.println("min angle reached!");
    }
   
  }

  /*
   * @param input joystick axis
   * changes shooter angle
   * accounts for the max and min angle limits
   */
  public void setAngle(double input) { 
      double speed = input * 0.5;
      if(input < -0.1 && angleEncoder.getPosition() > ShooterAngleConstants.SHOOTER_MIN_ANGLE){
        angleMotor.set(speed);
      } else if (input > 0.1 && angleEncoder.getPosition() < ShooterAngleConstants.SHOOTER_MAX_ANGLE){
        angleMotor.set(speed);
      } else {
        maintainAngle();
      }
     setpoint = angleEncoder.getPosition();
    }
  

  /*
   * maintains the angle that it was last set to with the joystick axis
   */
  public void maintainAngle() {
    double voltage = shooterAnglePID.calculate(angleEncoder.getPosition(), setpoint);
    angleMotor.setVoltage(voltage);
  }

  /*
   * @param the angle to compare the encoder reading with
   * 
   * @return if the angle of the shooter is within the threshold of the setpoint
   */
  public boolean isAtAngle(double angle) {
    return Math.abs(angle - 18.3 - angleEncoder.getPosition()) < 2;
  }

  /*
   * stops motor for shooter angle
   */
  public void stopAngle() {
    angleMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm angle: ", angleEncoder.getPosition() + 18.3);
  }
}
