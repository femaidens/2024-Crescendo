// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports.ShooterPorts;

public class Shooter extends SubsystemBase {
  private final CANSparkMax leftShooterMotor;
  private final CANSparkMax rightShooterMotor;

  // private final CANSparkFlex leftShooterFlex;
  // private final CANSparkFlex rightShooterFlex;

  private final RelativeEncoder leftShooterEncoder;
  private final RelativeEncoder rightShooterEncoder;

  private final SimpleMotorFeedforward shooterFF;

  private final PIDController shooterPID;
  /** Creates a new Shooter. */
  public Shooter() {
    leftShooterMotor = new CANSparkMax(ShooterPorts.LEFT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(ShooterPorts.RIGHT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);

    rightShooterMotor.setIdleMode(IdleMode.kCoast); //double check w/ engineering later
    leftShooterMotor.setIdleMode(IdleMode.kCoast);

    rightShooterMotor.follow(leftShooterMotor, true);

    rightShooterMotor.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    leftShooterMotor.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);

    leftShooterEncoder = leftShooterMotor.getEncoder();
    rightShooterEncoder = rightShooterMotor.getEncoder();

    leftShooterEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
    rightShooterEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);


    shooterFF = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);

    shooterPID = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

    // //rightShooterMotor.setInverted(true); //double check if it's left or right

    // leftShooterFlex = new CANSparkFlex(ShooterPorts.LEFT_SHOOTER_FLEX_PORT, MotorType.kBrushless);
    // rightShooterFlex = new CANSparkFlex(ShooterPorts.RIGHT_SHOOTER_FLEX_PORT, MotorType.kBrushless);

    // rightShooterFlex.follow(leftShooterFlex, true);

    // leftShooterFlex.setIdleMode(IdleMode.kCoast);
    // rightShooterFlex.setIdleMode(IdleMode.kCoast);

    // leftShooterFlex.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    // rightShooterFlex.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);

    // leftShooterEncoder = leftShooterFlex.getEncoder();
    // rightShooterEncoder = rightShooterFlex.getEncoder();

    // leftShooterFlex.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
    // rightShooterFlex.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
  }

  public void setDesiredVelocity(double speed){
    /*
    * speed in meters/second
    * getRate() in WPI might be better than getVelocity if conversion in Constants doesn't work
    * might need to switch inversion to left side
    */
    double voltage = shooterFF.calculate(speed);
    double error = shooterPID.calculate(leftShooterEncoder.getVelocity(), speed);

    leftShooterMotor.setVoltage(voltage + error);
    System.out.println(voltage + error);

  }


  public void stopShooter(){
    leftShooterMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
