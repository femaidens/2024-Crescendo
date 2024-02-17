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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Ports.ShooterPorts;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {
  //eft and right relative to robot direction
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  // private final CANSparkFlex leftShooterFlex;
  // private final CANSparkFlex rightShooterFlex;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final SimpleMotorFeedforward shooterFF;

  private final PIDController shooterPID;

  private final SysIdRoutine leftRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
      volts -> runLeftShooter(volts.in(Volts)), null, this));

  private final SysIdRoutine rightRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      volts -> runRightShooter(volts.in(Volts)), null, this));
  

  /** Creates a new Shooter. */
  public Shooter() {
    leftMotor = new CANSparkMax(ShooterPorts.LEFT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(ShooterPorts.RIGHT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);

    rightMotor.setIdleMode(IdleMode.kCoast); // double check w/ engineering later
    leftMotor.setIdleMode(IdleMode.kCoast);

    rightMotor.follow(leftMotor, true);

    rightMotor.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    leftMotor.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);
    rightEncoder.setVelocityConversionFactor(ShooterConstants.VELOCITY_CONVERSION_FACTOR);

    shooterFF = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV);

    shooterPID = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

    // //rightShooterMotor.setInverted(true); //double check if it's left or right

    // leftShooterFlex = new CANSparkFlex(ShooterPorts.LEFT_SHOOTER_FLEX_PORT,
    // MotorType.kBrushless);
    // rightShooterFlex = new CANSparkFlex(ShooterPorts.RIGHT_SHOOTER_FLEX_PORT,
    // MotorType.kBrushless);

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

  /*
   * @param the setpoint velocity of the wheels, in meters/second
   * sets the velocity of shooter wheels in meters per second
   * 
   * the inversion of the motors might need to be switched
   */
  public void setDesiredVelocity(double speed) {
    // getRate() in WPI might be better than getVelocity if conversion in Constants
    // doesn't work
    double voltage = shooterFF.calculate(speed);
    double error = shooterPID.calculate(leftEncoder.getVelocity(), speed);

    leftMotor.setVoltage(voltage + error);
    System.out.println(voltage + error);

  }

  /*
   * stops the motors for the shooter wheels
   */
  public void stopShooter() {
    leftMotor.setVoltage(0);
    // leftShooterFlex.setVoltage(0);
  }

  /*
   * @return the velocity of the shooter
   */
  public double getShooterSpeed() {
    return leftEncoder.getVelocity();
  }

  /*
   * sysids
   */
  public void runLeftShooter(double voltage){
    leftMotor.setVoltage(voltage);
  }

  public void runRightShooter(double voltage){
    rightMotor.setVoltage(voltage);
  }
  
  public Command leftQuas(SysIdRoutine.Direction direction){
    return leftRoutine.quasistatic(direction);
  }

  public Command leftDyna(SysIdRoutine.Direction direction){
    return leftRoutine.dynamic(direction);
  }

  public Command rightQuas(SysIdRoutine.Direction direction){
    return rightRoutine.quasistatic(direction);
  }

  public Command rightDyna(SysIdRoutine.Direction direction){
    return rightRoutine.dynamic(direction);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("Left: " + leftEncoder.getVelocity());
    System.out.println("Right: " + rightEncoder.getVelocity());
  }
}
