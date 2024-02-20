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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.Ports.ShooterPorts;
import static edu.wpi.first.units.Units.Volts;

public class ShooterWheel extends SubsystemBase {
  private final CANSparkMax leftShooterMotor;
  private final CANSparkMax rightShooterMotor;

  // private final CANSparkFlex leftShooterFlex;
  // private final CANSparkFlex rightShooterFlex;

  private final RelativeEncoder leftShooterEncoder;
  private final RelativeEncoder rightShooterEncoder;

  private final SimpleMotorFeedforward shooterFF;

  private final PIDController shooterPID;

  private final SysIdRoutine leftRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          volts -> setLeftVoltage(volts.in(Volts)), null, this));

  private final SysIdRoutine rightRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          volts -> setRightVoltage(volts.in(Volts)), null, this));

  /** Creates a new Shooter. */
  public ShooterWheel() {
    leftShooterMotor = new CANSparkMax(ShooterPorts.LEFT_SHOOTER, MotorType.kBrushless);
    rightShooterMotor = new CANSparkMax(ShooterPorts.RIGHT_SHOOTER, MotorType.kBrushless);

    leftShooterEncoder = leftShooterMotor.getEncoder();
    rightShooterEncoder = rightShooterMotor.getEncoder();

    // controls
    shooterFF = new SimpleMotorFeedforward(ShooterWheelConstants.kS, ShooterWheelConstants.kV);
    shooterPID = new PIDController(ShooterWheelConstants.kP, ShooterWheelConstants.kI, ShooterWheelConstants.kD);

    rightShooterMotor.setIdleMode(IdleMode.kCoast); // double check w/ engineering later
    leftShooterMotor.setIdleMode(IdleMode.kCoast);

    rightShooterMotor.follow(leftShooterMotor, true);

    rightShooterMotor.setSmartCurrentLimit(ShooterWheelConstants.CURRENT_LIMIT);
    leftShooterMotor.setSmartCurrentLimit(ShooterWheelConstants.CURRENT_LIMIT);

    leftShooterEncoder.setVelocityConversionFactor(ShooterWheelConstants.VEL_CFACTOR);
    rightShooterEncoder.setVelocityConversionFactor(ShooterWheelConstants.VEL_CFACTOR);

    leftShooterMotor.burnFlash();
    rightShooterMotor.burnFlash();

    // rightShooterMotor.setInverted(true); //double check if it's left or right

    /* FLEX VARIATIONS */
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
    double error = shooterPID.calculate(getLeftVelocity(), speed);

    leftShooterMotor.setVoltage(voltage + error);
    System.out.println(voltage + error);

  }

  // @return the velocities of shooter motors
  public double getLeftVelocity() {
    return leftShooterEncoder.getVelocity();
  }

  public double getRightVelocity() {
    return rightShooterEncoder.getVelocity();
  }

  // stops the motors for the shooter wheels
  public void stopShooter() {
    leftShooterMotor.setVoltage(0);
    // leftShooterFlex.setVoltage(0);
  }

  /* SYSID */ 
  public void setLeftVoltage(double voltage) {
    leftShooterMotor.setVoltage(voltage);
  }

  public void setRightVoltage(double voltage) {
    rightShooterMotor.setVoltage(voltage);
  }

  public Command leftQuas(SysIdRoutine.Direction direction) {
    return leftRoutine.quasistatic(direction);
  }

  public Command leftDyna(SysIdRoutine.Direction direction) {
    return leftRoutine.dynamic(direction);
  }

  public Command rightQuas(SysIdRoutine.Direction direction) {
    return rightRoutine.quasistatic(direction);
  }

  public Command rightDyna(SysIdRoutine.Direction direction) {
    return rightRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left shooter vel", getLeftVelocity());
    SmartDashboard.putNumber("right shooter vel", getRightVelocity());
  }
}