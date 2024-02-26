// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.Ports.ShooterPorts;
import edu.wpi.first.units.Units;

public class ShooterWheel extends SubsystemBase {
  private final CANSparkMax leaderMotor; // left motor
  private final CANSparkMax followerMotor; // right motor

  // private final CANSparkFlex leftShooterFlex;
  // private final CANSparkFlex rightShooterFlex;

  private final RelativeEncoder leaderEncoder;
  private final RelativeEncoder followerEncoder;

  private final SimpleMotorFeedforward shooterFF;

  private final PIDController shooterPID;

  private double vSetpoint;

  private final SysIdRoutine leftRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          volts -> setLeftVoltage(volts.in(Units.Volts)), null, this));

  private final SysIdRoutine rightRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          volts -> setRightVoltage(volts.in(Units.Volts)), null, this));

  public ShooterWheel() {
    leaderMotor = new CANSparkMax(ShooterPorts.LEFT_SHOOTER, MotorType.kBrushless);
    followerMotor = new CANSparkMax(ShooterPorts.RIGHT_SHOOTER, MotorType.kBrushless);

    leaderEncoder = leaderMotor.getEncoder();
    followerEncoder = followerMotor.getEncoder();

    // controls
    shooterFF = new SimpleMotorFeedforward(ShooterWheelConstants.kS, ShooterWheelConstants.kV);
    shooterPID = new PIDController(ShooterWheelConstants.kP, ShooterWheelConstants.kI, ShooterWheelConstants.kD);

    followerMotor.follow(leaderMotor, true);

    followerMotor.setIdleMode(IdleMode.kCoast); // double check w/ engineering later
    leaderMotor.setIdleMode(IdleMode.kCoast);

    followerMotor.setSmartCurrentLimit(ShooterWheelConstants.CURRENT_LIMIT);
    leaderMotor.setSmartCurrentLimit(ShooterWheelConstants.CURRENT_LIMIT);

    leaderEncoder.setVelocityConversionFactor(ShooterWheelConstants.VEL_CFACTOR);
    followerEncoder.setVelocityConversionFactor(ShooterWheelConstants.VEL_CFACTOR);

    leaderMotor.burnFlash();
    followerMotor.burnFlash();

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

  // sets the velocity of shooter wheels in degrees per second
  public void setVelocity() {
    // getRate() in WPI might be better than getVelocity if conversion in Constants
    // doesn't work
    double ff = shooterFF.calculate(vSetpoint);
    double error = shooterPID.calculate(getLeaderVelocity(), vSetpoint);

    leaderMotor.setVoltage(ff + error);
    System.out.println("wheel voltage" + (ff + error));
  }

  public void setVelocitySetpoint(double setpoint) {
    vSetpoint = setpoint;
  }

  // @return the velocities of shooter motors
  public double getLeaderVelocity() {
    return leaderEncoder.getVelocity();
  }

  public double getFollowerVelocity() {
    return followerEncoder.getVelocity();
  }

  public void setShooterSpeed(double speed) {
    leaderMotor.set(speed);
  }
  // stops the motors for the shooter wheels
  public void stopShooter() {
    leaderMotor.setVoltage(0);
    // leftShooterFlex.setVoltage(0);
  }
  /* COMMANDS */
  public Command SetShooterSpeed(double speed) {
    return this.runOnce(() -> setVelocitySetpoint(speed));
  }

  /* SYSID */
  public void setLeftVoltage(double voltage) {
    leaderMotor.setVoltage(voltage);
  }

  public void setRightVoltage(double voltage) {
    followerMotor.setVoltage(voltage);
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
    SmartDashboard.putNumber("left shooter vel", getLeaderVelocity());
    SmartDashboard.putNumber("right shooter vel", getFollowerVelocity());
  }
}