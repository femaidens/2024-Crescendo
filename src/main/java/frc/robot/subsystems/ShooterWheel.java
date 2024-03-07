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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.Ports.ShooterPorts;
import monologue.Logged;
import monologue.Annotations.Log;
import edu.wpi.first.units.Units;

public class ShooterWheel extends SubsystemBase implements Logged {
  private final CANSparkMax leaderMotor; // left motor
  private final CANSparkMax followerMotor; // right motor

  // private final CANSparkFlex leaderFlex;
  // private final CANSparkFlex followerFlex;

  private final RelativeEncoder leaderEncoder;
  private final RelativeEncoder followerEncoder;

  private final SimpleMotorFeedforward shooterWheel;

  private final PIDController shooterWheelPID;

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
    leaderMotor = new CANSparkMax(ShooterPorts.LEADER_MOTOR, MotorType.kBrushless);
    followerMotor = new CANSparkMax(ShooterPorts.FOLLOWER_MOTOR, MotorType.kBrushless);

    leaderEncoder = leaderMotor.getEncoder();
    followerEncoder = followerMotor.getEncoder();

    // controls
    shooterWheel = new SimpleMotorFeedforward(ShooterWheelConstants.kS, ShooterWheelConstants.kV);
    shooterWheelPID = new PIDController(ShooterWheelConstants.kP, ShooterWheelConstants.kI, ShooterWheelConstants.kD);

    shooterWheelPID.setTolerance(ShooterWheelConstants.V_TOLERANCE);
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
    // leaderFlex = new CANSparkFlex(ShooterPorts.LEADER_FLEX, MotorType.kBrushless);
    // followerFlex = new CANSparkFlex(ShooterPorts.FOLLOWER_FLEX, MotorType.kBrushless);

    // followerFlex.follow(leaderFlex, true);

    // leaderFlex.setIdleMode(IdleMode.kCoast);
    // followerFlex.setIdleMode(IdleMode.kCoast);

    // leaderFlex.setSmartCurrentLimit(ShooterWheelConstants.CURRENT_LIMIT);
    // followerFlex.setSmartCurrentLimit(ShooterWheelConstants.CURRENT_LIMIT);

    // leaderEncoder = leaderFlex.getEncoder();
    // followerEncoder = followerFlex.getEncoder();

    // leaderEncoder.setVelocityConversionFactor(ShooterWheelConstants.VEL_CFACTOR);
    // followerEncoder.setVelocityConversionFactor(ShooterWheelConstants.VEL_CFACTOR);
  }

  /* COMMANDS */
  public Command setVelocitySetpointCmd(double setpoint) {
    System.out.println("set wheel velocity setpoint");
    return this.runOnce(() -> setVelocitySetpoint(setpoint)).asProxy();
  }

  public Command stopMotorsCmd() {
    System.out.println("stopping wheel motors");
    return this.runOnce(() -> stopMotors());
  }

  // default command
  public Command setVelocityCmd(double angle) {
    System.out.println("setting wheel velocity with angle");
    return this.run(() -> setVelocity(angle));
  }

  // default command
  public Command setVelocityCmd() {
    System.out.println("setting wheel velocity w/o angle");
    return this.run(() -> setVelocity());
  }

  // sets fractional duty cycle
  public void setSpeed(double speed) {
    leaderMotor.set(speed);
  }

  public void setVelocity(double setpoint) {
    setVelocitySetpoint(setpoint);
    setVelocity();
  }

  // sets the velocity of shooter wheels in degrees per second
  public void setVelocity() {
    double ff = shooterWheel.calculate(vSetpoint);
    double error = shooterWheelPID.calculate(getLeaderVelocity(), vSetpoint);

    leaderMotor.setVoltage(ff + error);
    // System.out.println("shooter wheel voltage: " + (ff + error));
  }

  public void setVelocitySetpoint(double setpoint) {
    vSetpoint = setpoint;
  }

  // checks if current velocity is within error margin of vSetpoint
  public boolean atVelocity() {
    // System.out.println("shooter wheel at velocity");
    return shooterWheelPID.atSetpoint();
  }

  @Log.NT
  public double getSetpoint() {
    return shooterWheelPID.getSetpoint();
  }

  // returns the velocities of shooter motors
  public double getLeaderVelocity() {
    return leaderEncoder.getVelocity();
  }

  public double getFollowerVelocity() {
    return followerEncoder.getVelocity();
  }

  // stops the motors for the shooter wheels
  public void stopMotors() {
    leaderMotor.setVoltage(0);
    // leaderFlex.setVoltage(0);
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
    SmartDashboard.putNumber("left shooter vel: ", getLeaderVelocity());
    SmartDashboard.putNumber("right shooter vel: ", getFollowerVelocity());
    SmartDashboard.putNumber("desired shooter velocity: ", vSetpoint);
    
    SmartDashboard.putBoolean("at shooterVelocity", atVelocity());
    // SmartDashboard.putNumber("current shooter wheel voltage: ", leaderMotor.getOut());
  }
}