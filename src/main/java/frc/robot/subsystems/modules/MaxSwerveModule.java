// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.modules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.DrivetrainConstants.*;
import frc.robot.DrivetrainConstants.ModuleConstants.*;

public class MaxSwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivePID;
  private final SparkPIDController turningPID;
  // private final PIDController drivePID;
  // private final PIDController turningPID;

  private final SimpleMotorFeedforward driveFF;
  private final SimpleMotorFeedforward turnFF;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public MaxSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    driveMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

    drivePID = driveMotor.getPIDController();
    turningPID = turningMotor.getPIDController();

    drivePID.setFeedbackDevice(driveEncoder);
    turningPID.setFeedbackDevice(turningEncoder);

    // driveFF = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);

    driveEncoder.setPositionConversionFactor(Drive.DRIVE_ENCODER_PFACTOR);
    driveEncoder.setVelocityConversionFactor(Drive.DRIVE_ENCODER_VFACTOR);

    turningEncoder.setPositionConversionFactor(Turning.ENCODER_PFACTOR);
    turningEncoder.setVelocityConversionFactor(Turning.ENCODER_VFACTOR);

    turningEncoder.setInverted(Turning.ENCODER_INVERTED);

    // drivePID = new PIDController(Drive.kP, Drive.kI, Drive.kD);
    // turningPID = new PIDController(Turning.kP, Turning.kI, Turning.kD);

    driveFF = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
    turnFF = new SimpleMotorFeedforward(Turning.kS, Turning.kV, Turning.kA);

    // rev version
    turningPID.setPositionPIDWrappingEnabled(true);
    turningPID.setPositionPIDWrappingMinInput(Turning.ENCODER_PPID_MIN);
    turningPID.setPositionPIDWrappingMaxInput(Turning.ENCODER_PPID_MAX);

    // // wpilib version
    // turningPID.enableContinuousInput(Turning.ENCODER_PPID_MIN, Turning.ENCODER_PPID_MAX);

    drivePID.setP(Drive.kP);
    drivePID.setI(Drive.kI);
    drivePID.setD(Drive.kD);
    drivePID.setFF(Drive.kFF);
    drivePID.setOutputRange(Drive.kMinOutput,
        Drive.kMaxOutput);

    turningPID.setP(Turning.kP);
    turningPID.setI(Turning.kI);
    turningPID.setD(Turning.kD);
    turningPID.setFF(Turning.kFF);
    turningPID.setOutputRange(Turning.kMinOutput,
        Turning.kMaxOutput);

    driveMotor.setIdleMode(ModuleConstants.kDriveMotorIdleMode);
    turningMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);

    driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    turningMotor.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);
    
    driveMotor.burnFlash();
    turningMotor.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /**
   * Sets the desired state for the module, without drive motor PID.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredStateNoPID(SwerveModuleState desiredState){
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    // drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    driveFF.calculate(optimizedDesiredState.speedMetersPerSecond);
    turningPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  /* SYSID */
  // public void setDriveSpeed(double speed) {
  //   double driveFFCalculate = driveFF.calculate(speed);
  //   double driveVoltage = driveFFCalculate + drivePID.calculate(speed);
  //   driveMotor.set(driveVoltage);
  //   System.out.println(driveVoltage);
  // }

  // public void setTurnSpeed(double speed) {
  //   double turnVoltage = drivePID.calculate(speed);
  //   driveMotor.set(turnVoltage);
  //   System.out.println(turnVoltage);
  // }

  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    turningMotor.setVoltage(voltage);
  }

  public void periodic() {
    double velocity = driveEncoder.getVelocity();

    SmartDashboard.putNumber("Current Velocity: ", velocity);
    SmartDashboard.putNumber("Target Velocity: ", desiredState.speedMetersPerSecond);

    SmartDashboard.putNumber("Current Angle: ", getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Target Angle; ", desiredState.angle.getDegrees());
  }
}