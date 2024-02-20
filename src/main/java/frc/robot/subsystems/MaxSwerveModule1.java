// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Consumer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import frc.robot.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.ModuleConstants;

public class MaxSwerveModule1 {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivePID;
  private final SparkPIDController turningPID;

  // private final SimpleMotorFeedforward driveFF;
  // private final SimpleMotorFeedforward turnFF;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MaxSwerveModule1(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    driveMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

    drivePID = driveMotor.getPIDController();
    turningPID = turningMotor.getPIDController();

    drivePID.setFeedbackDevice(driveEncoder);
    turningPID.setFeedbackDevice(turningEncoder);

    // driveFF = new SimpleMotorFeedforward(DrivetrainConstants.ModuleConstants.kDrivingFFkS,
    //     DrivetrainConstants.ModuleConstants.kDrivingFFkV, DrivetrainConstants.ModuleConstants.kDrivingFFkA);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEncoder.setPositionConversionFactor(DrivetrainConstants.ModuleConstants.kDrivingEncoderPositionFactor);
    driveEncoder.setVelocityConversionFactor(DrivetrainConstants.ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(DrivetrainConstants.ModuleConstants.kTurningEncoderPositionFactor);
    turningEncoder.setVelocityConversionFactor(DrivetrainConstants.ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(DrivetrainConstants.ModuleConstants.kTurningEncoderInverted);
    // drivePID = new PIDController(
    // DrivetrainConstants.ModuleConstants.kDrivingP,
    // DrivetrainConstants.ModuleConstants.kDrivingI,
    // DrivetrainConstants.ModuleConstants.kDrivingD
    // );
    // turningPID = new PIDController(
    // DrivetrainConstants.ModuleConstants.kDrivingP,
    // DrivetrainConstants.ModuleConstants.kDrivingI,
    // DrivetrainConstants.ModuleConstants.kDrivingD
    // );
    // driveFF = new SimpleMotorFeedforward(
    // DrivetrainConstants.ModuleConstants.kDrivingFFkS,
    // DrivetrainConstants.ModuleConstants.kDrivingFFkV,
    // DrivetrainConstants.ModuleConstants.kDrivingFFkA
    // );

    // turnFF = new SimpleMotorFeedforward(
    // DrivetrainConstants.ModuleConstants.kTurningFFkS,
    // DrivetrainConstants.ModuleConstants.kTurningFFkV,
    // DrivetrainConstants.ModuleConstants.kTurningFFkA
    // );

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPID.setPositionPIDWrappingEnabled(true);
    turningPID.setPositionPIDWrappingMinInput(DrivetrainConstants.ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turningPID.setPositionPIDWrappingMaxInput(DrivetrainConstants.ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    // drivePID.setOutputRange(DrivetrainConstants.ModuleConstants.kDrivingMinOutput,
    // DrivetrainConstants.ModuleConstants.kDrivingMaxOutput);

    drivePID.setP(ModuleConstants.kDrivingP);
    drivePID.setI(ModuleConstants.kDrivingI);
    drivePID.setD(ModuleConstants.kDrivingD);
    drivePID.setFF(ModuleConstants.kDrivingFFkS);
    drivePID.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    turningPID.setP(ModuleConstants.kTurningP);
    turningPID.setI(ModuleConstants.kTurningI);
    turningPID.setD(ModuleConstants.kTurningD);
    turningPID.setFF(ModuleConstants.kTurningFFkS);
    turningPID.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    // turningPID.setOutputRange(DrivetrainConstants.ModuleConstants.kTurningMinOutput,
    // DrivetrainConstants.ModuleConstants.kTurningMaxOutput);

    driveMotor.setIdleMode(DrivetrainConstants.ModuleConstants.kDrivingMotorIdleMode);
    turningMotor.setIdleMode(DrivetrainConstants.ModuleConstants.kTurningMotorIdleMode);
    driveMotor.setSmartCurrentLimit(DrivetrainConstants.ModuleConstants.kDrivingMotorCurrentLimit);
    turningMotor.setSmartCurrentLimit(DrivetrainConstants.ModuleConstants.kTurningMotorCurrentLimit);

    driveMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    turningMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    driveMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turningMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
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

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

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

  // public void setDriveVoltage(double voltage) {
  //   driveMotor.setVoltage(voltage);
  // }

  // public void setTurnVoltage(double voltage) {
  //   turningMotor.setVoltage(voltage);
  // }

  public void periodic() {
    double velocity = driveEncoder.getVelocity();

    SmartDashboard.putNumber("Current Velocity: ", velocity);
    SmartDashboard.putNumber("Target Velocity: ", desiredState.speedMetersPerSecond);

    SmartDashboard.putNumber("Current Angle: ", getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Target Angle; ", desiredState.angle.getDegrees());
  }
}