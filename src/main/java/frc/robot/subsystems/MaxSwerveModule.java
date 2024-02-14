// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.Consumer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.DrivetrainConstants;


public class MaxSwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final PIDController m_drivingPIDController;
  private final PIDController m_turningPIDController;

  private final SimpleMotorFeedforward driveFF;
  private final SimpleMotorFeedforward turnFF;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MaxSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = new PIDController(
      DrivetrainConstants.ModuleConstants.kDrivingP, 
      DrivetrainConstants.ModuleConstants.kDrivingI,
      DrivetrainConstants.ModuleConstants.kDrivingD
      );
    m_turningPIDController = new PIDController(
      DrivetrainConstants.ModuleConstants.kDrivingP, 
      DrivetrainConstants.ModuleConstants.kDrivingI,
      DrivetrainConstants.ModuleConstants.kDrivingD
      );
    // m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    // m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    driveFF = new SimpleMotorFeedforward(
      DrivetrainConstants.ModuleConstants.kDrivingFFkS, 
      DrivetrainConstants.ModuleConstants.kDrivingFFkV,
      DrivetrainConstants.ModuleConstants.kDrivingFFkA
      );
    
    turnFF = new SimpleMotorFeedforward(
      DrivetrainConstants.ModuleConstants.kTurningFFkS, 
      DrivetrainConstants.ModuleConstants.kTurningFFkV,
      DrivetrainConstants.ModuleConstants.kTurningFFkA
      );

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(DrivetrainConstants.ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(DrivetrainConstants.ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(DrivetrainConstants.ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(DrivetrainConstants.ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(DrivetrainConstants.ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    // m_turningPIDController.setPositionPIDWrappingEnabled(true);
    // m_turningPIDController.setPositionPIDWrappingMinInput(DrivetrainConstants.ModuleConstants.kTurningEncoderPositionPIDMinInput);
    // m_turningPIDController.setPositionPIDWrappingMaxInput(DrivetrainConstants.ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(DrivetrainConstants.ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(DrivetrainConstants.ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(DrivetrainConstants.ModuleConstants.kDrivingD);
    // m_drivingPIDController.setOutputRange(DrivetrainConstants.ModuleConstants.kDrivingMinOutput,
    //     DrivetrainConstants.ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(DrivetrainConstants.ModuleConstants.kTurningP);
    m_turningPIDController.setI(DrivetrainConstants.ModuleConstants.kTurningI);
    m_turningPIDController.setD(DrivetrainConstants.ModuleConstants.kTurningD);
    // m_turningPIDController.setOutputRange(DrivetrainConstants.ModuleConstants.kTurningMinOutput,
    //     DrivetrainConstants.ModuleConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(DrivetrainConstants.ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(DrivetrainConstants.ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(DrivetrainConstants.ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(DrivetrainConstants.ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
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
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
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
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setSetpoint(optimizedDesiredState.speedMetersPerSecond);
    m_turningPIDController.setSetpoint(optimizedDesiredState.angle.getRadians());

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public void setDriveSpeed(double speed) {
    double driveFFCalculate = driveFF.calculate(speed);
    double driveVoltage = driveFFCalculate + m_drivingPIDController.calculate(speed);
    m_drivingSparkMax.set(driveVoltage);
    System.out.println(driveVoltage);
  }

  public void setTurnSpeed(double speed) {
    double turnFFCalculate = turnFF.calculate(speed);
    double turnVoltage = turnFFCalculate + m_drivingPIDController.calculate(speed);
    m_drivingSparkMax.set(turnVoltage);
    System.out.println(turnVoltage);
  }

  public void setDriveVoltage(double voltage) {
    m_drivingSparkMax.setVoltage(voltage);
  }

  public void setTurnVoltage(double voltage) {
    m_turningSparkMax.setVoltage(voltage);
  }
}