// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Ports.HopperPorts;

public class Intake extends SubsystemBase {
  //intake
  private static CANSparkMax intakeMotor;
  private static SparkAbsoluteEncoder encoder;
  private static PIDController velocityPID;
  private static RelativeEncoder intakeEncoder;
  private static SimpleMotorFeedforward ff;

  //hopper
  // private DigitalInput intakeBeamReceiver;
  // private DigitalOutput intakeBeamEmitter;
  // private final DigitalInput shooterBeamReceiver;
  // private final DigitalOutput shooterBeamEmitter;
  private final CANSparkMax hopperMotor;

  public Intake() {
    //intake
    intakeMotor = new CANSparkMax(Ports.IntakePorts.rollerNEOPort, MotorType.kBrushless);
    velocityPID = new PIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI, Constants.IntakeConstants.kD);
    intakeEncoder = intakeMotor.getEncoder();
    ff = new SimpleMotorFeedforward(Constants.IntakeConstants.kS, Constants.IntakeConstants.kV);
    intakeEncoder.setVelocityConversionFactor(Constants.IntakeConstants.VELOCITY_CONVERSION_FACTOR);


    //hopper
    // intakeBeamReceiver = new DigitalInput(HopperPorts.BEAM_INTAKE_RECEIVER_PORT);
    // // intakeBeamEmitter = new DigitalOutput(HopperPorts.BEAM_INTAKE_EMITTER_PORT);
    // shooterBeamReceiver = new DigitalInput(HopperPorts.BEAM_SHOOTER_RECEIVER_PORT);
    // shooterBeamEmitter = new DigitalOutput(HopperPorts.BEAM_SHOOTER_EMITTER_PORT);
    hopperMotor = new CANSparkMax(HopperPorts.HOPPER_MOTOR_PORT, MotorType.kBrushless);
    hopperMotor.setIdleMode(IdleMode.kBrake);
    hopperMotor.setSmartCurrentLimit(HopperConstants.HOPPER_CURRENT_LIMIT);
  }

  //enableBeam, disableBeam, getBeamStatus
  //intake methods
  public void setRollerSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public double getAbsoluteEncoderAngle() {
    return encoder.getPosition();
  }

  public void velocityPID(double setpoint){
    double voltage = ff.calculate(setpoint);
    double error = velocityPID.calculate(intakeEncoder.getVelocity(), setpoint);

    intakeMotor.setVoltage(error + voltage);
  }

  //hopper or beam break methods
  public void stopHopperMotor() {
    hopperMotor.setVoltage(0);
  }

  public void stopIntakeMotor() {
    intakeMotor.setVoltage(0);
  }

  // public void disableBeam() {
  //   intakeBeamEmitter.close();
  //   intakeBeamReceiver.close();
  // }

  // public void enableBeam() {
  //   intakeBeamReceiver = new DigitalInput(HopperPorts.BEAM_INTAKE_RECEIVER_PORT);
  //   intakeBeamEmitter = new DigitalOutput(HopperPorts.BEAM_INTAKE_EMITTER_PORT);
  // }

  // public boolean getStatus() {
  //  if( intakeBeamEmitter.get() && intakeBeamReceiver.get())
  //  {
  //   System.out.println("beam break uninterrupted");
  //   return true;
  //  }
  //   System.out.println("beam break interrupted");
  //   return false;
  // }

  // public void checkBeamBreakIntake() {
  //   if (intakeBeamReceiver.get() && intakeBeamEmitter.get()) {
  //     hopperMotor.set(0.5);
  //     // retract intake + stop intake motors
  //   } else {
  //     stopIntakeMotor();
  //     System.out.println("beam break intake sensor activated \n");
  //   }
  // }

  // public void checkBeamBreakShooter() {
  //   if (!shooterBeamEmitter.get() && !shooterBeamReceiver.get()) {
  //     hopperMotor.set(0.5);
  //   } else {
  //     stopHopperMotor();
  //     System.out.println("beam break shooter sensor activated \n");
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}