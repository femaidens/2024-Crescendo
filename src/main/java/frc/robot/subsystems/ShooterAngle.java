// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Ports.ShooterPorts;
import monologue.Annotations.Log;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import monologue.Logged;

public class ShooterAngle extends SubsystemBase implements Logged {
  private final CANSparkMax shooterAngleMotor;

  private final AbsoluteEncoder shooterAngleEncoder;

  @Log.NT
  private final PIDController shooterAnglePID;
  
  @Log.NT
  private final ProfiledPIDController profiledShooterAnglePID;

  private final TrapezoidProfile.Constraints trapezoidProfile;

  private double pSetpoint;

  private final SysIdRoutine.Config config = new SysIdRoutine.Config(Volts.of(0.4).per(Seconds.of(1)),
      Volts.of(2),
      Seconds.of(5),
      null
  );

  private final SysIdRoutine angleRoutine = new SysIdRoutine(
    config,
    new SysIdRoutine.Mechanism(
        volts -> setVoltage(volts.in(Volts)), null, this)
  );


  private final ArmFeedforward shooterAngleFF;

  // possibly add an armFF later

  public ShooterAngle() {
    shooterAngleMotor = new CANSparkMax(ShooterPorts.SHOOTER_ANGLE, MotorType.kBrushless);
    shooterAngleMotor.setIdleMode(IdleMode.kBrake); // check with engineering
    shooterAngleMotor.setSmartCurrentLimit(ShooterAngleConstants.CURRENT_LIMIT);

    shooterAngleEncoder = shooterAngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shooterAngleEncoder.setPositionConversionFactor(ShooterAngleConstants.POS_CFACTOR);

    shooterAnglePID = new PIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD);
    trapezoidProfile = new TrapezoidProfile.Constraints(100, 70);
    profiledShooterAnglePID = new ProfiledPIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD, trapezoidProfile);

    shooterAngleMotor.burnFlash();

    // shooterAnglePID.setTolerance(ShooterAngleConstants.P_TOLERANCE);
    profiledShooterAnglePID.setTolerance(2.0);

    shooterAngleFF = new ArmFeedforward(ShooterAngleConstants.kS, ShooterAngleConstants.kG, ShooterAngleConstants.kV);

    pSetpoint = 50;//ShooterAngleConstants.INITIAL_ANGLE;
  }

  /* COMMANDS */
  // default commands
  public Command setManualAngleCmd(double input) {
    // return Commands.print("setting manual angle setpoint");
    return this.run(() -> setManualAngle(input));
  }

  public Command setAngleSetpointCmd(double angle) {
    // return Commands.print("set regular angle setpoint");
    return this.runOnce(() -> setAngleSetpoint(angle)).asProxy();
  }

  public Command setAngleCmd(double angle) {
    return this.run(() -> setAngle(angle)).asProxy();
  }

  public Command setAngleCmd() {
    // return Commands.print("running angle pid");
    return this.run(() -> setAngle());
  }

  // sets shooter angle based on joystick input
  // accounts for the max and min angle limits
  public void setManualAngle(double input) {

    // move up if below max angle
    if (input > 0 && getAngle() < ShooterAngleConstants.SHOOTER_MAX_ANGLE) {
      shooterAngleMotor.set(ShooterAngleConstants.CONSTANT_SPEED);
      pSetpoint = getAngle();
    }
    // move down if above min angle
    else if (input < 0 && getAngle() > ShooterAngleConstants.SHOOTER_MIN_ANGLE) {
      shooterAngleMotor.set(-ShooterAngleConstants.CONSTANT_SPEED);
      pSetpoint = getAngle();
    }
    // run PID
    else {
      setAngle();
      // stopMotor();
    }
  }

  // sets shooter angle to current setpoint
  public void setAngle() {
    double voltage = profiledShooterAnglePID.calculate(getAngle(), pSetpoint);
    double ff = shooterAngleFF.calculate((Math.PI*profiledShooterAnglePID.getSetpoint().position)/180.0, (Math.PI*profiledShooterAnglePID.getSetpoint().velocity)/180.0);

    shooterAngleMotor.setVoltage(ff + voltage); 
    
    /* setangle with original pid */
    // double voltage = shooterAnglePID.calculate(getAngle(), pSetpoint); // with p and i constant
    // shooterAngleMotor.setVoltage(voltage);
    
    System.out.println("angle voltage: " + voltage);
    // // System.out.println("setting angle");
  }

  // for auton commands; overloads setAngle no params
  public void setAngle(double setpoint) {
    setAngleSetpoint(setpoint);
    setAngle();
  }

  // changes setpoint accordingly
  public void setAngleSetpoint(double setpoint) {
    // isManual = false;
    pSetpoint = setpoint;
    System.out.println("shooter angle changed");
  }

  // @Log.NT
  // public double getSetpoint() {
  //   //return shooterAnglePID.getSetpoint();
  //   return profiledShooterAnglePID.getGoal();
  // }

  @Log.NT
  // added physical offset lowest angle is 18.3 deg above the horizontal
  public double getAngle() {
    return shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET;
  }

  // public boolean getIsManual() {
  // return isManual;
  // }

  public void stopMotor() {
    shooterAngleMotor.stopMotor();
  }

  public boolean atAngle() {
    // return shooterAnglePID.atSetpoint();
    return profiledShooterAnglePID.atSetpoint();
  }

  /* SYSID */
  public void setVoltage(double voltage) {
    shooterAngleMotor.setVoltage(voltage);
  }

  public boolean atMaxAngle(){
    return (shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET) >= ShooterAngleConstants.MAX_ANGLE;
  }

  public boolean atMinAngle(){
    return (shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET) <= ShooterAngleConstants.MIN_ANGLE;
  }

  public Command quasiCmd(SysIdRoutine.Direction direction) {
    return angleRoutine.quasistatic(direction);
  }

  public Command dynaCmd(SysIdRoutine.Direction direction) {
    return angleRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("current arm angle", getAngle());
    SmartDashboard.putNumber("desired angle", pSetpoint);

    SmartDashboard.putBoolean("at min angle", atAngle());
    SmartDashboard.putBoolean("at amp angle", atAngle());
  }
}
