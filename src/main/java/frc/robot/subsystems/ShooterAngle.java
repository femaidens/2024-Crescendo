// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Ports.ShooterPorts;
import monologue.Annotations.Log;
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
    trapezoidProfile = new TrapezoidProfile.Constraints(180, 220);
    profiledShooterAnglePID = new ProfiledPIDController(ShooterAngleConstants.kP, ShooterAngleConstants.kI, ShooterAngleConstants.kD, trapezoidProfile);

    shooterAngleMotor.burnFlash();

    // shooterAnglePID.setTolerance(ShooterAngleConstants.P_TOLERANCE);
    profiledShooterAnglePID.setTolerance(2.0);

    shooterAngleFF = new ArmFeedforward(ShooterAngleConstants.kS, ShooterAngleConstants.kG, ShooterAngleConstants.kV);

    pSetpoint = ShooterAngleConstants.INITIAL_ANGLE; 
    //30.0
  }

  /* COMMANDS */
  // default commands
  
  /**
   * Sets shooter angle based on joystick input, currently NOT accounting for max and min angle limits
   * @param input Joystick input, PWM value, positive is up, negative is down
   * @return Parallel Run and Proxy Commands
   * @Note Not sure if this sequence works, especially runonce to set angle after run command
   */
  // accounts for the max and min angle limits
  public void setManualAngle(double input) {

    // move up if below max angle
    if (input > 0 ){//&& getAngle() < ShooterAngleConstants.MAX_ANGLE) {
      shooterAngleMotor.set(ShooterAngleConstants.CONSTANT_SPEED);
      pSetpoint = getAngle();
    }
    // move down if above min angle
    else if (input < 0){// && getAngle() > ShooterAngleConstants.MIN_ANGLE) {
      shooterAngleMotor.set(-ShooterAngleConstants.CONSTANT_SPEED);
      pSetpoint = getAngle();
    }
    // run PID
    else {
      setAngle();
      // stopMotor();
    }
  }

  /**
   * Sets the angle setpoint for shooter angle position, as a proxy
   * @param angle In degrees
   * @return Proxy Command
   */
  public Command setAngleSetpointCmd(double angle) {
    System.out.println("shooter angle changed");
    return this.runOnce(() -> pSetpoint = angle).asProxy();
  }

  /**
   * Not a proxy, sets the anfle setpoint for shooter angle psoition
   * @param angle In degrees
   * @return RunOnce Command
   */
  public Command autonSetAngleSetpointCmd(double angle) {
    System.out.println("shooter angle changed");
    return this.runOnce(() -> pSetpoint = angle);
  }

  /**
   * Runs the shooter angle motor to reach the angle position
   * @param angle In degrees
   * @return Sequential Proxy, then Run Command
   */
  public Command setAngleCmd(double angle) {
    return setAngleSetpointCmd(angle)
    .andThen(setAngleCmd());
  }

  /**
   * Runs the shooter angle motor to reach the setpoint angle, given in the subsystem. Uses profiled PID
   * @return Run Command
   */
  public Command setAngleCmd() {
    return this.run(() -> setAngle());
  }

  /*  METHODS  */

  /**
   * Runs the shooter angle motor to an angle position, given in the subsystem. In degrees
   */
  public void setAngle() {
    double voltage = profiledShooterAnglePID.calculate(getAngle(), pSetpoint);
    double ff = shooterAngleFF.calculate((Math.PI * profiledShooterAnglePID.getSetpoint().position) / 180.0, (Math.PI * profiledShooterAnglePID.getSetpoint().velocity) / 180.0);

    shooterAngleMotor.setVoltage(ff + voltage); 
    
    /* setangle with original pid */
    // double voltage = shooterAnglePID.calculate(getAngle(), pSetpoint); // with p and i constant
    // shooterAngleMotor.setVoltage(voltage);
    
    // System.out.println("angle voltage: " + (ff + voltage));
  }

  /**
   * Sets the fractional duty cycle of the angle motor
   * @param speed PWM value, positive is up, negative is down
   */
  public void setSpeed(double speed){
    shooterAngleMotor.set(speed);
  }

  /**
   * Returns the current measured angle of the shooter pivot
   * @return Double, angle in degrees
   */
  @Log.NT
  public double getAngle() {
    return (shooterAngleEncoder.getPosition() + ShooterAngleConstants.PHYSICAL_OFFSET) % 360;
  }

  /**
   * Returns if the current measured angle is within the threshold of the setpoint
   * @return Boolean
   */
  public boolean atAngle() {
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

    SmartDashboard.putBoolean("at shooter angle", atAngle());
  }
}
