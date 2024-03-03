// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HopperConstants;
import frc.robot.Ports.HopperPorts;

public class Hopper extends SubsystemBase {
    private final CANSparkMax hopperMotor;

    private final RelativeEncoder hopperEncoder;

    private final SimpleMotorFeedforward hopperFF;

    private double vHopperSetpoint;

     private final SysIdRoutine hopperRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          volts -> setVoltageHop(volts.in(Units.Volts)), null, this));  
  /** Creates a new Hopper. */
  public Hopper() {
    hopperMotor = new CANSparkMax(HopperPorts.HOPPER_MOTOR, MotorType.kBrushless);
    hopperEncoder = hopperMotor.getEncoder();
    hopperEncoder.setVelocityConversionFactor(HopperConstants.VEL_CFACTOR);
    hopperFF = new SimpleMotorFeedforward(HopperConstants.kS, HopperConstants.kV);
    hopperMotor.setIdleMode(IdleMode.kBrake); // prevent note from slipping out of hopper
    hopperMotor.setSmartCurrentLimit(HopperConstants.CURRENT_LIMIT);
    hopperMotor.burnFlash();

    vHopperSetpoint = 0;
  }

  public void setHopperVelocity() {
    double voltage = hopperFF.calculate(vHopperSetpoint);
    hopperMotor.setVoltage(voltage);
  }

  public void setHopperVelocitySetpoint(double setpoint) {
    vHopperSetpoint = setpoint;
  }

  public void setVoltageHop(double voltage){
    hopperMotor.setVoltage(voltage);
  }

   // for transition between hopper and shooter wheels
   public void setHopperSpeed(double speed) {
    hopperMotor.set(speed);
  }

  public double getHopperVelocity() {
    return hopperEncoder.getVelocity();
  }

  public void stopHopperMotor() {
    hopperMotor.stopMotor();
  }

  public Command hopperQuas(SysIdRoutine.Direction direction) {
    return hopperRoutine.quasistatic(direction);
  }

  public Command hopperDyna(SysIdRoutine.Direction direction) {
    return hopperRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hopper vel", getHopperVelocity());
    SmartDashboard.putNumber("hopper sp", vHopperSetpoint);
  }
}
