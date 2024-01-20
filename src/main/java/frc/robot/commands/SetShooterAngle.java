// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAngle;

public class SetShooterAngle extends Command {
  private final ShooterAngle shooterAngle;
  private final double angle;
  /** Creates a new SetShooterAngle. */
  public SetShooterAngle(ShooterAngle m_shooterAngle, double m_angle) {
    shooterAngle = m_shooterAngle;
    angle = m_angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterAngle.setShooterAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
