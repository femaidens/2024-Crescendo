// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonShoot extends SequentialCommandGroup {
  /** Creates a new AutonShoot. */
  public AutonShoot(double angle, double wheelVel, double hopperVel, Hopper hopper, ShooterAngle shooterAngle, ShooterWheel shooterWheel) {
    
    addCommands(
      // shoots into the speaker
      shooterWheel.setVelocitySetpointCmd(wheelVel),
      hopper.setStateLimitCmd(1),

      shooterAngle.setAngleSetpointCmd(angle),

      Commands.waitUntil(shooterAngle::atAngle).withTimeout(3),
  
      hopper.setVelocitySetpointCmd(hopperVel),

      Commands.waitUntil(() -> hopper.isHopperEmpty()).withTimeout(3),
      shooterWheel.setVelocitySetpointCmd(0).alongWith(hopper.setVelocitySetpointCmd(0))
    );
  }
}
