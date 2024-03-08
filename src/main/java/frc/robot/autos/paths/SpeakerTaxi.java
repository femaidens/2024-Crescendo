// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.paths;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerTaxi extends SequentialCommandGroup {
  /** Creates a new TaxiSpeaker. */
  public SpeakerTaxi(Drivetrain drivetrain, Hopper hopper, ShooterAngle shooterAngle, ShooterWheel shooterWheel) {
    // starting flush against speaker
    addCommands(
      // shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH),
      // hopper.setVelocitySetpointCmd(IntakeHopperConstants.INTAKE_NOTE_SPEED),
      // shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_FLUSH),
      // Commands.waitUntil(hopper::isHopperEmpty),
      
      // // taxi after shooting - may need to edit speed and direction
      // new RunCommand(() -> drivetrain.drive(0.1, 0, 0, true, false), drivetrain).withTimeout(AutoConstants.TAXISPEAKER_TIME)
    );
  }
}
