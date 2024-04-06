// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.paths;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.commands.Intaking;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerTaxiIntakeSpeaker extends SequentialCommandGroup {
  /** Creates a new SpeakerTaxiIntakeSpeaker. */
  public SpeakerTaxiIntakeSpeaker(Drivetrain drivetrain, Intaking intake, Hopper hopper, ShooterAngle shooterAngle, ShooterWheel shooterWheel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.zeroHeading()),
      // set shooter angle and velocity
      shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH)
        .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_FLUSH))
        .alongWith(hopper.setStateLimitCmd(1)),
      new WaitCommand(2),

      //move note
      hopper.setVelocitySetpointCmd(HopperConstants.TRANSITION_VEL),
      Commands.waitUntil(() -> hopper.isHopperEmpty()).withTimeout(2.5),
      shooterWheel.setVelocitySetpointCmd(0).alongWith(hopper.setVelocitySetpointCmd(0)),

      //drive and intake at once, arbritary timeout time
      new ParallelRaceGroup(
        new RunCommand(() -> drivetrain.drive(0.15, 0, 0, true, false), drivetrain)
        .withTimeout(6),
        intake.intakeNote()
      ),
       
      //shoot from the stage
      shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_STAGE)
        .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_STAGE))
        .alongWith(hopper.setStateLimitCmd(1)),
      new WaitCommand(2),

      //move note
      hopper.setVelocitySetpointCmd(HopperConstants.TRANSITION_VEL),
      Commands.waitUntil(() -> hopper.isHopperEmpty()).withTimeout(2.5),
      shooterWheel.setVelocitySetpointCmd(0).alongWith(hopper.setVelocitySetpointCmd(0))
      
    );
  }
}
