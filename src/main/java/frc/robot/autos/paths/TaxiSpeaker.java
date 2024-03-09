package frc.robot.autos.paths;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.*;
// import frc.robot.autos.AutoDrive;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiSpeaker extends SequentialCommandGroup {

  /** Creates a new TaxiAmp. */
  public TaxiSpeaker(Drivetrain drivetrain, Hopper hopper, ShooterAngle shooterAngle, ShooterWheel shooterWheel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.SPEAKER_FLUSH)
        .alongWith(shooterWheel.setVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_FLUSH))
        .alongWith(hopper.setStateLimitCmd(1)),
      new WaitCommand(1),
      // Commands.waitUntil(() -> shooterAngle.atAngle())
      hopper.setVelocitySetpointCmd(IntakeHopperConstants.INTAKE_NOTE_SPEED),
      Commands.waitUntil(() -> hopper.isHopperEmpty()).withTimeout(1.5),
      shooterWheel.setVelocitySetpointCmd(0).alongWith(hopper.setVelocitySetpointCmd(0)),
      new RunCommand(() -> drivetrain.drive(0.15, 0, 0, true, false), drivetrain)
        .withTimeout(AutoConstants.DRIVE_TIME) // positive because intake is forward
    );
  }
}