package frc.robot.autos.routines;
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
public class BlueRightSpeakerTaxi extends SequentialCommandGroup {

  /** Creates a new TaxiAmp. */
  public BlueRightSpeakerTaxi(Drivetrain drivetrain, Hopper hopper, ShooterAngle shooterAngle,
      ShooterWheel shooterWheel, LED led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // reset gyro
        new InstantCommand(() -> drivetrain.zeroHeading()),

        // makes sure that it's the proper angle
        shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.INITIAL_ANGLE),
        Commands.waitUntil(() -> shooterAngle.atAngle()).withTimeout(4),

        // // shoot
        // shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.INITIAL_ANGLE),
        // Commands.waitUntil(() -> shooterAngle.atAngle()).withTimeout(4),
        // shooterAngle.setAngleSetpointCmd(AutoConstants.AUTON_SPEAKER_FLUSH)//
        // ShooterAngleConstants.SPEAKER_FLUSH)
        // .alongWith(shooterWheel.setVelocitySetpointCmd(AutoConstants.AUTON_SPEAKER_SPEED))//
        // ShooterWheelConstants.SPEAKER_FLUSH))
        // .alongWith(hopper.setStateLimitCmd(1)),
        // // new WaitCommand(2),
        // Commands.waitUntil(() -> shooterAngle.atAngle()).withTimeout(2),
        // hopper.setVelocitySetpointCmd(HopperConstants.TRANSITION_VEL),
        // Commands.waitUntil(() -> hopper.isHopperEmpty()).withTimeout(2.5),
        // shooterWheel.setVelocitySetpointCmd(0).alongWith(hopper.setVelocitySetpointCmd(0)),

        // shoot
        new AutonShoot(AutoConstants.BLUE_RIGHT_FLUSH, AutoConstants.BLUE_RIGHT_WHEEL_VEL,
            AutoConstants.BLUE_RIGHT_HOPPER, 
            hopper, shooterAngle, shooterWheel)
        // led.setGreenCmd().withTimeout(0.5),

        // // taxi
        // new RunCommand(() -> drivetrain.drive(0.15, 0, 0, true, false), drivetrain)
        //     .withTimeout(AutoConstants.TAXI_SPEAKER_TIME) // positive because intake is forward
    );
  }
}