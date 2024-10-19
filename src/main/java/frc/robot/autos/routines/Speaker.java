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
public class Speaker extends SequentialCommandGroup {

  /** Creates a new TaxiAmp. */
  public Speaker(Drivetrain drivetrain, Hopper hopper, ShooterAngle shooterAngle, ShooterWheel shooterWheel,
      LED led) {

    addCommands(
        // reset gyro
        new InstantCommand(() -> drivetrain.zeroHeading()),
        // makes sure that it's the proper angle
        // shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.INITIAL_ANGLE),
        // Commands.waitUntil(() -> shooterAngle.atAngle()).withTimeout(4),

        // shoot
        new AutonShoot(ShooterAngleConstants.SPEAKER_FLUSH, 1, ShooterWheelConstants.SPEAKER_FLUSH,
            HopperConstants.TRANSITION_VEL,
            hopper, shooterAngle, shooterWheel), 
        led.setGreenCmd().withTimeout(3) //0.5 after qual 42

        // taxi
        //  new RunCommand(() -> drivetrain.drive(0.15, 0, 0, true, false), drivetrain)
        //     .withTimeout(AutoConstants.TAXI_SPEAKER_TIME) // positive because intake is forward
    );
  }
}
