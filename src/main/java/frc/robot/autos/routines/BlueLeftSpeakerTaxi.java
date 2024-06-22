// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/*
 * This is for THE LEFT SIDE, WHEN IT MAY RUN INTO THE AMP, where it wouldn't exceed the black taxi line
 */
public class BlueLeftSpeakerTaxi extends SequentialCommandGroup {
  /** Creates a new BlueLeftSpeakerTaxi. */
  public BlueLeftSpeakerTaxi(Drivetrain drivetrain, Hopper hopper, ShooterAngle shooterAngle, ShooterWheel shooterWheel, LED led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // reset gyro
      new InstantCommand(() -> drivetrain.zeroHeading()),

      // makes sure that it's the proper angle
      shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.INITIAL_ANGLE),
      Commands.waitUntil(() -> shooterAngle.atAngle()).withTimeout(3),

      new AutonShoot(AutoConstants.BLUE_RIGHT_FLUSH, AutoConstants.BLUE_RIGHT_WHEEL_VEL,
      AutoConstants.BLUE_RIGHT_HOPPER, 
      hopper, shooterAngle, shooterWheel),

       new RunCommand(() -> drivetrain.drive(0.15, -0.10, 0, true, false), drivetrain)
            .withTimeout(AutoConstants.TAXI_SPEAKER_TIME) // positive because intake is forward

    );
  }
}
