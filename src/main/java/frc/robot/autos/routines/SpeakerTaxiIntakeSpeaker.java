// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
import frc.robot.subsystems.LED;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerTaxiIntakeSpeaker extends SequentialCommandGroup {
  /** Creates a new SpeakerTaxiIntakeSpeaker. */
  public SpeakerTaxiIntakeSpeaker(Drivetrain drivetrain, Intaking intaking, Hopper hopper, ShooterAngle shooterAngle,
      ShooterWheel shooterWheel, LED led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // reset gyro
        new InstantCommand(() -> drivetrain.zeroHeading()),
        // // makes sure that it's the proper angle
        shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.INITIAL_ANGLE),
        Commands.waitUntil(() -> shooterAngle.atAngle()).withTimeout(4),
        // new ShooterSetRPM(shooterWheel, hopper, 200, 1),
        // new ShooterToAngle(shooterAngle, ShooterAngleConstants.SPEAKER_FLUSH), 
        
        // shoot
        // // shoot
        new AutonShoot(ShooterAngleConstants.SPEAKER_FLUSH, ShooterWheelConstants.SPEAKER_FLUSH,
            HopperConstants.TRANSITION_VEL,
            hopper, shooterAngle, shooterWheel),
          
        // //* drive and intake at once, arbritary timeout time
        new TaxiIntake1(drivetrain, intaking, shooterAngle)

        // new AutonShoot(ShooterAngleConstants.SPEAKER_STAGE, ShooterWheelConstants.SPEAKER_STAGE,
        //     HopperConstants.TRANSITION_VEL,
        //     drivetrain, hopper, shooterAngle, shooterWheel)

        // led.setGreenCmd().withTimeout(2)
        //* 
    );
  }
}
// shoot from the stage
    // shooterAngle.autonSetAngleSetpointCmd(ShooterAngleConstants.SPEAKER_STAGE)
    // .alongWith(shooterWheel.autonSetVelocitySetpointCmd(ShooterWheelConstants.SPEAKER_STAGE))
    // .alongWith(hopper.setStateLimitCmd(1)),
    // new WaitCommand(2),
 // //move note
    // hopper.autonSetVelocitySetpointCmd(HopperConstants.TRANSITION_VEL),
    // Commands.waitUntil(() -> hopper.isHopperEmpty()).withTimeout(2.5),
    // shooterWheel.autonSetVelocitySetpointCmd(0).alongWith(hopper.setVelocitySetpointCmd(0))
