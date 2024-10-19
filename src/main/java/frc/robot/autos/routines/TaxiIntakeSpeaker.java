// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterWheelConstants;
import frc.robot.commands.Intaking;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterWheel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiIntakeSpeaker extends SequentialCommandGroup {
  /** Creates a new TaxiIntake1. */
  public TaxiIntakeSpeaker(Drivetrain drivetrain, Intaking intaking, ShooterAngle shooterAngle, ShooterWheel shooterWheel, Hopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      shooterAngle.setAngleSetpointCmd(ShooterAngleConstants.INTAKE_ANGLE),

      new RunCommand(() -> drivetrain.drive(0.15, 0, 0, true, false), drivetrain).asProxy().withTimeout(3)
            .alongWith(intaking.intakeNote().asProxy().withTimeout(5)),

      intaking.reverseShooterWheels().asProxy(),

      shooterWheel.setVelocitySetpointCmd(0),

      new AutonShoot(ShooterAngleConstants.SPEAKER_STAGE, 1, ShooterWheelConstants.SPEAKER_FLUSH, HopperConstants.TRANSITION_VEL, hopper, shooterAngle, shooterWheel).asProxy());
  }
}
