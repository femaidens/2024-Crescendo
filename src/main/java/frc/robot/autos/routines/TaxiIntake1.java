// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.routines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.commands.Intaking;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiIntake1 extends ParallelRaceGroup {
  /** Creates a new TaxiIntake1. */
  public TaxiIntake1(Drivetrain drivetrain, Intaking intaking, ShooterAngle shooterAngle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RunCommand(() -> drivetrain.drive(0.15, 0, 0, true, false), drivetrain)
            .withTimeout(6),

           shooterAngle.autonSetAngleSetpointCmd(ShooterAngleConstants.INTAKE_ANGLE)
            .alongWith(intaking.intakeNote())
            .withTimeout(6));
  }
}
