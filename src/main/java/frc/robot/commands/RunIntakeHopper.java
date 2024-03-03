// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
// import frc.robot.commands.RunIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIntakeHopper extends ParallelCommandGroup {
  /** Creates a new runIntakeHopper. */
  // private Intake intake;
  // private Hopper hopper;
  public RunIntakeHopper(Intake intake, Hopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    intake = new Intake();
    hopper = new Hopper();
    addCommands(
      new RunIntake(intake),
      new RunHopper(hopper)
    );
  }
}
